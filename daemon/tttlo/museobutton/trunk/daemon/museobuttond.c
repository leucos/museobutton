/**
 * @file museobuttond.c
 * @brief Main daemon code
 *
 * @author Michel Blanc
 * @date 2008-02-6
 *
 * @version $Id: main.c 389 2007-08-07 11:13:12Z cs $
 *
 * - Project: museobutton
 * - Tabsize: 2
 * - Copyright: (c) 2008 ERASME
 * - License: GNU GPL v2
 * - Part of this code (c) Objective Development Gmbh (USB)
 *
 * This program controls the microcontroller via USB, and
 * receive it's orders in OSC. It also sends OSC events
 * when the capacitive button is touched/released.
 *
 * @refs AvrUsb: http://www.obdev.at/products/avrusb/index.html
 * @sa main.c
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <glib.h>
#include <sys/stat.h>
#include <syslog.h>
#include <pthread.h>

#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */

#include <lo/lo.h>

#define USBDEV_DEFAULT_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_DEFAULT_SHARED_PRODUCT   0x05DD  /* Obdev's free shared PID */

#define OSC_DEFAULT_LOCAL_PORT 1234

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

#define VERSION  0.01

#define USBDELAY 20000

/** 
 * @brief List of available USB commands.
 * 
 * The same enum must be present in the firmware's main.c
 * @sa main.c
 */
enum USB_COMMANDS {
  USB_ECHO,              /**< a simple echo test         */
  USB_GET_ID,            /**< the device internal ID     */
  USB_SET_ID,            /**< the device internal ID     */
  USB_SET_RED,           /**< sets red component         */
  USB_SET_GREEN,         /**< sets green component       */
  USB_SET_BLUE,          /**< sets blue component        */
  USB_SET_PERIOD,        /**< sets wave period           */
  USB_SET_COLOR_PERIOD,  /**< sets wave period for color */
  USB_SET_WAVEFORM,      /**< sets waveform              */
  USB_SET_COLOR_WAVEFORM,/**< sets waveform for color    */
  USB_SET_DUTY_CYCLE,
  USB_GET_BUTTON,        /**< gets button state          */
  USB_GET_TEMP,          /**< gets temperature           */
};

typedef enum WAVEFORMS {
  PULSEMODE_NONE,
  PULSEMODE_TRIANGLE,
  PULSEMODE_SAWTOOTH_UP,
  PULSEMODE_SAWTOOTH_DOWN,
  PULSEMODE_SIN,
  PULSEMODE_SINE = PULSEMODE_SIN,
  PULSEMODE_SUBPWM
} waveform_t;

int oDebug = FALSE;
int oDaemonize = TRUE;
pthread_mutex_t usbmutex = PTHREAD_MUTEX_INITIALIZER;

static void 
debug(int level, const char * template, ...)
{
  char buffer[1024];
  va_list ap;

  va_start (ap, template);

  if (oDaemonize) {
    vsprintf (buffer, template, ap);
    syslog(level, buffer);
  } else {
    vfprintf(stderr, template, ap);
    fprintf(stderr,"\n");
    fflush(stderr);
  }

  va_end (ap);
}


static void 
daemonize(void)
{
  pid_t pid, sid;

  /* already a daemon */
  if ( getppid() == 1 ) return;
  
  /* Fork off the parent process */
  pid = fork();
  if (pid < 0) {
    exit(EXIT_FAILURE);
  }
  /* If we got a good PID, then we can exit the parent process. */
  if (pid > 0) {
    exit(EXIT_SUCCESS);
  }
  
  /* At this point we are executing as the child process */
  fprintf(stdout,"Daemonization done. Messages are now logged to syslog.\n");
  
  /* Change the file mode mask */
  umask(0);
  
  /* Create a new SID for the child process */
  sid = setsid();
  if (sid < 0) {
    exit(EXIT_FAILURE);
  }
  
  /* Change the current working directory.  This prevents the current
     directory from being locked; hence not being able to remove it. */
  if ((chdir("/")) < 0) {
    exit(EXIT_FAILURE);
  }

  /* Redirect standard files to /dev/null */
  freopen( "/dev/null", "r", stdin);
  freopen( "/dev/null", "w", stdout);
  freopen( "/dev/null", "w", stderr);
}

static int 
usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen)
{
  char    buffer[256];
  int     rval, i;
  
  if((rval = usb_control_msg(dev, USB_ENDPOINT_IN, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, langid, buffer, sizeof(buffer), 1000)) < 0)
    return rval;
  if(buffer[1] != USB_DT_STRING)
    return 0;
  if((unsigned char)buffer[0] < rval)
    rval = (unsigned char)buffer[0];
  rval /= 2;
  /* lossy conversion to ISO Latin1 */
  for(i=1;i<rval;i++){
    if(i > buflen)  /* destination buffer overflow */
      break;
    buf[i-1] = buffer[2 * i];
    if(buffer[2 * i + 1] != 0)  /* outside of ISO Latin1 range */
      buf[i-1] = '?';
  }
  buf[i-1] = 0;
  return i-1;
}


/**
 * @brief : USB device lookup and opening
 *
 * usbOpenDevice Enumerates all USB devices and tries to open
 * our device, specified by vendorId and productId.
 *
 * @param[out] device This pointer will be filled with a handle to our device for future operations
 * @param[in] vendor The vendorID for the device we want to open
 * @param[in] vendor The productID for the device we want to open
 * @param[in] vendorName The vendor name we want to open. We'll use NULL, since we'll be searching by ID.
 * @param[in] productName The product name we want to open. We'll use NULL, since we'll be searching by ID.
 *
 * @return 0 on success, something else in cas of error
 *
 * @remark This code is 100% take from @ref PowerSwitch http://www.obdev.at/products/avrusb/powerswitch.html
 *
 */

int usbOpenDevice(usb_dev_handle **device, int vendor, char *vendorName, int product, char *productName)
{
  struct usb_bus      *bus;
  struct usb_device   *dev;
  usb_dev_handle      *handle = NULL;
  int                 errorCode = USB_ERROR_NOTFOUND;
  static int          didUsbInit = 0;
  
  if(!didUsbInit){
    didUsbInit = 1;
    usb_init();
  }
  usb_find_busses();
  usb_find_devices();
  for(bus=usb_get_busses(); bus; bus=bus->next){
    for(dev=bus->devices; dev; dev=dev->next){
      if(dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product){
        char    string[256];
        int     len;
        handle = usb_open(dev); /* we need to open the device in order to query strings */
        if(!handle){
          errorCode = USB_ERROR_ACCESS;
          debug(LOG_WARNING, "cannot open USB device: %s", usb_strerror());
          continue;
        }
        if(vendorName == NULL && productName == NULL){  /* name does not matter */
          break;
        }
        /* now check whether the names match: */
        len = usbGetStringAscii(handle, dev->descriptor.iManufacturer, 0x0409, string, sizeof(string));
        if(len < 0){
          errorCode = USB_ERROR_IO;
          debug(LOG_WARNING, "cannot query manufacturer for device: %s", usb_strerror());
        }else{
          errorCode = USB_ERROR_NOTFOUND;
          debug(LOG_DEBUG, "seen device from vendor ->%s<-", string);
          if(strcmp(string, vendorName) == 0){
            len = usbGetStringAscii(handle, dev->descriptor.iProduct, 0x0409, string, sizeof(string));
            if(len < 0){
              errorCode = USB_ERROR_IO;
              debug(LOG_WARNING, "cannot query product for device: %s", usb_strerror());
            }else{
              errorCode = USB_ERROR_NOTFOUND;
              debug(LOG_DEBUG, "seen product ->%s<-", string);
              if(strcmp(string, productName) == 0)
                break;
            }
          }
        }
        usb_close(handle);
        handle = NULL;
      }
    }
    if(handle)
      break;
  }
  if(handle != NULL){
    errorCode = 0;
    *device = handle;
  }
  return errorCode;
}

/**
 * @brief : Usage of command line switches
 *
 * Usually called when a wrong command line switch is used, or
 * when a mandatory option is missing (but there is not as of now).
 */
void 
usage()
{
  fprintf(stdout, "museobuttond, version %.2f\n\n",VERSION);
  fprintf(stdout, "  Usage : museobuttond -[tfgh] -v <id> -p <id> -l <port> -r <port> -d <ip> [id0] [id1] [...]\n\n");

  fprintf(stderr, "\t-h : this help\n");
  fprintf(stderr, "\t-t : run initial USB tests (default : no)\n");
  fprintf(stderr, "\t-f : run foreground, do not daemonize (default : no)\n");
  fprintf(stderr, "\t-g : debug (default : no)\n");
  fprintf(stderr, "\t-v <id> : USB vendor ID (default : USBDEV_DEFAULT_SHARED_VENDOR\n");
  fprintf(stderr, "\t-p <id> : USB product ID (default : USBDEV_DEFAULT_SHARED_PRODUCT\n");
  fprintf(stderr, "\t-l <port> : local OSC port (default 1234)\n");
  fprintf(stderr, "\t-r <port> : remote OSC port (default 1234)\n");
  /** @todo : make broadcasting the default behaviour */
  fprintf(stderr, "\t-d <ip> : remote OSC machine (default : none)\n");
  fprintf(stderr, "\t-s <id> : local sensor ID (default : none)\n");
  fprintf(stderr, "\t[id0] ... : list of IDs to open (default : first found)\n");

  fprintf(stderr,"\n");
}

/**
 * @brief Run USB tests
 *
 * This test is called when the program is invoked with '-t'.
 * It simply sends 100 random numbers over USB, and 
 * waits for echo.
 *
 * @param handle USB  handle
 *
 * @return TRUE on success, FALSE on failure
 */ 
int 
runtest(usb_dev_handle *handle) 
{
  int v, r, i, nBytes;
  unsigned char buffer[8];

  /* The test consists of writing 100 random numbers to the device and checking
   * the echo. This should discover systematic bit errors (e.g. in bit stuffing).
   */
  for(i=0;i<1000;i++){
    fprintf(stdout,"."); 
    fflush(stdout);

    v = rand() & 0xffff;
    nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USB_ECHO, v, 0, (char *)buffer, sizeof(buffer), 5000);
    if(nBytes < 2){
      if(nBytes < 0)
        debug(LOG_ERR,"USB error: %s", usb_strerror());
      debug(LOG_ERR,"only %d bytes received in iteration %d", nBytes, i);
      exit(1);
    }
    r = buffer[0] | (buffer[1] << 8);
    if(r != v){
      debug(LOG_WARNING,"data error: received 0x%x instead of 0x%x in iteration %d", r, v, i);
      return FALSE;
    }
  }

  return TRUE;
}


void usb_error(usb_dev_handle* handle, const char* caller) {

  debug(LOG_ERR, "fatal USB error in %s", caller);
  usb_close(handle);

  exit(EXIT_FAILURE);
}

/** 
 * @brief Complain callback for OSC errors
 *
 * This function is called when an error occurs with OSC communications
 */

void 
lo_error(int num, const char *msg, const char *path)
{
  debug(LOG_ERR, "liblo server error %d in path %s: %s", num, path, msg);
}

/**
 * @brief Set LEDs RGV values via USB
 *
 * Controls the device and set the max R, G and B values.
 * These values will cap all possible values taken by the 
 * timer compare values, regardless of the waveform used.
 * So the waveform is constrained on each color channel
 * from here.
 *
 * set_led being an OSC callback, several parameters are compulsory, but unused and
 * not listed.
 *
 * @param user_data Contains a voided pointer do usb handle.
 * @param argv Data received via OSC, containing the 3 color values
 */
int 
set_led(const char *path, const char *types, lo_arg **argv, int argc,
        void *data, void *user_data)
{
  unsigned char       buffer[8];
  char* wave = &argv[0]->s;

  debug(LOG_DEBUG,"setting color #%.2x%.2x%.2x over %s with period of %d x 0.1 secs (%.2f Hz)",
        argv[2]->i,
        argv[3]->i,
        argv[4]->i, 
        wave,
        (int)(argv[1]->f), 
        (10/argv[1]->f));

  debug(LOG_DEBUG, "sending waveform");

  pthread_mutex_lock(&usbmutex);

  usleep(USBDELAY);

  if (!strcmp("sine", wave)) {
    usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                    USB_SET_WAVEFORM, PULSEMODE_SIN, 0, (char *)buffer, sizeof(buffer), 5000);
  } else if (!strcmp("triangle",  wave)) {
    usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                    USB_SET_WAVEFORM, PULSEMODE_TRIANGLE, 0, (char *)buffer, sizeof(buffer), 5000);
  } 

  usleep(USBDELAY);

  debug(LOG_DEBUG, "sending red value");
  usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                  USB_SET_RED, argv[2]->i, 0, (char *)buffer, sizeof(buffer), 5000);

  usleep(USBDELAY);

  debug(LOG_DEBUG, "sending green value");
  usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                  USB_SET_GREEN, argv[3]->i, 0, (char *)buffer, sizeof(buffer), 5000);

  usleep(USBDELAY);

  debug(LOG_DEBUG, "sending blue value");
  usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                  USB_SET_BLUE, argv[4]->i, 0, (char *)buffer, sizeof(buffer), 5000);

  usleep(USBDELAY);
 
  debug(LOG_DEBUG, "sending period value");
  usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                  USB_SET_PERIOD, (int)(argv[1]->f), 0, (char *)buffer, sizeof(buffer), 5000);

  usleep(USBDELAY);
  pthread_mutex_unlock(&usbmutex);


  return TRUE;
}

/** @brief Returns the current button state
 *
 * @param handle USB handle
 * @returns the button state (0|1) and -1 if error
 **/
int get_button(usb_dev_handle* handle) 
{
  unsigned char buffer[8];

  pthread_mutex_lock(&usbmutex);

  usleep(USBDELAY);
  int nBytes = usb_control_msg(handle, 
                           USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USB_GET_BUTTON, 
                           0, 
                           0, 
                           (char *)buffer, 
                           sizeof(buffer), 
                           5000);
  usleep(USBDELAY);
  pthread_mutex_unlock(&usbmutex);

  if(nBytes < 0) {
    debug(LOG_ERR, "error in USB control transfer: %s\n", usb_strerror());
    usb_error(handle, "get_button");
  }
  if(nBytes == 0) // no more data 
    return -1;
  if(nBytes != 1){
    debug(LOG_WARNING, "data format error in get_button, only %d bytes received (1 expected)\n", nBytes);
    return -1;
  }

  return (int)buffer[0];
}

int get_temperature(usb_dev_handle* handle)
{
  unsigned char buffer[8];

  pthread_mutex_lock(&usbmutex);

  int nBytes = usb_control_msg(handle, 
                              USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USB_GET_TEMP, 
                              0, 
                              0, 
                              (char *)buffer, 
                              sizeof(buffer), 
                              5000);
  usleep(USBDELAY);
  pthread_mutex_unlock(&usbmutex);

  if(nBytes < 0) {
    debug(LOG_WARNING, "error in USB control transfer: %s\n", usb_strerror());
    usb_error(handle, "get_temperature");
  }
  if(nBytes == 0) // no more data 
    return -127;
  if(nBytes != 2){
    debug(LOG_WARNING, "data format error in get_temperature, only %d bytes received (2 expected)\n", nBytes);
    return -127;
  } else
    return (buffer[0] << 8) | buffer[1]; 
}

/**
 * @brief Well, this is main...
 *
 * Just orchestrating stuff : daemonization, OSC callback set-up, etc...
 *
 * @return Hopefuly not
 */
int 
main(int argc, char **argv)
{
  unsigned char       buffer[8];
  char                osc_path[256];
  int                 nBytes;
  char                c;
  lo_server_thread    st;
  lo_address          osc_dest;
  int                 button_last_state = 0;
  int                 button_current_state = 0;
  int                 temperature = 0;

  usb_dev_handle      *handle = NULL;

  /* Options */
  char *              oOscLocalPort = NULL;
  char *              oOscRemotePort = NULL;
  char *              oOscDestination = NULL;

  int                 oVendor = USBDEV_DEFAULT_SHARED_VENDOR;
  int                 oProduct = USBDEV_DEFAULT_SHARED_PRODUCT;

  int                 oRunTest = FALSE;
  int                 oSensorId;

  while ((c = getopt(argc, argv, "htfv:p:s:l:r:d:g")) != -1 ) {
    switch (c) {
    case 's':
      oSensorId = atoi(optarg);
      break;
    case 'v':
      oVendor = atoi(optarg);
      break;
    case 'p':
      oProduct = atoi(optarg);;
      break;
    case 't':
      oRunTest = TRUE;
      break;
    case 'f':
      oDaemonize = FALSE;
      break;
    case 'l': /* ## free needed */
      oOscLocalPort = g_malloc(strlen(optarg));
      strcpy(oOscLocalPort,optarg);
      break;
    case 'r': /* ## free needed */
      oOscRemotePort = g_malloc(strlen(optarg));
      strcpy(oOscRemotePort,optarg);
      break;
    case 'd': /* ## free needed */
      oOscDestination = g_malloc(strlen(optarg));
      strcpy(oOscDestination,optarg);
      break;
    case 'g':
      oDebug = TRUE;
      break;
    case '?':
      if (isprint (optopt))
          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
      else
        fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
    case 'h':
    default:
      usage();
      exit(EXIT_FAILURE);
    }
  }

  
  if (oDaemonize) {
    daemonize();
  }

  openlog("museobuttond", LOG_PID, LOG_LOCAL5);
  debug(LOG_INFO, "opening device %d:%d", oVendor, oProduct);

  
  if (usbOpenDevice(&handle, oVendor, NULL, oProduct, NULL)) {
    debug(LOG_ERR, "unable to open any device matching VID/PID %d/%d", oVendor, oProduct);
    exit(EXIT_FAILURE);
  }

  if(usb_set_configuration(handle, 1) < 0){
    debug(LOG_ERR, "error setting USB configuration: %s", usb_strerror());
  }
  
  if(usb_claim_interface(handle, 0) < 0){
    debug(LOG_ERR, "error setting USB interface: %s", usb_strerror());
  }
  
  if(oRunTest){
    if (runtest(handle))
      debug(LOG_NOTICE, "echo test successded");
    else
      debug(LOG_NOTICE, "echo test failure");
  }

  /* start OSC server */
  debug(LOG_NOTICE, "starting OSC server");
  
  st = lo_server_thread_new(oOscLocalPort, lo_error);
  sprintf(osc_path,"/erasme/device/led/%d/set", oSensorId);
  lo_server_thread_add_method(st, osc_path, "sfiii", set_led, (void *)handle);
  lo_server_thread_start(st);
  
  /* OSC destination */
  //osc_dest = lo_address_new("192.168.0.228", "1235");
  osc_dest = lo_address_new(oOscDestination, oOscRemotePort);

  debug(LOG_NOTICE, "OSC server started");

  for (;;) {
    for(;;) {   

      //button_current_state = get_button(handle);
      
      if (button_last_state != button_current_state) {
        button_last_state = button_current_state;
        debug(LOG_NOTICE, "button is now %s", (button_current_state ? "on" : "off"));

        if (button_current_state) {
          sprintf(osc_path,"/erasme/sensor/keyboard/%d/press", oSensorId);
          debug(LOG_NOTICE, "sending OSC notification %s", osc_path);
          if (lo_send(osc_dest, osc_path, "i", 0) == -1) {
            debug(LOG_WARNING, "OSC error %d: %s\n", lo_address_errno(osc_dest), lo_address_errstr(osc_dest));
          }
        } else {
          sprintf(osc_path,"/erasme/sensor/keyboard/%d/release", oSensorId);
          debug(LOG_NOTICE, "sending OSC notification %s", osc_path);
          if (lo_send(osc_dest, osc_path, "i", 0) == -1) {
            debug(LOG_WARNING, "OSC error %d: %s\n", lo_address_errno(osc_dest), lo_address_errstr(osc_dest));
          }
        }
      }
      
      
        temperature = get_temperature(handle);
      sprintf(osc_path,"/erasme/sensor/temperature/%d/release", oSensorId);

      debug(LOG_NOTICE, "sending OSC notification %s", osc_path);

      if (lo_send(osc_dest, osc_path, "i", 0) == -1) {
        printf("OSC error %d: %s\n", lo_address_errno(osc_dest), lo_address_errstr(osc_dest));
      };
      
      
      sleep(2);   
    }

    // wait for interrupt, set timeout to more than a week 
    pthread_mutex_lock(&usbmutex);
    nBytes = usb_interrupt_read(handle, USB_ENDPOINT_IN | 1 , 
                                (char *)buffer, sizeof(buffer), 700000 * 1000); 
    usleep(USBDELAY);
    pthread_mutex_unlock(&usbmutex);

    if(nBytes < 0){
      debug(LOG_WARNING, "error in USB interrupt read: %s\n", usb_strerror());
      usb_error(handle,"main loop");
    }
    
  }

  return 0;
}
  
/* -------------------*/
/* Some documentation */

/** @mainpage My Personal Index Page
 *
 * @section intro_sec Introduction
 *
 * This is the introduction.
 *
 * @section install_sec Installation
 *
 * @subsection step1 Step 1: Opening the box
 *  
 * etc...
 */

/** @page 1_circuits Schematics & PCB description
 * Schematics & PCB board are included for the latest version only (6.4).
 *
 * @section schematics_1_1 Schematics
 * This page contains schemtics for the @ref sch_circuitry_1_1_1 "main circuitry" (µcontroller, LED driver,
 * USB port), the electrode the subsections @ref sch_circuitry_1_1_1, @ref sch_electrode_1_1_2
 * and @ref sch_led_1_1_3.
 *
 * @subsection sch_circuitry_1_1_1 The first subsection
 * Text.
 * @subsection sch_electrode_1_1_2 The second subsection
 * More text.
 * @subsection sch_led_1_1_3 The second subsection
 *
 * @section drillingmap_1_2 Drilling map
 * The drilling map is meant to be used on Multicomp MB6 boxes.
 *
 * The PDF can be downloaded @ref here http://reseau.erasme.org/static/museobutton/percage.svg.pdf
 * . The original SVG file is @ref here http://reseau.erasme.org/static/museobutton/percage.svg
 *
 */

/*! \page page2 Another page
  Even more info.
*/
