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
//#include <pthread.h>
//#include <glib.h>
#include <fcntl.h> 
#include <termios.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <usb.h>    /* this is libusb, see http://libusb.sourceforge.net/ */

#include <lo/lo.h>

#define USBDEV_DEFAULT_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_DEFAULT_SHARED_PRODUCT   0x05DD  /* Obdev's free shared PID */

#define OSC_DEFAULT_LOCAL_PORT 1234

#define USB_ERROR_NOTFOUND  1
#define USB_ERROR_ACCESS    2
#define USB_ERROR_IO        3

#define VERSION  0.01

#define USB_DELAY    20000 /* µs */
#define SERIAL_DELAY 20000 /* µs */

#define SERIAL_MODE 0
#define USB_MODE    1

/** 
 * @brief Boolean type 
 *
 */
#ifdef TRUE
#  ifndef boolean_t
#    define boolean_t int
#  endif
#else
#  ifdef boolean_t
#    define FALSE 0
#    define TRUE 1
#  else
     typedef enum {FALSE, TRUE} boolean_t;
#  endif
#endif

/** 
 * @brief List of available USB commands.
 * 
 * The same enum must be present in the firmware's main.c
 * @sa main.c
 */
typedef enum USB_COMMANDS {
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
} usb_command_t;

typedef enum WAVEFORMS {
  PULSEMODE_NONE,
  PULSEMODE_TRIANGLE,
  PULSEMODE_SAWTOOTH_UP,
  PULSEMODE_SAWTOOTH_DOWN,
  PULSEMODE_SIN,
  PULSEMODE_SINE = PULSEMODE_SIN,
  PULSEMODE_SUBPWM
} waveform_t;

typedef struct usb_message {
  usb_command_t cmd;
  char          data[8];
  uint8_t       len;

} usb_message_t;

typedef struct async_queue_config {
  GAsyncQueue    *to_device;
  GAsyncQueue    *from_device;
  usb_dev_handle *device_handle;
} async_queue_config_t;


boolean_t oDebug = FALSE;
boolean_t oDaemonize = TRUE;

static void 
debug(int level, const char * template, ...)
{
  char buffer[1024];
  va_list ap;

  va_start (ap, template);

  if (oDaemonize) {
    vsprintf (buffer, template, ap);
    syslog(level, "%s", buffer);
  } else {
    if (oDebug < level) return;
    vfprintf(stderr, template, ap);
    fprintf(stderr,"\n");
    fflush(stderr);
  }

  va_end (ap);

  return;
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
  freopen("/dev/null", "r", stdin);
  freopen("/dev/null", "w", stdout);
  freopen("/dev/null", "w", stderr);
}

/**
 * @brief : Initialize the serial port with some tweaks.
 *
 * Open the serial port and set parameters for it.
 * Mainly, it is set at 8 data bits, no parity, 1 stop bit.
 * Port is also set RAW, and VMIN is set to 2 since all data
 * that need to be read has even bytes.
 *
 * @param fd 
 *  The file descriptor to read from
 * @param baudrate 
 *  The baudrate to set
 * @return 
 *  TRUE if happy, FALSE if something went wrong.
 * @warning 
 *  This function should not be used externally
 */

boolean_t 
serialInitPort(int fd, int baudrate) 
{

	// use :   fd = open(port, O_RDWR | O_NOCTTY);

	struct termios options;

  debug(LOG_DEBUG,"Setting port attributes");

	/* Get the current options for the port */
	tcgetattr(fd, &options);

	/* Set the baud rates to BAUDRATE */
  if (cfsetispeed(&options, baudrate) == -1)
    debug(LOG_WARNING,"cfsetispeed error ");

	if (cfsetospeed(&options, baudrate))
    debug(LOG_WARNING,"cfsetospeed error ");

  /* 8N1 */
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  /* No hardware flow control */
  options.c_cflag &= ~CRTSCTS;
  
  /* Turn on READ & ignore ctrl lines */
  options.c_cflag |= CREAD | CLOCAL;  

  /* No software flow control */
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  
  /* Make raw */
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  
  /* read() will block until there's 26 bytes of data available
   * see: http://unixwiz.net/techtips/termios-vmin-vtime.html */
  options.c_cc[VMIN]   = 2; 
  options.c_cc[VTIME]  = 0; 
  
	/* Apply the new options for the port */
	tcsetattr(fd, TCSANOW, &options);

  return TRUE;
}

/**
 * @brief : Serial device lookup and opening
 *
 * opens device given in parameters. If device is NULL, 
 * this function will try to autodetect the port (NOT UMPLEMENTED).
 *
 * @param[out] fd The file descriptor to read to/write from is open is successful.
 * @param[in] port The serial rort device to use (e.g. "/dev/ttyUSB1")
 * @param[in] baudrate The communication baud rate (only 9600, 19200 and 57600 are supported)
 *
 * @return 0 on failure, a real fd number otherwise.
 *
 */

int 
serialOpen(const char *port, int baudrate) {
  
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	//	fcntl(fd, F_SETFL, FNDELAY);

  debug(LOG_DEBUG,"Opening port %s...", port);

  if (fd == -1) {
		debug(LOG_ERR,"SERIAL error: port %s opening failed. Permission problem ?", port);
    exit(1);
  } 

  debug(LOG_INFO,"Port %s opened successfuly", port);

  /* Set port attributes (baudrate only) */
  if (baudrate == 57600)
    serialInitPort(fd, B57600);
  else if (baudrate==19200) 
    serialInitPort(fd, B19200);
  else if (baudrate==9600)
    serialInitPort(fd, B9600); 
  else {
		debug(LOG_ERR,"SERIAL error: unsupported baud rate (only 9600, 19200 and 57600 are supported)");
    exit(1);
  }

  debug(LOG_INFO,"Baudrate set successfuly to %d", baudrate);

	return fd;
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
  fprintf(stderr, "\t-g <0-7> : debug level (default : no debug, see sys/syslog.h for levels)\n");
  fprintf(stderr, "\t-u : USB mode (default : serial mode)\n");
  fprintf(stderr, "\t-P <port> : serial port (default : none\n");
  fprintf(stderr, "\t-b <rate> : serial baud rate (default : 9600; use 9600, 19200 or 57600)\n");
  fprintf(stderr, "\t-v <id> : USB vendor ID (default : USBDEV_DEFAULT_SHARED_VENDOR\n");
  fprintf(stderr, "\t-p <id> : USB product ID (default : USBDEV_DEFAULT_SHARED_PRODUCT\n");
  fprintf(stderr, "\t-l <port> : local OSC port (default 1234)\n");
  fprintf(stderr, "\t-r <port> : remote OSC port (default 1235)\n");
  /** @todo : make broadcasting the default behaviour */
  fprintf(stderr, "\t-d <ip> : remote OSC machine (default 127.0.0.1)\n");
  fprintf(stderr, "\t-s <id> : local sensor ID (default : 0)\n");
	//  fprintf(stderr, "\t[id0] ... : list of IDs to open (default : first found)\n");
  fprintf(stderr, "\n  Example : ./museobuttond -f -g7 -P /dev/ttyUSB0 -b9600 -l1234 -r1235 -d127.0.0.1\n");

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
boolean_t 
usbRunTest(usb_dev_handle *handle) 
{
  int v, r, i, nBytes;
  unsigned char buffer[8];

  /* The test consists of writing 100 random numbers to the device and checking
   * the echo. This should discover systematic bit errors (e.g. in bit stuffing).
   */
  for(i=0;i<100;i++){
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


void usbError(usb_dev_handle* handle, const char* caller) {

  debug(LOG_ERR, "fatal USB error in %s", caller);
  sleep(1);

  if (handle) {
    usb_reset(handle);
    usb_close(handle);
  }

  usleep(USB_DELAY);

  if (usbOpenDevice(&handle, USBDEV_DEFAULT_SHARED_VENDOR, NULL, USBDEV_DEFAULT_SHARED_PRODUCT, NULL)) {
    debug(LOG_ERR, "unable to reopen device");
    exit(EXIT_FAILURE);
  }

  return;

  usb_close(handle);

  exit(EXIT_FAILURE);
}

/** 
 * @brief Complain callback for OSC errors
 *
 * This function is called when an error occurs with OSC communications
 */

void 
loError(int num, const char *msg, const char *path)
{
  debug(LOG_ERR, "liblo server error %d in path %s: %s", num, path, msg);
}


void *
usbTalker(usb_dev_handle* handle, usb_message_t* message) {
  char buffer[8];
  int nBytes;

  debug(LOG_NOTICE, "usbTalker : cmd %.2x, data %d (%.2x)", message->cmd, (uint8_t)message->data[0], (uint8_t)message->data[0]);

  switch (message->cmd) {
  case USB_ECHO:
  case USB_GET_ID:
  case USB_GET_BUTTON:
  case USB_GET_TEMP:
    debug(LOG_DEBUG, "issuing USB_ENDPOINT_IN message");
    nBytes = usb_control_msg(handle,
                             USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
                             message->cmd,
                             (uint8_t)message->data[0], 
                             0, 
                             (char *)message->data, 
                             sizeof(message->data), 
                             5000);

    //    nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USB_ECHO, v, 0, (char *)buffer, sizeof(buffer), 5000);

    if(nBytes < 0) {
      debug(LOG_WARNING, "error in USB control transfer: %s\n", usb_strerror());
      usbError(handle, "usbTalker");
      g_free(message);
      message = NULL;
    }
    if(nBytes != message->len){
      debug(LOG_WARNING, "data format error in usbTalker : only %d bytes received (%d expected)\n", nBytes, message->len);
      g_free(message);
      message = NULL;
    } else {
      debug(LOG_DEBUG, "got USB_ENDPOINT_IN reply");  
    }
    break;
    
  case USB_SET_ID:
  case USB_SET_RED:
  case USB_SET_GREEN:
  case USB_SET_BLUE:
  case USB_SET_PERIOD:
  case USB_SET_COLOR_PERIOD:
  case USB_SET_WAVEFORM:
  case USB_SET_COLOR_WAVEFORM:
  case USB_SET_DUTY_CYCLE:
    debug(LOG_DEBUG, "issuing USB_ENDPOINT_OUT message");
    usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
                    message->cmd, 
                    message->data[0], 
                    0, 
                    (char *)buffer, 
                    sizeof(buffer), 
                    5000);
    usleep(USB_DELAY);
  }

  return (void *)NULL;
}

/**
 * @brief Set LEDs RGV values via SERIAL
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
serialSetLed(const char *path, const char *types, lo_arg **argv, int argc,
        void *data, void *user_data) 
{
  debug(LOG_NOTICE,"got OSC message");
  
	int i;
	int written;

  uint8_t red   = argv[2]->i;
  uint8_t green = argv[3]->i;
  uint8_t blue  = argv[4]->i;

  float frequency = argv[1]->f;

  char *wave = &argv[0]->s;

  uint8_t period_in_tenths = (10/frequency);

	int *fd = (int*)user_data;

	char buf[8];

  debug(LOG_NOTICE,"setting color #%.2x%.2x%.2x over %s with period of %d x 0.1 secs (%.2f Hz)",
        red,green,blue,wave,period_in_tenths, frequency);

  debug(LOG_DEBUG, "sending waveform");

  /* sending waveform */
  if (!strcmp("sine", wave))
		buf[2] = PULSEMODE_SIN;
  else if (!strcmp("triangle",  wave))
		buf[2] = PULSEMODE_TRIANGLE;

	debug(LOG_DEBUG, "sending period values");
	buf[3] = period_in_tenths;

	debug(LOG_DEBUG, "sending rgb values");
	
	buf[4] = red;
	buf[5] = green;
	buf[6] = blue;

	buf[0] = buf[1] = buf[7] = 0x0d;

	for (i=3; i<6; i++) {
		if (buf[i] == 0x0d) {
			buf[i]++;
		}
	}

	debug(LOG_DEBUG, "protocol string is %02X %02X %02X %02X %02X %02X %02X %02X", 
				(unsigned char) buf[0],
				(unsigned char) buf[1],
				(unsigned char) buf[2],
				(unsigned char) buf[3],
				(unsigned char) buf[4],
				(unsigned char) buf[5],
				(unsigned char) buf[6],
				(unsigned char) buf[7]);

	fcntl(*fd, F_SETFL, ~O_NONBLOCK);

	/* OMG, I hope mom won't see this */
	written = write(*fd,&buf,sizeof(buf));
	usleep(200*1000);
	written = write(*fd,&buf,sizeof(buf));
	usleep(200*1000);
	written = write(*fd,&buf,sizeof(buf));
	usleep(200*1000);
	written = write(*fd,&buf,sizeof(buf));
	usleep(200*1000);
	written = write(*fd,&buf,sizeof(buf));
	usleep(200*1000);
	written = write(*fd,&buf,sizeof(buf));

	while (written != sizeof(buf)) {
		if (written == -1) {
			debug(LOG_CRIT, "Unable to write to fd");
			exit(2);
		}
		debug(LOG_ERR, "Wrong number of bytes written retrying...");
		usleep(200*1000);
		written = write(*fd,&buf,sizeof(buf));
	}

	fcntl(*fd, F_SETFL, O_NONBLOCK);


  return TRUE;
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
usbSetLed(const char *path, const char *types, lo_arg **argv, int argc,
        void *data, void *user_data) 
{
  debug(LOG_NOTICE,"got OSC message");

  usb_message_t *message;
  
  uint8_t red   = argv[2]->i;
  uint8_t green = argv[3]->i;
  uint8_t blue  = argv[4]->i;

  float frequency = argv[1]->f;

  char *wave = &argv[0]->s;

  uint8_t period_in_tenths = (10/frequency);

  usb_dev_handle* handle = (usb_dev_handle*)user_data;

  debug(LOG_NOTICE,"setting color #%.2x%.2x%.2x over %s with period of %d x 0.1 secs (%.2f Hz)",
        red,green,blue,wave,period_in_tenths, frequency);

  debug(LOG_DEBUG, "sending waveform");

  /* sending waveform */
  message = (usb_message_t*)malloc(sizeof(usb_message_t));
  message->cmd = USB_SET_WAVEFORM;

  if (!strcmp("sine", wave))
    message->data[0] = PULSEMODE_SIN;
  else if (!strcmp("triangle",  wave))
    message->data[0] = PULSEMODE_TRIANGLE;
  debug(LOG_DEBUG, "pushing red waveform to queue");
  usbTalker(handle,message);
 /*
    usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                    USB_SET_WAVEFORM, PULSEMODE_SIN, 0, (char *)buffer, sizeof(buffer), 5000);
  } else if (!strcmp("triangle",  wave)) {
    usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                    USB_SET_WAVEFORM, PULSEMODE_TRIANGLE, 0, (char *)buffer, sizeof(buffer), 5000);
  } 
  */

  /* sending red */
  message = (usb_message_t*)malloc(sizeof(usb_message_t));
  message->cmd = USB_SET_RED;
  message->data[0] = red;
  debug(LOG_DEBUG, "pushing RED value to queue");
  usbTalker(handle,message);

  /*
  usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                  USB_SET_RED, argv[2]->i, 0, (char *)buffer, sizeof(buffer), 5000);
  */

  /* sending green */
  message = (usb_message_t*)malloc(sizeof(usb_message_t));
  message->cmd = USB_SET_GREEN;
  message->data[0] = green;
  debug(LOG_DEBUG, "pushing GREEN value to queue");
  usbTalker(handle,message);



  /*
  usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                  USB_SET_GREEN, argv[3]->i, 0, (char *)buffer, sizeof(buffer), 5000);
  */

  /* sending blue */
  message = (usb_message_t*)malloc(sizeof(usb_message_t));
  message->cmd = USB_SET_BLUE;
  message->data[0] = blue;
  debug(LOG_DEBUG, "pushing BLUE value to queue");
  usbTalker(handle,message);



  /*
  usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                  USB_SET_BLUE, argv[4]->i, 0, (char *)buffer, sizeof(buffer), 5000);

  */
 
  /* sending blue */
  message = (usb_message_t*)malloc(sizeof(usb_message_t));
  message->cmd = USB_SET_PERIOD;
  message->data[0] = period_in_tenths;
  debug(LOG_DEBUG, "pushing BLUE value to queue");
  usbTalker(handle,message);


  /*  usb_control_msg((usb_dev_handle*)user_data, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
                  USB_SET_PERIOD, (int)(argv[1]->f), 0, (char *)buffer, sizeof(buffer), 5000);
  */
  return TRUE;
}

/** @brief Returns the current button state
 *
 * @param handle USB handle
 * @returns the button state (0|1) and -1 if error
 **/
int usbGetButton(usb_dev_handle* handle) 
{
  char buffer[8];

  int nBytes = usb_control_msg(handle, 
                               USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USB_GET_BUTTON, 
                               0, 
                               0, 
                               (char *)buffer, 
                               sizeof(buffer),
                               5000);

  if(nBytes < 0) {
    debug(LOG_WARNING, "error in USB control transfer: %s\n", usb_strerror());
    usbError(handle, "get_button");
  }
  if(nBytes == 0) // no more data 
    return -127;
  if(nBytes != 1){
    debug(LOG_WARNING, "data format error in get_button, only %d bytes received (1 expected)\n", nBytes);
    return -127;
  } else
    return buffer[0];
}

int usbGetTemperature(usb_dev_handle* handle)
{
  unsigned char buffer[8];

  int nBytes = usb_control_msg(handle, 
                               USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USB_GET_TEMP, 
                              0, 
                              0, 
                              (char *)buffer, 
                              sizeof(buffer), 
                              5000);

  if(nBytes < 0) {
    debug(LOG_WARNING, "error in USB control transfer: %s\n", usb_strerror());
    usbError(handle, "get_temperature");
  }
  if(nBytes == 0) // no more data 
    return -127;
  if(nBytes != 2){
    debug(LOG_WARNING, "data format error in get_temperature, only %d bytes received (2 expected)\n", nBytes);
    return -127;
  } else
    printf("got temp bytes : 0x%.2x%.2x\n",buffer[1],buffer[0]);
    return (buffer[0] << 8) | buffer[1]; 
}


/*
 * OSC functions
 */

int udpCreateServer(int port)
{
  int socket_id;
  struct sockaddr_in sockname;
  int optval;

  debug(LOG_INFO, "create_udp_server on port %d", port);

  if(-1 == (socket_id = socket(PF_INET, SOCK_DGRAM, 0)))
  {
    debug(LOG_ERR, "error, can't create the OSC socket");
    exit(EXIT_FAILURE);
  }

  optval = 1;
  setsockopt(socket_id, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int));

  memset((char*)&sockname, 0, sizeof(struct sockaddr_in));
  sockname.sin_family = AF_INET;
  sockname.sin_port = htons(port);
  sockname.sin_addr.s_addr = htonl(INADDR_ANY);

  if(bind(socket_id, (struct sockaddr *)&sockname, sizeof(struct sockaddr_in)) < 0)
  {
    debug(LOG_ERR, "error, can't bind OSC socket");
    exit(EXIT_FAILURE);
  }
  return(socket_id);
}

void oscReadMessage(int socket_id, usb_dev_handle* handle)
{
  socklen_t sockaddr_in_len = sizeof(struct sockaddr_in);
  struct sockaddr_in sockname;
  unsigned char buffer[2048];
  int r;
  int bufpos = 0;

  char* path;
  char* types;
  char* form;
  float freq;
  int   red, green, blue;

  /* check if data is available with a select */
  fd_set rfds;
  struct timeval tv;
  int retval;
  
  /* watch read on socket_id */
  FD_ZERO(&rfds);
  FD_SET(socket_id, &rfds);
  
  /* wait until 20 ms */
  tv.tv_sec = 0;
  tv.tv_usec = 20000;
  
  retval = select(socket_id+1, &rfds, NULL, NULL, &tv);
  /* return if timed out */ 
  if(retval == 0)
    return;

  /* Read OSC messages */
  if(-1 == (r = recvfrom(socket_id, buffer, sizeof(buffer), 0, (struct sockaddr *)&sockname, &sockaddr_in_len)))
  {
    debug(LOG_ERR, "error in OSC recvfrom");
    shutdown(socket_id, 2);
    close(socket_id);
    exit(EXIT_FAILURE);
  }
  
  path = (char*)&buffer[bufpos];
  printf("OSC message received, address '%s'\n", path);
  bufpos += strlen(path)+1;
  bufpos += (bufpos & 3)?(4 - (bufpos & 3)):0;
  types = (char*)&buffer[bufpos];
  printf("types '%s'\n", types);
  bufpos += strlen(types)+1;
  bufpos += (bufpos & 3)?(4 - (bufpos & 3)):0;
  
  form = (char*)&buffer[bufpos];
  printf("form %s\n", form);
  bufpos += strlen(types)+1;
  bufpos += (bufpos & 3)?(4 - (bufpos & 3)):0;

  unsigned int tmp = buffer[bufpos+3] | (buffer[bufpos+2] << 8) | (buffer[bufpos+1] << 16) | (buffer[bufpos] << 24);
  //freq = *((float*)&buffer[bufpos]);
  freq = *((float*)&tmp);
  printf("freq %f\n", freq);
  bufpos += 4;

  red = ntohl(*((int*)&buffer[bufpos]));
  printf("red %d\n", red);
  bufpos += 4;

  green = ntohl(*((int*)&buffer[bufpos]));
  printf("green %d\n", green);
  bufpos += 4;

  blue = ntohl(*((int*)&buffer[bufpos]));
  printf("blue %d\n", blue);
  bufpos += 4;

  //  set_led(red, green, blue, freq, form, handle);
}

/**
 * @brief This never ending loop is called in USB mode
 *
 */
void 
serialLoop(const char *serialPort, int baudRate, int sensorId, const char *localPort, const char *destination, const char *remotePort) {
	int                 fd;

  int                 button_last_state = 0;
  int                 button_current_state = 0;

  char                osc_path[256];
  lo_address          osc_dest;
  lo_server_thread    st;

	char                data[1];

	char                button_req[7]  = {0x0D,0x09,0x09,0x09,0x09,0x09,0x0D};

	fd = serialOpen(serialPort, baudRate);

	/* Blink LED to say hello */


  /* start OSC server */
  debug(LOG_NOTICE, "starting OSC server");
  
  fflush(stderr);
  fflush(stdout);

  st = lo_server_thread_new(localPort, loError);
  sprintf(osc_path,"/erasme/device/led/%d/set", sensorId);
  lo_server_thread_add_method(st, osc_path, "sfiii", serialSetLed, &fd);
  lo_server_thread_start(st);
  
  //  socket_id = create_udp_server(atoi(oOscLocalPort));
  debug(LOG_NOTICE, "OSC server started");

  /* OSC destination */
  osc_dest = lo_address_new(destination, remotePort);

	for (;;) {
		usleep(SERIAL_DELAY);
		
		write(fd, button_req, 7);
		usleep(SERIAL_DELAY);

		if (read(fd, &data[0], 1) == -1) {
			continue;
		}

		if ((data[0] != 0x0) && (data[0] != 0x01)) {
			continue;
		}

		button_current_state = (int)data[0];

		if (button_last_state != button_current_state) {
			button_last_state = button_current_state;
			debug(LOG_NOTICE, "button is now %s", (button_current_state ? "on" : "off"));

			if (button_current_state) {
				debug(LOG_NOTICE, "button press for %d", sensorId);
				sprintf(osc_path,"/erasme/sensor/keyboard/%d/press", sensorId);
				debug(LOG_NOTICE, "sending OSC notification %s", osc_path);
				if (lo_send(osc_dest, osc_path, "i", 0) == -1) {
					debug(LOG_WARNING, "OSC error %d: %s\n", lo_address_errno(osc_dest), lo_address_errstr(osc_dest));
				}
			} else {
				sprintf(osc_path,"/erasme/sensor/keyboard/%d/release", sensorId);
				debug(LOG_NOTICE, "sending OSC notification %s", osc_path);
				if (lo_send(osc_dest, osc_path, "i", 0) == -1) {
					debug(LOG_WARNING, "OSC error %d: %s\n", lo_address_errno(osc_dest), lo_address_errstr(osc_dest));
				}
			}
		}
	}
}

/**
 * @brief This never ending loop is called in USB mode
 *
 */
void 
usbLoop(int vendor, int product, boolean_t runTest, int sensorId, const char *localPort, const char *destination, const char *remotePort) {

  usb_dev_handle    * handle = NULL;
  char                osc_path[256];
  int                 button_last_state = 0;
  int                 button_current_state = 0;

  lo_address          osc_dest;
  lo_server_thread    st;

  debug(LOG_INFO, "opening device %d:%d", vendor, product);
  if (usbOpenDevice(&handle, vendor, NULL, product, NULL)) {
    debug(LOG_ERR, "unable to open any device matching VID/PID %d/%d", vendor, product);
    exit(EXIT_FAILURE);
  }

  /*
  if(usb_set_configuration(handle, 1) < 0){
    debug(LOG_ERR, "error setting USB configuration: %s", usb_strerror());
  }
  
  if(usb_claim_interface(handle, 0) < 0){
    debug(LOG_ERR, "error setting USB interface: %s", usb_strerror());
  }
  */
  if(runTest){
    if (usbRunTest(handle))
      debug(LOG_NOTICE, "echo test successded");
    else
      debug(LOG_NOTICE, "echo test failure");
  }

  /* start OSC server */
  debug(LOG_NOTICE, "starting OSC server");
  
  fflush(stderr);
  fflush(stdout);

  st = lo_server_thread_new(localPort, loError);
  sprintf(osc_path,"/erasme/device/led/%d/set", sensorId);
  lo_server_thread_add_method(st, osc_path, "sfiii", usbSetLed, handle);
  lo_server_thread_start(st);
  
  //  socket_id = create_udp_server(atoi(oOscLocalPort));
  debug(LOG_NOTICE, "OSC server started");

  /* OSC destination */
  osc_dest = lo_address_new(destination, remotePort);

  for (;;) {
    for(;;) {   

      //read_osc_message(socket_id, handle);

      usleep(USB_DELAY);

      button_current_state = usbGetButton(handle);

      if (button_last_state != button_current_state) {
        button_last_state = button_current_state;
        debug(LOG_NOTICE, "button is now %s", (button_current_state ? "on" : "off"));

        if (button_current_state) {
          debug(LOG_NOTICE, "button press for %d", sensorId);
          sprintf(osc_path,"/erasme/sensor/keyboard/%d/press", sensorId);
          debug(LOG_NOTICE, "sending OSC notification %s", osc_path);
          if (lo_send(osc_dest, osc_path, "i", 0) == -1) {
            debug(LOG_WARNING, "OSC error %d: %s\n", lo_address_errno(osc_dest), lo_address_errstr(osc_dest));
          }
        } else {
          sprintf(osc_path,"/erasme/sensor/keyboard/%d/release", sensorId);
          debug(LOG_NOTICE, "sending OSC notification %s", osc_path);
          if (lo_send(osc_dest, osc_path, "i", 0) == -1) {
            debug(LOG_WARNING, "OSC error %d: %s\n", lo_address_errno(osc_dest), lo_address_errstr(osc_dest));
          }
        }
      }
      
      //      debug(LOG_DEBUG,"asking for temperature");
      //      temperature = get_temperature(handle);
      //      sprintf(osc_path,"/erasme/sensor/temperature/%d/%d", oSensorId, temperature);

      //      debug(LOG_NOTICE, "sending OSC notification %s", osc_path);
      
      //      if (lo_send(osc_dest, osc_path, "i", 0) == -1) {
      //        printf("OSC error %d: %s\n", lo_address_errno(osc_dest), lo_address_errstr(osc_dest));
      //      };
    }

    /*
    // wait for interrupt, set timeout to more than a week 
    nBytes = usb_interrupt_read(handle, USB_ENDPOINT_IN | 1 , 
                                (char *)buffer, sizeof(buffer), 700000 * 1000); 
    usleep(USB_DELAY);

    if(nBytes < 0){
      debug(LOG_WARNING, "error in USB interrupt read: %s\n", usb_strerror());
      usb_error(handle,"main loop");
      }*/
    
  }
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
  char                c;

  int                 button_last_state = 0;
  int                 button_current_state = 0;
  int                 temperature = 0;

  char                osc_path[256];

  /* Options */
	int                 oVendor  = USBDEV_DEFAULT_SHARED_VENDOR;
	int                 oProduct = USBDEV_DEFAULT_SHARED_PRODUCT;

  char *              oOscLocalPort = NULL;
  char *              oOscRemotePort = NULL;
  char *              oOscDestination = NULL;

  char *              oSerialPort = NULL;
  int                 oSerialBaudRate = 9600;

  boolean_t           oRunTest = FALSE;
  int                 oSensorId = 0;

	int                 oRunMode = SERIAL_MODE;

  /* OSC */
  
  int socket_id;

  /* thread stuff */
  while ((c = getopt(argc, argv, "htfv:p:s:l:r:d:g:uP:b:")) != -1 ) {
		printf("serialport:%s###\n",oSerialPort);
		debug(LOG_DEBUG, "handling option %c",c);
    switch (c) {
    case 's':
      oSensorId = atoi(optarg);
      debug(LOG_INFO, "options : sensorID is %d", oSensorId);
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
			// ##TODO : use advertized default
      //oOscLocalPort = g_malloc(strlen(optarg));
      //strcpy(oOscLocalPort,optarg);
			oOscLocalPort = optarg;
      break;
    case 'r': /* ## free needed */
			// ##TODO : use advertized default
      //oOscRemotePort = g_malloc(strlen(optarg));
      //strcpy(oOscRemotePort,optarg);
			oOscRemotePort = optarg;
      break;
    case 'd': /* ## free needed */
			// ##TODO : use advertized default
      //oOscDestination = g_malloc(strlen(optarg));
      //strcpy(oOscDestination,optarg);
      oOscDestination = optarg;
      break;
    case 'g':
      oDebug = atoi(optarg);
      break;
    case 'P': /* ## free needed */
			// ##TODO : use advertized default
      //oSerialPort = g_malloc(strlen(optarg)+1);
      //strcpy(oSerialPort,optarg);
      oSerialPort = optarg;
			printf("serialport:%s###\n",oSerialPort);
      break;
    case 'b':
      oSerialBaudRate = atoi(optarg);
      break;
		case 'u':
			oRunMode = USB_MODE;
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

  debug(LOG_DEBUG, "finished handling options");

  if (oDaemonize) {
    daemonize();
  }


  openlog("museobuttond", LOG_PID, LOG_LOCAL5);

  debug(LOG_INFO, "sensorID is %d", oSensorId);
  

	if (oRunMode == USB_MODE) {
		debug(LOG_INFO, "entering USB mode");
		usbLoop(oVendor, oProduct, oRunTest, oSensorId, oOscLocalPort, oOscDestination, oOscRemotePort);
	} else {
		debug(LOG_INFO, "entering SERIAL mode");
		printf("serialport:%s###\n",oSerialPort);
		serialLoop(oSerialPort, oSerialBaudRate, oSensorId, oOscLocalPort, oOscDestination, oOscRemotePort);
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


/* Local Variables: */
/* mode:c           */
/* comment-column:0 */
/* compile-command: "make" */
/* tab-width: 2 */
/* End:             */
