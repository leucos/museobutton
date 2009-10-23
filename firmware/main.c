/**
 * @file main.c
 * @brief Main microcontroller code
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
 * - Part of this code (c) ATMEL (TWI)
 * - Part of this code (c) Objective Development Gmbh (USB)
 *
 * Microcontroller pin assignements are listed below :
 *
 * <table><tr><th>µMATCH</th><th>µC_PIN</th><th>USAGE</th></tr>
 * <tr><td>8</td><td>PB0</td><td>QT Mode</td></tr>
 * <tr><td>1</td><td>PB1 (OC1A)</td><td>Red anode</td></tr>
 * <tr><td>2</td><td>PB2 (OC1B)</td><td>Green anode</td></tr>
 * <tr><td>3</td><td>PB3 (OC2)</td><td>Blue anode</td></tr>
 * <tr><td></td><td>PB4 (MISO)</td><td>ISCP only</td></tr>
 * <tr><td></td><td>PB5 (SCK)</td><td>ICSP only</td></tr>
 * <tr><td></td><td>PB6 (XTAL1)</td><td>Xtal only</td></tr>
 * <tr><td></td><td>PB7 (XTAL1)</td><td>Xtal only</td></tr>
 * <tr><td></td><td>PC0-2</td><td>NC</td></tr>
 * <tr><td></td><td>PC3</td><td>Activity LED (active low)</td></tr>
 * <tr><td></td><td>PC4 (SDA)</td><td>i2c bus</td></tr>
 * <tr><td></td><td>PC5 (SCL)</td><td>i2c bus</td></tr>
 * <tr><td></td><td>PC6 (RST)</td><td>ICSP only</td></tr>
 * <tr><td></td><td>PD0 (RX)</td><td>UART</td></tr>
 * <tr><td></td><td>PD1 (TX)</td><td>UART</td></tr>
 * <tr><td></td><td>PD2 (INT0)</td><td>USB D+</td></tr>
 * <tr><td></td><td>PD3</td><td>USB D-</td></tr>
 * <tr><td></td><td>PD4-5</td><td>NC</td></tr>
 * <tr><td>5</td><td>PD6</td><td>QT out</td></tr>
 * <tr><td>7</td><td>PD7</td><td>VDD QT</td></tr>
 * <tr><td>4</td><td></td><td>GND</td></tr>        
 * <tr><td>6</td><td></td><td>GND</td></tr>
 * <tr><td>9-10</td><td></td><td>NC</td></tr>       
 * </table>
 *
 * @sa Atmel: http://www.atmel.com
 * @sa AvrUsb: http://www.obdev.at/products/avrusb/index.html
 */


/** 
 * @def F_CPU
 * @brief CPU system clock frequency (12MHz)
 *
 * @def ISR_PER_SEC
 * @brief Number of timer interrupts per second
 *
 * @def ISR_PER_TENTH
 * @brief Number of timer interrupts per tenths of second
 * 
 * @def PRESCALER
 * @brief Prescaler used for timers
 * @warning Changing the value of prescaler here DOESN'T CHANGE 
 * the real prescaler bits !
 *
 * @def TMP100
 * @brief TMP100 i2c address. ADDR0-1 are set low,
 * thus address is 10010000 (0x90), Cf datasheet p8
 */

#define F_CPU 12000000
#define PRESCALER 256
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define ISR_PER_SEC   ((F_CPU/PRESCALER)/256)
#define ISR_PER_TENTH (ISR_PER_SEC/10)

#define TMP100 0x90   /* 10010000 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include <util/delay.h>
#include <avr/eeprom.h>

#include <stdlib.h>

#include "usbdrv.h"
#include "TWI_Master.h"

#define STEPS   256
#define NUM_WAVEFORMS 2

#define RED   OCR1AL
#define GREEN OCR1BL
#define BLUE  OCR2

#ifdef USB
/** 
 * @brief List of available USB commands.
 * 
 * The same enum must be present in museobuttond.c
 * @sa museobuttond.c
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
#endif

/** 
 * @brief List of available waveforms
 * 
 * The same enum *should* be present in museobuttond.c
 * @warning This enum is not stabilized yet
 * @sa museobuttond.c
 */

typedef enum WAVEFORMS {
  PULSEMODE_NONE,
  PULSEMODE_TRIANGLE,
  PULSEMODE_SAWTOOTH_UP,
  PULSEMODE_SAWTOOTH_DOWN,
  PULSEMODE_SIN,
  PULSEMODE_SINE = PULSEMODE_SIN,
  PULSEMODE_SUBPWM
} waveform_t;

/** 
 * @brief Array index shortcuts
 *
 * R, G and G are respectively the red/green/blue indexes.
 * The fourth (D) index is for the debug led.
 * Theses indexes are shortcuts to address count and maxcount for
 * all LEDs, and maxcolor and waveform for R/G/B LED.
 * 
 */

typedef enum COLORS {
  R,
  G,
  B,
  D
} color_t;

//uint8_t pulse_mode[3];
uint8_t button_state;

/**
 * @brief Port address for color setting
 *
 * This array will host the PWM register address for the color
 */
volatile uint8_t *port[3] = { &RED, &GREEN, &BLUE };

/**
 * @brief Maximum red/green/blue achievable value
 *
 * This value will max the possible red/green/blue value. 
 * It is here to compensate for color differences
 */
volatile uint8_t maxcolor[4];

/**
 * @brief Last seen temperature
 *
 * This variable contains the last seen temperature from the sensor
 */
uint8_t serial_buf_index = 0;
char serial_buffer[16];

/**
 * @brief Last seen temperature
 *
 * This variable contains the last seen temperature from the sensor
 */
uint16_t last_temperature;

/**
 * @brief Number of Timer ISR events for the requested 
 * waveform period.
 *
 * This variable holds how many times the timer interrupt
 * routine will be called for the requested period.
 *
 * This value is necessary to interpolate waveform data.
 * The default value provides some blinking feedback at 1Hz.
 *
 */
volatile uint32_t maxcount[4] = { ISR_PER_SEC/2, 
                                  ISR_PER_SEC/2, 
                                  ISR_PER_SEC/2, 
                                  ISR_PER_SEC};

/**
 * @brief Current position in wave period
 *
 * Holds how many times the timer ISR has been called, so 
 * interpolation can be calculated.
 *
 * The range for this variable is [0..maxcount]
 *
 * @sa maxcount
 */
volatile uint32_t count[4];

/**
 * @brief Duty cycles for SUBPWN waveform
 *
 * Holds how many times the timer ISR has been called, so 
 * interpolation can be calculated.
 *
 * The range for this variable is [0..100]
 *
 */
volatile uint8_t duty[4];

/**
 * @brief Waveform in use
 *
 * Hold the waveform currently in use.
 *
 * Default value is PULSEMODE_NONE except for
 * the heartbeat LED (sine).
 *
 * @sa waveform_t WAVEFORMS
*/
volatile waveform_t waveform[3] = { PULSEMODE_SINE,
                                    PULSEMODE_SINE,
                                    PULSEMODE_SINE, };
                                    
/** 
 * @brief Sine waveform interpolation points
 *
 * The code will interpolate between these points to generate
 * waveforms. Those points are stored in EEPROM.
 */
uint8_t sine_values[STEPS] EEMEM = { 0, 0, 0, 0, 1, 1, 1, 2, 
                                     2, 3, 4, 5, 6, 6, 8, 9, 
                                     10, 11, 12, 14, 15, 17, 18, 20, 
                                     22, 23, 25, 27, 29, 31, 33, 35, 
                                     38, 40, 42, 45, 47, 49, 52, 54, 
                                     57, 60, 62, 65, 68, 71, 73, 76, 
                                     79, 82, 85, 88, 91, 94, 97, 100, 
                                     103, 106, 109, 113, 116, 119, 122, 125, 
                                     128, 131, 135, 138, 141, 144, 147, 150, 
                                     153, 156, 159, 162, 165, 168, 171, 174, 
                                     177, 180, 183, 186, 189, 191, 194, 197, 
                                     199, 202, 204, 207, 209, 212, 214, 216, 
                                     218, 221, 223, 225, 227, 229, 231, 232, 
                                     234, 236, 238, 239, 241, 242, 243, 245, 
                                     246, 247, 248, 249, 250, 251, 252, 252, 
                                     253, 253, 254, 254, 255, 255, 255, 255, 
                                     255, 255, 255, 255, 254, 254, 253, 253, 
                                     252, 252, 251, 250, 249, 248, 247, 246, 
                                     245, 243, 242, 241, 239, 238, 236, 234, 
                                     232, 231, 229, 227, 225, 223, 221, 218, 
                                     216, 214, 212, 209, 207, 204, 202, 199, 
                                     197, 194, 191, 189, 186, 183, 180, 177, 
                                     174, 171, 168, 165, 162, 159, 156, 153, 
                                     150, 147, 144, 141, 138, 135, 131, 128, 
                                     125, 122, 119, 116, 113, 109, 106, 103, 
                                     100, 97, 94, 91, 88, 85, 82, 79, 
                                     76, 73, 71, 68, 65, 62, 60, 57, 
                                     54, 52, 49, 47, 45, 42, 40, 38, 
                                     35, 33, 31, 29, 27, 25, 23, 22, 
                                     20, 18, 17, 15, 14, 12, 11, 10, 
                                     9, 8, 6, 6, 5, 4, 3, 2, 
                                     2, 1, 1, 1, 0, 0, 0, 0, };

/**
 * @brief The device's internal ID location in EEPROM space
 */
uint8_t eeid EEMEM = 0x01;

/**
 * @defgroup twi TWI declarations
 * @{
 */
unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg);
uint8_t TWI_transBuff[4], TWI_recBuff[3];
uint8_t TWI_targetSlaveAddress, TWI_operation;

/**
 * @}
 */

/**
 * @defgroup wavefuncs Waveform generation functions
 * @{
 */
uint8_t triangle(uint8_t count, uint8_t maxcount, uint8_t maxcolor);
uint8_t sawtooth_up(uint8_t count, uint8_t maxcount, uint8_t maxcolor);
uint8_t sawtooth_down(uint8_t count, uint8_t maxcount, uint8_t maxcolor);
uint8_t sine(uint32_t count, uint32_t maxcount, uint8_t maxcolor);
uint8_t subpwm(uint8_t count, uint8_t maxcount, uint8_t maxcolor, uint8_t dutycyle);

/**
 * @}
 */

/** 
 * @defgroup isr Interrupt routines
 * @{
 */
void TIMER1_OVF_vect(void) __attribute__((interrupt));

/**
 * @brief Timer1 interrupt routine
 *
 * @warning This routine is declared <dfn>__attribute__((interrupt))</dfn> so 
 * it can be interrupted by other interrupts.
 * This is necessary to keep USB from timing out.
 */
void TIMER1_OVF_vect(void) {
  int i;

  /* We loop over all LED slots */
  for (i=0; i<4; i++) {
    /* How many times have we been here ? */
    count[i]++;

    if (count[i] >= maxcount[i]) {
      count[i] = 0;
      if (i == D) {
        PORTC ^= _BV(PC3);
      }
    }

    if (maxcount[i] == 0) {
      *port[i] = maxcolor[i];
      continue;
    }

    /* If i == 4, no need to check waveform, just bail out */
    if (i == D)
      return;
    
    /* Check current waveform and change OCRxn values accordingly */
    switch (waveform[i]) {
    case PULSEMODE_NONE:
      break;
#ifdef USE_TRIANGLE
    case PULSEMODE_TRIANGLE:
      *port[i] = triangle(count[i], maxcount[i], maxcolor[i]);
      break;
#endif
#ifdef USE_SAWTOOTH_UP
    case PULSEMODE_SAWTOOTH_UP:
      *port[i] = sawtooth_up(count[i], maxcount[i], maxcolor[i]);
      break;
#endif
#ifdef USE_SAWTOOTH_DOWN
    case PULSEMODE_SAWTOOTH_DOWN:
      *port[i] = sawtooth_down(count[i], maxcount[i], maxcolor[i]);
      break;
#endif
#ifdef USE_SIN
    case PULSEMODE_SIN:
      *port[i] = sine(count[i], maxcount[i], maxcolor[i]);
      break;
#endif
#ifdef USE_SUBPWM
    case PULSEMODE_SUBPWM:
      *port[i] = subpwm(count[i], maxcount[i], maxcolor[i], duty[i]);
      break;
#endif
    }
  }
  return;
}


/**
 * @} 
*/


/**
 * @defgroup initgroup Initialisation functions 
 * @{
 *
 */

/**
 * Initializes i2c bus
 * @sa initgroup
 */

#ifdef I2C
void 
i2c_init() {
	TWI_Master_Initialise();
	TWI_targetSlaveAddress = TMP100;
  TWI_operation = SEND_DATA;
}
#endif

/**
 * @brief Initializes port directions and pin values
 *
 * This function initializes ports and pull-ups according
 * to their assignements. See main.c description for a complete 
 * pin assignement map.
 * @sa main.c
 */
void
pins_init() {
  /**
   * @remarks
   * PORTB :
   * - 7,6 : N/A
   * - 5,4 : N/A
   * - 3-1 : BGR anodes (output)
   * - 0   : mode (output)
   *
   */
  DDRB = _BV(PB3) | _BV(PB2) | _BV(PB1) | _BV(PB0);
  PORTB = 0x08;


  /**
   * @remarks
   * PORTC :
   * - 6   : for ISCP only
   * - 5,4 : i2c
   * - 3   : activity LED (output, active low)
   * - 0-2 : NC
   *
   */
  DDRC = _BV(PC5) | _BV(PC4) | _BV(PC3);
  PORTC = 0x08;

  /**
   * @remarks
   * PORTD :
   * - 7   : QT power (output)
   * - 6   : QT out (input)
   * - 5,4 : NS
   * - 3   : USB D- (output)
   * - 2   : USB D+ (input)
   * - 0,1 : UART
   *
   */

  DDRD = _BV(PD3) | _BV(PD7);
  PORTD = 0x00;

  /*  PORTB &= ~_BV(PB4); 
  PORTB &= ~_BV(PB3); 
  PORTB &= ~_BV(PB2);*/
}

void 
reset_qt() {
  PORTD &= ~_BV(PD7);
  _delay_ms(30);
   PORTD |= _BV(PD7); 
}

#ifdef USB

void
reset_usb() {
  DDRD = ~0;          /* output SE0 for faked USB disconnect */
  _delay_ms(500);
  DDRD = ~(USBMASK | _BV(PD6));    /* all outputs except USB data */
  //pins_init();
}

#else
static inline void usart_init(void)
{
  /* Enable in/out channels */
  UCSRB |= (1 << RXEN) | (1 << TXEN);

  /* Use 8-bit character sizes - URSEL bit set 
   * to select the UCRSC register */
  UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); 

  /* Load lower 8-bits of the baud rate value 
   * into the low byte of the UBRR register */
  UBRRL = BAUD_PRESCALE; 

  /* Load upper 8-bits of the baud rate value 
   * into the high byte of the UBRR register */
  UBRRH = (BAUD_PRESCALE >> 8);
}
#endif

void
pwm_init() {
  /* RED LED : OC1B pin, timer 1, 8bits, fast PWM, 
   * GREEN LED : OC1A pin, timer 1, 8bits, fast PWM */
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10) | _BV(WGM12);
  /**
   * CS12 CS11 CS10 Description
   * 0    0    0    No clock source. (Timer/Counter stopped)
   * 0    0    1    clkI/O/1 (No prescaling)
   * 0    1    0    clkI/O/8 (From prescaler)
   * 0    1    1    clkI/O/64 (From prescaler)
   * 1    0    0    clkI/O/256 (From prescaler)
   * 1    0    1    clkI/O/1024 (From prescaler)
   * 1    1    0    External clock source on T1 pin. Clock on falling edge.
   * 1    1    1    External clock source on T1 pin. Clock on rising edge.
   **/
  TCCR1B = _BV(CS12);
   
  /* BLUE LED : OC2 pin, timer 0, 8bits, fast PWM */
  /**
   *  CS22 CS21 CS20 Description
   *  0    0    0    No clock source (Timer/Counter stopped).
   *  0    0    1    clkT2S/(No prescaling)
   *  0    1    0    clkT2S/8 (From prescaler)
   *  0    1    1    clkT2S/32 (From prescaler)
   *  1    0    0    clkT2S/64 (From prescaler)
   *  1    0    1    clkT2S/128 (From prescaler)
   *  1    1    0    clkT2S/256 (From prescaler)
   *  1    1    1    clkT2S/1024 (From prescaler)
   **/
  TCCR2 = _BV(COM21) | _BV(WGM20) | _BV(WGM21) | _BV(CS22) | _BV(CS21);


  /* Enable Timer 1 interrupts */
  TIMSK |= _BV(TOIE1);
  //  TIMSK |= _BV(TOIE2);

  OCR1A = 255;
  OCR1B = 255;
  OCR2 = 255;
}

/**
 * @}
 * <!-- End of initfunctions -->
 */

#ifdef USB

USB_PUBLIC uchar usbFunctionSetup(uchar data[8])
{
  usbRequest_t    *rq = (void *)data;
  static uchar    replyBuf[8];

  usbMsgPtr = replyBuf;

  if(rq->bRequest == USB_ECHO) {  /* ECHO */
    replyBuf[0] = rq->wValue.bytes[0];
    replyBuf[1] = rq->wValue.bytes[1];
    return 2;
  }
  if(rq->bRequest == USB_GET_ID) {
    replyBuf[0] = eeprom_read_byte(&eeid);
    return 1;
  }
  if(rq->bRequest == USB_SET_ID) {
    eeprom_write_byte(&eeid, rq->wValue.bytes[0]);
    return 1;
  }
  if(rq->bRequest == USB_SET_RED){
    /* set PWM duty cycle on PB4 */
    maxcolor[R] = rq->wValue.bytes[0];
    return 0;
  }
  if(rq->bRequest == USB_SET_GREEN){
    /* set PWM duty cycle on PB3 */
    maxcolor[G] = rq->wValue.bytes[0];
    return 0;
  }
  if(rq->bRequest == USB_SET_BLUE){
    /* set PWM duty cycle on PB2 */
    maxcolor[B] = rq->wValue.bytes[0];
    return 0;
  }
  if(rq->bRequest == USB_SET_PERIOD){ 
    /** max */
    maxcount[R] = maxcount[G] = maxcount[B] = ISR_PER_TENTH*rq->wValue.bytes[0];
    count[R] = count[G] = count[B] = 0;
    return 1;
  }
  if(rq->bRequest == USB_SET_WAVEFORM){
    waveform[R] = waveform[G] = waveform[B] = rq->wValue.bytes[0]; 
    return 1;
  }
  if(rq->bRequest == USB_SET_DUTY_CYCLE){
    duty[R] = duty[G] = duty[B] = rq->wValue.bytes[0]; 
    return 1;
  }
  if(rq->bRequest == USB_SET_COLOR_PERIOD){ 
    /** max */
    maxcount[ISR_PER_TENTH*rq->wValue.bytes[0]] = ISR_PER_TENTH*rq->wValue.bytes[1];
    count[ISR_PER_TENTH*rq->wValue.bytes[0]] = 0;
    return 1;
  }

  if(rq->bRequest == USB_SET_COLOR_WAVEFORM){
    waveform[rq->wValue.bytes[0]] = rq->wValue.bytes[1]; 
    return 1;
  }

  if(rq->bRequest == USB_GET_BUTTON){ 
    replyBuf[0] = button_state;
    return 1;
  }

  if(rq->bRequest == USB_GET_TEMP){ 
    replyBuf[0] = (uint8_t)last_temperature >> 8;
    replyBuf[1] = (uint8_t)last_temperature;
    return 2;
  }
  return 0;
}
#else

static inline void send_char(uint8_t byte)
{
  /* Wait until output buffer is empty */
  while ((UCSRA & (1 << UDRE)) == 0) {};

  /* Send out the byte value in the variable "ByteToSend" */
  UDR = byte;
}

static inline void serial_print(const char *txt) 
{
  while(*txt) {
    send_char(*txt++);
  }   
}

static inline int get_byte()
{
	if (UCSRA & (1<<RXC)) {
		serial_buffer[serial_buf_index] = UDR;
		serial_buf_index++;
		return TRUE;
	}
	return FALSE;
}
// 04 01 ff 00 00
static inline void serial_poll()
{
	get_byte();

	/* Abnormal condition */
	if (serial_buf_index > 6) {
		serial_buf_index = 0;
		return;
	} else if (serial_buffer[5] != 0x0d && serial_buf_index == 6) {
		//serial_print("error : protocol mismatch\n"); 
		serial_buf_index = 0;
		return;
	} else if (serial_buffer[5] == 0x0d && serial_buf_index == 6) {
		//serial_print("ok\n"); 

		if (serial_buffer[0] == 0x09) {
			/*if (button_state)
				serial_print("button pressed\n"); 
			else
				serial_print("button not pressed\n"); 
			*/
			send_char(button_state);
		} else {
			waveform[R] = waveform[G] = waveform[B] = serial_buffer[0];
			maxcount[R] = maxcount[G] = maxcount[B] = ISR_PER_TENTH*serial_buffer[1];
			maxcolor[R] = serial_buffer[2];
			maxcolor[G] = serial_buffer[3];
			maxcolor[B] = serial_buffer[4];
		}

		serial_buf_index = 0;
	} else if (serial_buffer[serial_buf_index-1] == 0x0d && serial_buf_index != 6) {
		serial_buf_index = 0;
	}
}

#endif

#ifdef I2C

/* allow some inter-device compatibility */
#  if !defined TCCR0 && defined TCCR0B
#    define TCCR0   TCCR0B
#  endif
#  if !defined TIFR && defined TIFR0
#    define TIFR    TIFR0
#  endif

/**
 * @brief Read temperature from sensor
 * @return TRUE is a temperature acquisition has been made,
 * FALSE otherwise.
 * @todo Check problem w/ oscillo
 
 * This function reads temperature from TMP100 sensor
 * via i2c bus.
 * It has to be called several times to set the temparature value
 * in last_temperature
 */
uint8_t
get_temp(void)
{
  /* Check if the TWI Transceiver has completed an operation */
  if ( !TWI_Transceiver_Busy() ) {
    /* Check if the last operation was successful */
    if ( TWI_statusReg.lastTransOK ) {
      /* Determine what action to take now */
      if (TWI_operation == SEND_DATA) {
        /* Send data to slave */
        TWI_transBuff[0] = (TWI_targetSlaveAddress) | (FALSE<<TWI_READ_BIT);

        /* Write address 0x00 in pointer register (temperature register address) */
        TWI_transBuff[1] = 0x00;

        /* send TWI_transBuff[0] and TWI_transBuff[1] to TMP100 */
        TWI_Start_Transceiver_With_Data(TWI_transBuff,2);

        /* Set next operation */
        TWI_operation = REQUEST_DATA;
      } else if (TWI_operation == REQUEST_DATA) {
        /* Read high temperature byte from slave */
        TWI_recBuff[0] = (TWI_targetSlaveAddress) | (TRUE<<TWI_READ_BIT);

        /* 2 = TWI_recBuff[0] byte + temp high byte + temp low byte */
        TWI_Start_Transceiver_With_Data(TWI_recBuff,3);

        /* Set next operation */
        TWI_operation = READ_DATA_FROM_BUFFER;
      } else if (TWI_operation == READ_DATA_FROM_BUFFER) { 
        /* Get the received data from the transceiver buffer */
        TWI_Get_Data_From_Transceiver(TWI_recBuff);        

        /* Set next operation */
        TWI_operation = SEND_DATA;

        /* Compute temperature */
        last_temperature = (TWI_recBuff[1] << 4) | (TWI_recBuff[2] >> 4);
        return TRUE;
      }
    } else { 
      /* Got an error during the last transmission
       * Use TWI status information to detemine cause of failure and take appropriate actions */
      TWI_Act_On_Failure_In_Last_Transmission(TWI_Get_State_Info());
      return FALSE;
    }
  } /* End of TWI status check */
  return FALSE;
}

#endif
  
int
main(void)
{
  pins_init();
  pwm_init();

  //wdt_enable(WDTO_1S);

  /* We fake an USB disconnect by pulling D+ and D- to 0 during reset. This is
   * necessary if we had a watchdog reset or brownout reset to notify the host
   * that it should re-enumerate the device. Otherwise the host's and device's
   * concept of the device-ID would be out of sync.
   */

#ifdef USB
  reset_usb();
  
  usbInit();
#else
	usart_init();
#endif

#ifdef I2C
  i2c_init();
#endif

  sei();

  //  TWI_Start_Transceiver();
  reset_qt();

  waveform[R] = waveform[G] = waveform[B] = PULSEMODE_SIN;
  duty[R] = duty[G] = duty[B] = 5;
  maxcolor[R] = maxcolor[G] = maxcolor[B] = 255;

	serial_print("hello\n"); 

  while(1) {
#ifdef USB
    usbPoll();
#else
		serial_poll();
#endif

#ifdef I2C
    get_temp();
#endif

    /*    if ((PIND & _BV(PD6)) && (button_state == 0)) {
      maxcolor[R] = rand(); //(uint8_t) (0x000000ff && rand());
      maxcolor[G] = rand();
      maxcolor[B] = rand();
    }
    */
    if (PIND & _BV(PD6)) {
      button_state = 1;
    } else {
      button_state = 0;
    }
  }

  return 1;
}

#ifdef I2C
unsigned char 
TWI_Act_On_Failure_In_Last_Transmission (unsigned char TWIerrorMsg)
{
  if ((TWIerrorMsg == TWI_MTX_ADR_NACK) | (TWIerrorMsg == TWI_MRX_ADR_NACK))
    TWI_Start_Transceiver();
  
  return TWIerrorMsg; 
}
#endif

#ifdef USE_TRIANGLE 
uint8_t 
triangle(uint8_t count, uint8_t maxcount, uint8_t maxcolor)
{

  //return (uint8_t) (maxcolor * ((float) count) / (float)maxcount);

  if ((count*2)<=maxcount)
    return (uint8_t) (2.0 * maxcolor * ((float)count / (float)maxcount));
  else
    //    return maxcolor - (uint8_t)(((2 * count - maxcount) / maxcount) * 2 * maxcolor); 
    //  return (uint8_t) (maxcolor - (2 * maxcolor * ((float)count / (float)maxcount)));

    //return (uint8_t) (maxcolor - (maxcolor * ((float)count / (float)maxcount) /2));
    return (uint8_t)((float)maxcolor *  (1.0 - (float)(count - (maxcount >> 1)) / ((float)(maxcount >> 1)))); 
}
#endif

#ifdef USE_SAWTOOTH_UP
uint8_t
sawtooth_up(uint8_t count, uint8_t maxcount, uint8_t maxcolor)
{
  return (uint8_t)( 255 * ((float)count/(float)maxcount));
}
#endif

#ifdef USE_SAWTOOTH_DOWN
uint8_t
sawtooth_down(uint8_t count, uint8_t maxcount, uint8_t maxcolor)
{
  return 255-sawtooth_up(count, maxcount, maxcolor);
}
#endif

#ifdef USE_SIN
uint8_t 
sine(uint32_t count, uint32_t maxcount, uint8_t maxcolor)
{
  uint8_t offset = (uint8_t)( 255.0 * ((float)count/(float)maxcount));
  uint8_t eepromsine = eeprom_read_byte((uint8_t*)(sine_values+offset));

  //  return eepromsine;
  //  return rand();
  return (uint8_t) ((float)eepromsine * ((float)maxcolor) / (float)255);
}
#endif

#ifdef USE_SUBPWM
uint8_t
subpwm(uint8_t count, uint8_t maxcount, uint8_t maxcolor, uint8_t dutycycle)
{
  return ( 100*((float)count/(float)maxcount) > dutycycle ? maxcolor : 0 );
}
#endif
