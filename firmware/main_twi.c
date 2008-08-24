/* Name: main.c
 * Project: museobutton
 * Author: Michel Blanc
 * Creation Date: 2008-02-6
 * Tabsize: 2
 * Copyright: (c) 2008 ERASME
 * License: GNU GPL v2
 * This Revision: $Id: main.c 389 2007-08-07 11:13:12Z cs $
 * Part of this code (c) ATMEL (TWI)
 * Part of this code (c) Objective Development Gmbh (USB)
 */


/* 
RED led : PB4
GREEN led : PB3
BLUE led : PB2
*/

#define F_CPU 12000000
#define USBSUX 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#include "usbdrv.h"
#include "oddebug.h"
#include <util/delay.h>

//#include "TWI_Master.h"

enum {
  USB_ECHO,
  USB_SETRED,
  USB_SETGREEN,
  USB_SETBLUE,
  USB_SETFREQUENCY,
  USB_SETWAVEFORM,
  USB_GETBUTTON,
  USB_GETTEMP,
};

enum {
  PULSEMODE_TRIANGLE,
  PULSEMODE_SIN,
  PULSEMODE_SAWTOOTH
};

//unsigned char TWI_Act_On_Failure_In_Last_Transmission(unsigned char TWIerrorMsg);

//uint8_t TWI_transBuff[4], TWI_recBuff[3];
//uint8_t TWI_targetSlaveAddress, TWI_operation;

uint8_t pulse_mode;

USB_PUBLIC uchar usbFunctionSetup(uchar data[8])
{
  usbRequest_t    *rq = (void *)data;
  static uchar    replyBuf[8];

  usbMsgPtr = replyBuf;

  if(rq->bRequest == USB_ECHO){  /* ECHO */
    replyBuf[0] = rq->wValue.bytes[0];
    replyBuf[1] = rq->wValue.bytes[1];
    return 2;
  }
  if(rq->bRequest == USB_SETRED){
    /* set PWM duty cycle on PB4 */
    OCR1BL = rq->wValue.bytes[0];
    return 0;
  }
  if(rq->bRequest == USB_SETGREEN){
    /* set PWM duty cycle on PB3 */
    OCR1AL = rq->wValue.bytes[0];
    return 0;
  }
  if(rq->bRequest == USB_SETBLUE){
    /* set PWM duty cycle on PB2 */
    OCR0A = rq->wValue.bytes[0];
    return 0;
  }
  if(rq->bRequest == USB_SETFREQUENCY){ 
    OCR0A = rq->wValue.bytes[0];

    return 1;
  }
  if(rq->bRequest == USB_SETWAVEFORM){ 
    return 1;
  }

  if(rq->bRequest == USB_GETBUTTON){ 
    replyBuf[0] = 0x12;
    return 1;
  }

  if(rq->bRequest == USB_GETTEMP){ 
    replyBuf[0] = 0x12;
    replyBuf[1] = 0x34;
    return 2;
  }
  return 0;
}

/* allow some inter-device compatibility */
#if !defined TCCR0 && defined TCCR0B
#define TCCR0   TCCR0B
#endif
#if !defined TIFR && defined TIFR0
#define TIFR    TIFR0
#endif

#ifdef TWISUX

void 
i2cInit() {
	TWI_Master_Initialise();
	TWI_targetSlaveAddress = 0x42;		// MHCMC6352's address
	
  TWI_transBuff[0] = (TWI_targetSlaveAddress) | (TRUE<<TWI_READ_BIT);
	TWI_transBuff[1] = 'A';
}

#endif

void
pins_init() {
  //  DDRD = 0xff; 
  DDRD = ~(1 << 2);   /* all outputs except PD2 = INT0 */
  PORTD = 0x00;

  /* 1111 1110b: all out except PB0 */
  PORTB = 0x02; /* 0000 0011 : sourcing from PB1 */
  DDRB = 0xfe;  /* 1111 1110 : PB0 input */

  /* TODO : init pwm values */
}

void reset_qt() {
  PORTB &= ~_BV(PB1);
  _delay_ms(30);
   PORTB |= _BV(PB1); 
}

int
main(void)
{
  uint8_t i=0;


  pins_init();

  //wdt_enable(WDTO_1S);

  /* We fake an USB disconnect by pulling D+ and D- to 0 during reset. This is
   * necessary if we had a watchdog reset or brownout reset to notify the host
   * that it should re-enumerate the device. Otherwise the host's and device's
   * concept of the device-ID would be out of sync.
   */

#ifdef USBSUX

  DDRD = ~0;          /* output SE0 for faked USB disconnect */

  i = 0;
  while(--i){         /* fake USB disconnect for > 500 ms */
    wdt_reset();
    _delay_ms(2);
  }

  DDRD = ~USBMASK;    /* all outputs except USB data */
  
  usbInit();

  /* i2cInit(); */

  sei();
#endif

  /*  while (1) {
    PORTB &= ~_BV(PB4); 
    PORTB &= ~_BV(PB3); 
    PORTB &= ~_BV(PB2); 

    i = 255; 
    while (i--)
      _delay_ms(4);


    PORTB |= _BV(PB4); 
    PORTB |= _BV(PB3); 
    PORTB |= _BV(PB2); 

    i = 255; 
    while (i--)
      _delay_ms(4);
      }*/

  reset_qt();

  while(1) {
    usbPoll();
    if (PINB & _BV(PB0)) {    
      PORTB |= _BV(PB4); 
    } else {
      PORTB &= ~_BV(PB4);       
    }
  }

  while (1) {

    PORTB &= ~_BV(PB4); 
    PORTB &= ~_BV(PB3); 
    PORTB &= ~_BV(PB2); 

    PORTB |= _BV(PB4); 

    i = 255; 
    while (i--)
      _delay_ms(4);

    PORTB &= ~_BV(PB4); 
    PORTB |= _BV(PB3); 

    i = 255; 
    while (i--)
      _delay_ms(4);

    PORTB &= ~_BV(PB3); 
    PORTB |= _BV(PB2); 

    i = 255; 
    while (i--)
      _delay_ms(4);

    PORTB |= _BV(PB4); 
    PORTB |= _BV(PB3); 

    i = 255; 
    while (i--)
      _delay_ms(4);

    PORTB &= ~_BV(PB4); 
    PORTB &= ~_BV(PB3); 
    PORTB &= ~_BV(PB2); 
  }

  return 1;
}

#ifdef TWISUX

int stub(void) {
  uint16_t loops;
  uint8_t heading[2];
  uint16_t heading16;

	TWI_Start_Transceiver_With_Data(TWI_transBuff,2);
	TWI_operation = REQUEST_DATA; 		// Set the next operation
  	
	for (;;) {
    // Check if the TWI Transceiver has completed an operation.
    if ( ! TWI_Transceiver_Busy() ) {
      // Check if the last operation was successful
      if ( TWI_statusReg.lastTransOK ) {
        // Determine what action to take now
        if (TWI_operation == SEND_DATA) {
          // Send data to slave
          TWI_transBuff[0] = (TWI_targetSlaveAddress) | (FALSE<<TWI_READ_BIT);
          TWI_transBuff[1] = 'A';
          TWI_Start_Transceiver_With_Data(TWI_transBuff,2);	// send TWI_transBuff[0] and TWI_transBuff[1] to HMC6352
          TWI_operation = REQUEST_DATA; 						// Set next operation
        } else if (TWI_operation == REQUEST_DATA) {
          // Request data from slave
          TWI_recBuff[0] = (TWI_targetSlaveAddress) | (TRUE<<TWI_READ_BIT);
          TWI_Start_Transceiver_With_Data(TWI_recBuff,3); // 3 = TWI_recBuff[0] byte + heading high byte + heading low byte
          TWI_operation = READ_DATA_FROM_BUFFER; 			// Set next operation        
        } else if (TWI_operation == READ_DATA_FROM_BUFFER) { 
          // Get the received data from the transceiver buffer
          TWI_Get_Data_From_Transceiver(TWI_recBuff);        
          TWI_operation = SEND_DATA;    					// Set next operation        
          // compute the heading.  If you have JTAG, you may wish to set a breakpoint on this
          // line to see the heading during development or trouble shooting.
          heading[0] = TWI_recBuff[1];
          heading[1] = TWI_recBuff[2];

          heading16 = (heading[0] << 8) | heading[1];

          if ((heading16 > 3550) || (heading16 < 50)) {
            PORTB &= ~_BV(PB0); 
            PORTB &= ~_BV(PB1);             
          } else if (heading16 < 1800) {
            PORTB &= ~_BV(PB0); 
            PORTB |= _BV(PB1); 
          } else {
            PORTB &= ~_BV(PB1); 
            PORTB |= _BV(PB0); 
          }
        }
      } else { // Got an error during the last transmission
        // Use TWI status information to detemine cause of failure and take appropriate actions. 
        TWI_Act_On_Failure_In_Last_Transmission(TWI_Get_State_Info( ));
      }
    } //end of TWI status check
    
    usbPoll();

	    // put your main program code here
		_delay_ms(100); // don't read the HMC6352 too fast

  
	} // end for(;;)

  return 0;

  for(;;){    /* main event loop */
    //wdt_reset();
    usbPoll();

    loops++;

    if (loops > 10000) {
      loops = 0;
      PORTB ^= _BV(PB0);
      PORTB ^= _BV(PB1);
    }
  }
  return 0;
}


unsigned char 
TWI_Act_On_Failure_In_Last_Transmission (unsigned char TWIerrorMsg)
{
	if ((TWIerrorMsg == TWI_MTX_ADR_NACK) | (TWIerrorMsg == TWI_MRX_ADR_NACK))
		TWI_Start_Transceiver();
    
 	return TWIerrorMsg; 
}

#endif
