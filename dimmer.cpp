#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdio.h>
#include "Mirf.h"
#include "Mirf_nRF24L01.h"

//DEVICE definition
#define DEV_ADDR 5 //1 is master, so it is not possible

#define PWM_LVL_1 OCR1A
#define	PWM_LVL_2 OCR1B
#define	PWM_LVL_3 OCR2A

#define TIMER_3_SEC_PERIOD 300
#define TIMER_60_SEC_PERIOD 6000

#define SENSOR_TYPES 7 //PWM

mirfPacket volatile inPacket;
mirfPacket volatile outPacket;

uint16_t volatile longTimer;
uint8_t volatile timerInterruptTriggered;

typedef union {
  uint16_t uint;
  struct {
    uint8_t lsb;
    uint8_t msb;
  };
} IntUnion;

void main() __attribute__ ((OS_main,noreturn));

ISR(TIMER0_COMPA_vect) {
  	timerInterruptTriggered++;
  	longTimer++;
}

ISR(BADISR_vect) { //just for case
  __asm__("nop\n\t");
}


//======================================================
void setup()
{

  //start Radio
  Mirf.init();
  Mirf.config();
  Mirf.setDevAddr(DEV_ADDR);
  Mirf.powerUpRx();

  //timer0 10ms period, interrupt enable
  //prescaler 1024, count to 156
  OCR0A = 156;
  OCR0B = 170;
  TCCR0A = 2;
  TCCR0B = 5;
  TIMSK0 = 2;

  //disable unused peripherials
  ACSR |= _BV(ACD); //disable comparator
  PRR = ( _BV(PRTWI) | _BV(PRUSART0) ) ;

  //PWM pins to output
  DDRB = 0b00001110; (OC1A, OC1B, OC2A)

  //start timer 1 (for PWM1 and PWM2
  TCCR1A = 0x23; //fast PWM
  TCCR1B = 0x01;
 
  //start timer 2 (for PWM3)
  TCCR2A = 0x23; //fast	PWM
  TCCR2B = 0x01;
}

//======================================================
void main(void)
{
 wdt_disable();

 setup();



 // endless program loop
 for(;;) {

  if (timerInterruptTriggered > 0)
  {
    timerInterruptTriggered = 0;
    Mirf.handleRxLoop();
    Mirf.handleTxLoop();
  }

  //zpracovat prichozi packet
  if (Mirf.inPacketReady)
  {
    Mirf.readPacket((mirfPacket*)&inPacket);
    if ( (PACKET_TYPE)inPacket.type == REQUEST )
    {

	payloadRequestStruct *req = (payloadRequestStruct*)&inPacket.payload;
	outPacket.type = RESPONSE;
	outPacket.rxAddr = inPacket.txAddr;
	payloadResponseStruct *res = (payloadResponseStruct*)&outPacket.payload;
	res->cmd = req->cmd;
	res->from_sensor = req->for_sensor;
	res->len = 1;

        if (req->for_sensor == 0)
        {
          if (req->cmd == READ)
        }
    }


    if (Mirf.sendingStatus == IN_FIFO)
    {
      Mirf.handleTxLoop();
    }
  }

 }
}
