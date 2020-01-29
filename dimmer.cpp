#define __AVR_ATmega328P__ 1

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
#define SENSOR_TYPES (SENSOR_TYPE)PWM //PWM

#define PWM_LVL_1 OCR1A
#define	PWM_LVL_2 OCR1B
#define	PWM_LVL_3 OCR2B

#define TIMER_3_SEC_PERIOD 300
#define TIMER_60_SEC_PERIOD 6000

//PORTC pins PC0, PC1, PC2 (A0, A1, A2 on arduino)
#define BUTTON_PORT PORTC
#define BUTTON_DDR DDRC
#define BUTTON_INPUT_GATE PINC
#define BUTTON_1 PC0
#define BUTTON_2 PC1
#define BUTTON_3 PC2
#define BUTTON_MASK ((1 << BUTTON_1) | (1 << BUTTON_2) | (1 << BUTTON_3))
#define BUTTON_MASK_1 (1 << BUTTON_1)
#define BUTTON_MASK_2 (1 << BUTTON_2)
#define BUTTON_MASK_3 (1 << BUTTON_3)
#define BUTTON_STATES ((~BUTTON_INPUT_GATE) & BUTTON_MASK)
#define MAX_LONG_PRESS 63

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

uint8_t volatile buttonLongPress[3];
uint8_t volatile lastButtonStates;
uint8_t volatile pwmOutput[3];

//======================================================

inline void ReadAndProcessButtonStates(void)
{
  	uint8_t actual_states = BUTTON_STATES;

  	// 1st button
	if (actual_states & BUTTON_MASK_1)
	{
		if (lastButtonStates & BUTTON_MASK_1)
		{
			if (buttonLongPress[0] < MAX_LONG_PRESS) buttonLongPress[0]++;
		}
	}
	else
	{
		buttonLongPress[0] = 0;
	}

  	// 2nd button
	if (actual_states & BUTTON_MASK_2)
	{
		if (lastButtonStates & BUTTON_MASK_2)
		{
			if (buttonLongPress[1] < MAX_LONG_PRESS) buttonLongPress[1]++;
		}
	}
	else
	{
		buttonLongPress[1] = 0;
	}

  	// 3rd button
	if (actual_states & BUTTON_MASK_3)
	{
		if (lastButtonStates & BUTTON_MASK_3)
		{
			if (buttonLongPress[2] < MAX_LONG_PRESS) buttonLongPress[2]++;
		}
	}
	else
	{
		buttonLongPress[2] = 0;
	}

  	lastButtonStates = actual_states;
}

ISR(TIMER0_COMPA_vect) {
  	timerInterruptTriggered++;
  	longTimer++;
  	ReadAndProcessButtonStates(); //maybe later could be moved to main() in timerInterruptTriggered block
}

EMPTY_INTERRUPT(BADISR_vect) //just for case

//======================================================
void setup()
{

  //start Radio
  Mirf.init(); //WARNING - to be able to use OC1A & OC1B pwm outputs, I had to move CE and CSN pins of mirf to different position, because they overlayed
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

  //PWM pins to output. OC1A (PB1, D9), OC1B (PB2, D10), OC2B (PD3, D3)
  DDRB |= (1 << PB1);
  DDRB |= (1 << PB2);
  DDRD |= (1 << PD3);

  //Button pins (input) with internal Pull-Ups (WARNING - buttons are active LOW)
  BUTTON_PORT |= BUTTON_MASK;

  //start timer 1 (for PWM1 and PWM2
  TCCR1A = 0x23; //fast PWM
  TCCR1B = 0x01;
 
  //start timer 2 (for PWM3)
  TCCR2A = 0x23; //fast	PWM
  TCCR2B = 0x01;
}

//======================================================
void __attribute__ ((OS_main,noreturn)) main (void)
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

  //handle buttons and pwm
  for (uint8_t i=3; i != 0; --i)
  {

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
		res->len = 3;
		//do not need to check req->for_sensor value, because every time we handle all 3 bytes in one shot

		if (req->cmd == READ)
		{
			res->payload[0] = PWM_LVL_1;
			res->payload[1] = PWM_LVL_2;
			res->payload[2] = PWM_LVL_3;
			Mirf.sendPacket((mirfPacket*)&outPacket);
		}
		else if (req->cmd == WRITE)
		{
			PWM_LVL_1 = req->payload[0];
			PWM_LVL_2 = req->payload[1];
			PWM_LVL_3 = req->payload[2];
		}
    }
    else if ( (PACKET_TYPE)inPacket.type == PRESENTATION_REQUEST )
    {
		outPacket.type = PRESENTATION_RESPONSE;
		payloadPresentationStruct *res = (payloadPresentationStruct*)&outPacket.payload;
		res->num_sensors = 3;
  		res->sensor_type[0] = SENSOR_TYPES;
  		res->sensor_type[1] = SENSOR_TYPES;
  		res->sensor_type[2] = SENSOR_TYPES;
  		Mirf.sendPacket((mirfPacket*)&outPacket);
    }

    if (Mirf.sendingStatus == IN_FIFO)
    {
      Mirf.handleTxLoop();
    }
  } //end packet handling

 } //end for loop
}
