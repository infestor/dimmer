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

#define RELAY_PIN PD5
#define RELAY_DDR DDRD
#define RELAY_PORT PORTD

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
#define MAX_LONG_PRESS 50 //*10ms
#define DOUBLECLICK_TIMEOUT 40 //*10ms

#define DIMING_DIRECTION_DOWN 0
#define DIMING_DIRECTION_UP 1

enum Actions {
	TURN_OFF = 0,
	TURN_ON,
	RAMP_UP,
	RAMP_DOWN
};

//------------------------------------------------------

mirfPacket volatile inPacket;
mirfPacket volatile outPacket;

uint16_t volatile longTimer;
uint8_t volatile timerInterruptTriggered;

uint8_t volatile buttonLongPress[3];
uint8_t volatile lastButtonStates; //slouzi pro vsechny tri tlacitka po bitech
uint8_t volatile pwmOutput[3];
uint8_t volatile doubleClickCountdown[3];
uint8_t volatile rampingDirection; //slouzi pro vsechny tri tlacitka - kazde ma svuj jeden bit, maska je stejna jako pro ostatni operace
//A protoze po startu je tam nula, coz je pro smer dolu, hodi se to univerzalne, protoze i po jakymkoli startu (at uz full On nebo rampou) by dalsi ramping mel smerovat dolu
uint8_t volatile rampingActive; //slouzi pro vsechny tri tlacitka po bitech
uint8_t volatile lastAction[3];

//======================================================

inline void ReadAndProcessButtonStates(void)
{
  	uint8_t actual_states = BUTTON_STATES;
  	uint8_t button_mask = 1; //points bitmask to first button position in one-bit-per-button variables

  	for (uint8_t i = 0; i < 3; i++)
  	{
  		if (doubleClickCountdown[i] != 0) doubleClickCountdown[i]--;

		if (actual_states & button_mask) //je stisknuto
		{
			if (lastButtonStates & button_mask) //taky minule bylo stisknuto
			{
				if (buttonLongPress[i] < MAX_LONG_PRESS) buttonLongPress[i]++;
				if (buttonLongPress[i] == MAX_LONG_PRESS) //je cas zahajit ramping
				{
					//Turn ON ramping
					buttonLongPress[i]++; //abysme se uměle posunuli za max_long_press a naznacili, ze ramping zacal, ale mozna tady na to neni to spravny misto
					//nejak triggerovat ramping
					rampingActive |= button_mask;
					//musime poznat jestli smerem nahoru nebo dolu a tudiz urcit i vychozi pwm hodnotu
					if ((pwmOutput[i] == 0) || (!(rampingDirection & button_mask)))
					{
						//ramping nahoru, protoze predtim bylo vypnuto, nebo se skoncilo smerem dolu
						rampingDirection |= button_mask;
						lastAction[i] = Actions::RAMP_UP;
					}
					else
					{
						//ramping dolu v ostatnich pripadech
						rampingDirection &= ~button_mask;
						lastAction[i] = Actions::RAMP_DOWN;
					}
				}
			}
		}
		else //neni stisknuto
		{
			if (lastButtonStates & button_mask) //ale bylo stisknuto jeste minule, tzn. release of button happened
			{
				//if (buttonLongPress[i] < 2) //nothing to do, just zero the longPress below
				if ((buttonLongPress[i] > 2) && (buttonLongPress[i] < MAX_LONG_PRESS)) //Short click
				{
					if (pwmOutput[i] == 0)
					{
						//turn the light ON (to 100%)
						lastAction[i] = Actions::TURN_ON;
					}
					else
					{
						//Turn the light OFF (to 0%)
						lastAction[i] = Actions::TURN_OFF;

					}

					doubleClickCountdown[i] = DOUBLECLICK_TIMEOUT;
				}
				else if (buttonLongPress[i] >= MAX_LONG_PRESS) //release after long press
				{
					//Turn OFF ramping
					rampingActive &= ~button_mask;
					//Do not care about ramping direction, it is recognized and set at start of ramping ONLY
				}
			}

			buttonLongPress[i] = 0;
		}

		button_mask = (button_mask << 1); //move the single bit (aka mask) to next button position in byte
  	}

  	lastButtonStates = actual_states;
}

ISR(TIMER0_COMPA_vect) {
  	timerInterruptTriggered++;
  	longTimer++;
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

  //Relay pin (output) PD5 (D5 on arduino)
  RELAY_DDR |= (1 << RELAY_PIN);

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

    ReadAndProcessButtonStates();
  }

  //handle buttons and pwm
  for (uint8_t i=0; i < 3; i++)
  {
	if (buttonLongPress[i] == MAX_LONG_PRESS)
	{
		if (pwmOutput[i] == 0) //start ramping up from 0
		{

		}
		else //long pres during ON state
		{

		}
	}
	else if (buttonLongPress[i] && !(lastButtonStates & (1 << i))) //short click
	{

	}
  }
  //buttonLongPress[2] = 0;

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
