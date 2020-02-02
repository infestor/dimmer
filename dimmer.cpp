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
#define SENSOR_TYPES (SENSOR_TYPE)PWM_3_CHANNEL //PWM

//OC0A (PD6, D6), OC0B (PD5, D5), OC2B (PD3, D3)
#define PWM_LVL_1 OCR0A //PD6
#define	PWM_LVL_2 OCR0B //PD5
#define	PWM_LVL_3 OCR2B //PD3

#define RELAY_PIN PD4
#define RELAY_DDR DDRD
#define RELAY_PORT PORTD
#define RELAY_TIMEOUT 255 //*10ms means 2.5s

//PORTC pins PC0, PC1, PC2 (A0, A1, A2 on arduino)
#define BUTTON_PORT PORTC
#define BUTTON_DDR DDRC
#define BUTTON_INPUT_GATE PINC
#define BUTTON_1 PC0
#define BUTTON_2 PC1
#define BUTTON_3 PC2
#define BUTTON_MASK ((1 << BUTTON_1) | (1 << BUTTON_2) | (1 << BUTTON_3))
#define BUTTON_STATES ((~BUTTON_INPUT_GATE) & BUTTON_MASK)
#define MAX_LONG_PRESS 50 //*10ms
#define DOUBLECLICK_TIMEOUT 40 //*10ms

#define DIMING_DIRECTION_DOWN 0
#define DIMING_DIRECTION_UP 1

enum Actions {
	TURN_OFF = 0,
	TURN_ON,
	RAMP_UP,
	RAMP_DOWN,
	STABLE_ON,
	STABLE_OFF,
	EXTERNAL_REQUEST
};

//------------------------------------------------------
uint8_t volatile relayTimer;
uint8_t volatile timerInterruptTriggered;

uint8_t volatile buttonLongPress[3];
uint8_t volatile lastButtonStates; //slouzi pro vsechny tri tlacitka po bitech
uint8_t volatile pwmOutput[3];
uint8_t volatile doubleClickCountdown[3];
uint8_t volatile rampingDirection; //slouzi pro vsechny tri tlacitka - kazde ma svuj jeden bit, maska je stejna jako pro ostatni operace
//uint8_t volatile rampingActive; //slouzi pro vsechny tri tlacitka po bitech
uint8_t volatile intendedState[3];
uint8_t volatile externalRequestValues[3];

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
					//rampingActive |= button_mask;
					//musime poznat jestli smerem nahoru nebo dolu a tudiz urcit i vychozi pwm hodnotu
					if ((pwmOutput[i] == 0) || (!(rampingDirection & button_mask)))
					{
						//ramping nahoru, protoze predtim bylo vypnuto, nebo se skoncilo smerem dolu
						rampingDirection |= button_mask;
						intendedState[i] = Actions::RAMP_UP;
					}
					else
					{
						//ramping dolu v ostatnich pripadech
						rampingDirection &= ~button_mask;
						intendedState[i] = Actions::RAMP_DOWN;
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
						intendedState[i] = Actions::TURN_ON;
					}
					else
					{
						//Turn the light OFF (to 0%)
						intendedState[i] = Actions::TURN_OFF;

					}

					doubleClickCountdown[i] = DOUBLECLICK_TIMEOUT;
				}
				else if (buttonLongPress[i] >= MAX_LONG_PRESS) //release after long press
				{
					//Turn OFF ramping
					intendedState[i] = Actions::STABLE_ON;
					//rampingActive seems not to be necessary so commenting out
					//rampingActive &= ~button_mask;
					//Do not care about ramping direction, it is recognized and set at start of ramping ONLY

					//this will prevent rare situation, where there is small probability, that user starts slow ramp up, but releases button so quickly,
					//that we stop the ramp below 13 (5%) of intensity which is the bottom when ramping down.
					//And if the user would then want to ramp down, the behavior would be actually to only set pwm to 13, which would
					//effectively lead to increase of intensity, instead of decrease.
					//So we have to make slow ramp up also 13 as the lowest value
					if (pwmOutput[i] < 13) pwmOutput[i] = 13;
				}
			}

			buttonLongPress[i] = 0;
		}

		button_mask = (button_mask << 1); //move the single bit (aka mask) to next button position in byte
  	} // end FOR

  	lastButtonStates = actual_states;
}

// this function is run every 10ms, so we can count on that in timing considerations
void DoPwmStuff(void)
{
  	uint8_t button_mask = 1; //points bitmask to first button position in one-bit-per-button variables

  	for (uint8_t i = 0; i < 3; i++)
  	{
  		if (intendedState[i] == Actions::TURN_ON)
  		{
  			//turn on means fast ramp to 100% (255 pwm)
  			//one step is 9, so it is 28.3 steps *10ms = 280ms
  			if (pwmOutput[i] < 246) //do we need continue to fast ramp up?
  			{
  				pwmOutput[i] += 9;
  			}
  			else
  			{
  				//finally fully turned ON, we can stop the ramp
  				pwmOutput[i] = 255;
  				intendedState[i] = Actions::STABLE_ON;
  			}
  			rampingDirection |= button_mask; //simulate direction UP
  		}
  		else if (intendedState[i] == Actions::TURN_OFF)
  		{
  			//turn off means fast ramp down to 0% (0 pwm)
  			if (pwmOutput[i] > 9) //do we need continue to fast ramp down?
  			{
  				pwmOutput[i] -= 9;
  			}
  			else
  			{
  				//finally turned OFF, we can stop the ramp
  				pwmOutput[i] = 0;
  				intendedState[i] = Actions::STABLE_OFF;
  			}
  			rampingDirection &= ~button_mask; //simulate direction down
  		}
  		else if (intendedState[i] == Actions::RAMP_UP)
  		{
  			//Slow ramp up, step is 1 so full ramp takes 2.55s
  			if (pwmOutput[i] < 254) //do we need continue to ramp up?
  			{
  				pwmOutput[i]++;
  			}
  			else
  			{
  				//finally turned full ON, we can stop the ramp
  				pwmOutput[i] = 255;
  				intendedState[i] = Actions::STABLE_ON;
  			}
  		}
  		else if (intendedState[i] == Actions::RAMP_DOWN)
  		{
  			//Slow ramp down, step is 1, but the floor is not 0! It is 5% (13 pwm) so full ramp to minimum takes 2.42s
  			if (pwmOutput[i] > 14) //do we need continue to ramp down?
  			{
  				pwmOutput[i]--;
  			}
  			else
  			{
  				//finally down, we can stop the ramp, but remember - the light is still on 5%, so it is not turned off!
  				pwmOutput[i] = 13;
  				intendedState[i] = Actions::STABLE_ON;
  			}
  		}
  		else if (intendedState[i] == Actions::EXTERNAL_REQUEST)
  		{
  			uint8_t difference = abs(pwmOutput[i] - externalRequestValues[i]);

  			if (pwmOutput[i] > externalRequestValues[i]) //do we need move down
  			{
  				if (difference > 9)
  				{
  					pwmOutput[i] -= 9;
  				}
  				else
  				{
  					pwmOutput[i] = externalRequestValues[i];
  					intendedState[i] = Actions::STABLE_ON;
  				}
	  			rampingDirection &= ~button_mask; //simulate direction down
  			}
  			else if (pwmOutput[i] < externalRequestValues[i]) //move up
  			{
  				if (difference > 9)
  				{
  					pwmOutput[i] += 9;
  				}
  				else
  				{
  					pwmOutput[i] = externalRequestValues[i];
  					intendedState[i] = Actions::STABLE_ON;
  				}
	  			rampingDirection |= button_mask; //simulate direction UP
  			}
  			else
  			{
  				intendedState[i] = Actions::STABLE_ON;
  			}
  		}

		button_mask = (button_mask << 1); //move the single bit (aka mask) to next button position in byte
  	} //end FOR

  	//Now we need to move values to real registers. I would do it 'pointer-wise' way (like have const array of &PWM_LVL_x), but it didnt work well. So here it is supersimple.
  	PWM_LVL_1 = pwmOutput[0];
  	PWM_LVL_2 = pwmOutput[1];
  	PWM_LVL_3 = pwmOutput[2];

  	//handle relay state
  	if ((pwmOutput[0] > 0) || (pwmOutput[1] > 0) || (pwmOutput[2] > 0))
  	{
  		RELAY_PORT |= (1 << RELAY_PIN);
  		relayTimer = 0; //every 10ms zero the timer if light is ON
  	}
  	else
  	{
  		if (relayTimer == RELAY_TIMEOUT)
  		{
  			RELAY_PORT &= (~(1 << RELAY_PIN));
  		}
  	}
}

ISR(TIMER1_COMPA_vect) {
  	timerInterruptTriggered++;
  	relayTimer++;
}

EMPTY_INTERRUPT(BADISR_vect) //just for case

//======================================================
void setup()
{

  //start Radio
  Mirf.init();
  Mirf.config();
  Mirf.setDevAddr(DEV_ADDR);
  Mirf.powerUpRx();

  //timer1 10ms period, interrupt enable, CTC mode
  //prescaler 1024, count to 156
  OCR1A = 156;
  OCR1B = 170;
  TCCR1A = 0;
  TCCR1B = 0b00001101;
  TIMSK1 = 2;

  //disable unused peripherials
  ACSR |= _BV(ACD); //disable comparator
  PRR = ( _BV(PRTWI) | _BV(PRUSART0) ) ;

  //PWM pins to output. OC0A (PD6, D6), OC0B (PD5, D5), OC2B (PD3, D3)
  //Relay pin (output) PD4 (D5 on arduino)
  DDRD |= (1 << PD6) | (1 << PD5) | (1 << PD3) | (1 << RELAY_PIN);

  //Button pins (input) with internal Pull-Ups (WARNING - buttons are active LOW)
  BUTTON_PORT |= BUTTON_MASK;

  //start timer 0 (for PWM1 and PWM2
  TCCR0A = 0b10100001; //phase correct PWM, both OCA OCB outputs activated non inverted, no interrupt
   //start timer 2 (for PWM3)
  TCCR2A = 0b10100001; //phase correct PWM, both OCA OCB outputs activated non inverted, no interrupt

  //prescalers for timer 0 and 2
  TCCR0B = 0b00000001; //no prescaler, 31Khz
  TCCR2B = 0b00000001; //no prescaler, 31Khz
}

//======================================================
void __attribute__ ((OS_main,noreturn)) main (void)
{
 wdt_disable();

 setup();
 sei();


 // endless program loop
 for(;;) {

  if (timerInterruptTriggered > 0)
  {
    timerInterruptTriggered = 0;
    Mirf.handleRxLoop();
    Mirf.handleTxLoop();

    ReadAndProcessButtonStates();
    DoPwmStuff();
  }

  //zpracovat prichozi packet
  if (Mirf.inPacketReady)
  {
    mirfPacket inPacket;

    Mirf.readPacket((mirfPacket*)&inPacket);
    if ( (PACKET_TYPE)inPacket.type == REQUEST )
    {
    	payloadRequestStruct *req = (payloadRequestStruct*)&inPacket.payload;
		//do not need to check req->for_sensor value, because every time we handle all 3 bytes in one shot

		if (req->cmd == READ)
		{
	    	mirfPacket outPacket;

	    	outPacket.type = RESPONSE;
			outPacket.rxAddr = inPacket.txAddr;
			payloadResponseStruct *res = (payloadResponseStruct*)&outPacket.payload;
			res->cmd = req->cmd;
			res->from_sensor = req->for_sensor;
			res->len = 3;

			res->payload[0] = PWM_LVL_1;
			res->payload[1] = PWM_LVL_2;
			res->payload[2] = PWM_LVL_3;

			Mirf.sendPacket((mirfPacket*)&outPacket);
		}
		else if (req->cmd == WRITE)
		{
			for (uint8_t i = 0; i < 3; i++)
			{
				externalRequestValues[i] = req->payload[i];

				if (req->payload[i] == 0)
				{
					intendedState[i] = Actions::TURN_OFF;
				}
				else
				{
					intendedState[i] = Actions::EXTERNAL_REQUEST;
				}

			} // end FOR
		}
    }
    else if ( (PACKET_TYPE)inPacket.type == PRESENTATION_REQUEST )
    {
    	mirfPacket outPacket;

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
