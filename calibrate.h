/*	calibrate.h
 *
 *	
 */

#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "lpc_types.h"
#include "m3pi.h"

/*	Serial.c
 *
 *	Initialises UART3 for MBED to 3pi interface
 */
 
void putc(uint8_t tmp)
{
		UART_Send((LPC_UART_TypeDef *)LPC_UART3, &tmp, 1, BLOCKING);	//Move forward instruction
}

uint8_t getc(void)	// Not tested
{
		uint8_t tmp;
		UART_Receive((LPC_UART_TypeDef *)LPC_UART3, &tmp, 1, BLOCKING);	//Move forward instruction
		return tmp;
}

void m3pi_init()
{	        
		UART_CFG_Type 		UARTConfigStruct;	// UART Configuration structure variable
		UART_FIFO_CFG_Type 	UARTFIFOConfigStruct;	// UART FIFO configuration Struct variable
        PINSEL_CFG_Type PinCfg;
        
		// Conifg P1.9 as ENET_RDX0 and P0.10 as TXD2
		PinCfg.Funcnum = 2;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Portnum = 0;
		PinCfg.Pinnum = 0;
		PINSEL_ConfigPin(&PinCfg);
		PinCfg.Pinnum = 1;
		PINSEL_ConfigPin(&PinCfg);
		
		UART_ConfigStructInit(&UARTConfigStruct);	//Initialize UART Configuration parameter structure to default
		UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);	//Initialize FIFOConfigStruct to default state
    	UARTConfigStruct.Baud_rate = 115200;	// Re-configure baudrate to 115200bps
		UART_Init((LPC_UART_TypeDef *)LPC_UART3, &UARTConfigStruct);	//Setup the basic data structures for the device
		UART_FIFOConfig((LPC_UART_TypeDef *)LPC_UART3, &UARTFIFOConfigStruct);	//initialize FIFO for UART3 peripheral
		UART_TxCmd((LPC_UART_TypeDef *)LPC_UART3, ENABLE);	//Enable UART3 Transmit
		
		// Send starting message "Ramy"
		putc(DO_PRINT);	//_ser.putc(DO_PRINT)
		putc(0x4);	//putc(0x4)
		putc(0x52); // R
		putc(0x61); // a
		putc(0x6d); // m
		putc(0x79); // y
}

/* TIMER0 INITIALISATION
 *
 * Initialise timer0
 */


void timer0_init()
{
		// Declare structs
		TIM_TIMERCFG_Type TIM_ConfigStruct;
		TIM_MATCHCFG_Type TIM_MatchConfigStruct ;

        PINSEL_CFG_Type PinCfg;
		// Conifg P1.28 as MAT0.0
		PinCfg.Funcnum = 3;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Portnum = 1;
		PinCfg.Pinnum = 28;
		PINSEL_ConfigPin(&PinCfg);

		// Initialize timer 0, prescale count time of 100uS
		TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
		TIM_ConfigStruct.PrescaleValue  = 100;

		// use channel 0, MR0
		TIM_MatchConfigStruct.MatchChannel = 0;
		// Enable interrupt when MR0 matches the value in TC register
		TIM_MatchConfigStruct.IntOnMatch   = TRUE;
		//Enable reset on MR0: TIMER will reset if MR0 matches it
		TIM_MatchConfigStruct.ResetOnMatch = TRUE;
		//Stop on MR0 if MR0 matches it
		TIM_MatchConfigStruct.StopOnMatch  = FALSE;
		//Toggle MR0.0 pin if MR0 matches it
		TIM_MatchConfigStruct.ExtMatchOutputType =TIM_EXTMATCH_TOGGLE;
		// Set Match value, count value of 100 (50 * 100uS = 5000us = 50ms --> 0.05 Hz)
		TIM_MatchConfigStruct.MatchValue   = 50;

		// Set configuration for Tim_config and Tim_MatchConfig
		TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
		TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

		/* preemption = 1, sub-priority = 1 */
		NVIC_SetPriority(TIMER0_IRQn, ((0x01<<3)|0x01));
		/* Enable interrupt for timer 0 */
		NVIC_EnableIRQ(TIMER0_IRQn);
		// To start timer 0
		TIM_Cmd(LPC_TIM0,ENABLE);

		// Set the direction of the pins on Port 1 for the mbed onboard LEDs to output.
		GPIO_SetDir(1, 1<<18|1<<20|1<<21|1<<23, 1);
}

/*	adc0.c
 *
 *	Initialises adc channel 0 for sensor to MBED interface
 *
 *  NOTE: Sharp GP2Y0A21YK0F has timing 38.3ms ± 9.6ms
 */

void adc0_init (void)
{

	_DBG_("Starting");

   // Set up the pin configuration struct for using analogue in on mbed pin 16.
   PINSEL_CFG_Type PinCfg;
   PinCfg.Funcnum   = PINSEL_FUNC_1;
   PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
   PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
   PinCfg.Portnum   = PINSEL_PORT_0;
   PinCfg.Pinnum    = PINSEL_PIN_23;
   
   // Configure mbed pin 16 to use the analogue in channel 1 function (AD0.1).
   PINSEL_ConfigPin(&PinCfg);
   
   // Set up the ADC sampling at 200kHz (maximum rate).
   ADC_Init(LPC_ADC, 200000);
   
   ADC_IntConfig(LPC_ADC,ADC_ADINTEN0,ENABLE);
   
   // Enable ADC channel 1.
   ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}

#endif