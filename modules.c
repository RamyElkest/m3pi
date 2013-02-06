/*	modules.c
 *
 */
#include "debug_frmwrk.h"
#include "LPC17xx.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_systick.h"
#include "lpc17xx_i2c.h"
#include "m3pi.h"
#include "modules.h"

/*	Serial.c
 *
 *	Initialises UART3 for MBED to 3pi interface
 */
 
void serputc(uint8_t tmp) {
		UART_Send((LPC_UART_TypeDef *)LPC_UART3, &tmp, 1, BLOCKING);	//Move forward instruction
}

uint8_t sergetc(void)	// Not tested
{
		uint8_t tmp;
		UART_Receive((LPC_UART_TypeDef *)LPC_UART3, &tmp, 1, BLOCKING);	//Move forward instruction
		return tmp;
}

void m3pi_init(void)
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
		serputc(DO_PRINT);	//_ser.serputc(DO_PRINT)
		serputc(0x4);	//serputc(0x4)
		serputc(0x52); // R
		serputc(0x61); // a
		serputc(0x6d); // m
		serputc(0x79); // y
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
		// Set Match value, count value of 100 (100 * 100uS = 10000us = 100ms --> 0.05 Hz)
		TIM_MatchConfigStruct.MatchValue   = 100;

		// Set configuration for Tim_config and Tim_MatchConfig
		TIM_Init(LPC_TIM0, TIM_TIMER_MODE,&TIM_ConfigStruct);
		TIM_ConfigMatch(LPC_TIM0,&TIM_MatchConfigStruct);

		/* preemption = 1, sub-priority = 1 */
		NVIC_SetPriority(TIMER0_IRQn, ((0x01<<3)|0x01));
		/* Enable interrupt for timer 0 */
		NVIC_EnableIRQ(TIMER0_IRQn);
		// To start timer 0
		TIM_Cmd(LPC_TIM0,ENABLE);
}

/*	ADC
 *
 *	Initialises adc channel 0 for sensor to MBED interface
 *
 *  NOTE: Sharp GP2Y0A21YK0F has timing 38.3ms ± 9.6ms
 */

void adc_init (uint8_t portNum, uint8_t pinNum, uint8_t funcNum, uint8_t channelNum)
{
	// Set up the pin configuration struct for using analogue in on mbed pin 16.
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum   = funcNum;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode   = PINSEL_PINMODE_PULLUP;
	PinCfg.Portnum   = portNum;
	PinCfg.Pinnum    = pinNum;

	// Configure mbed pin to use the analogue sensor
	PINSEL_ConfigPin(&PinCfg);

	// Set up the ADC sampling at 200kHz (maximum rate).
	ADC_Init(LPC_ADC, 200000);

	// Disable ADC interrupt to use as polling
	ADC_IntConfig(LPC_ADC, channelNum, DISABLE);   
}

/* digital GPIO
 *
 *	Initialises a GPIO for a digital sensor
 *
 *	Uses external interrupt 3, rising edge
 *
 *	@param GPIO pin number
 *	@param GPIO pin port
 */
void digital_init (uint8_t port, uint8_t pinNum)
{
	// Set up GPIO direction for using digital input
	GPIO_SetDir(port, 1<<pinNum, 0);
	
	// Enable GPIO interrupt External 3 on rising edge
	GPIO_IntCmd(port,(1<<pinNum),1);
	NVIC_EnableIRQ(EINT3_IRQn);
}

/* Compass Initialisation
 *
 *	Initialises a I2C for a compass (accelerometer, gyroscope, magnetometer)
 *
 *	Uses external interrupt 3, rising edge
 *
 *	@param GPIO pin number
 *	@param GPIO pin port
 */
void compass_init ()
{
	 /* Configure Pins */
   PINSEL_CFG_Type PinCfg;

   PinCfg.Funcnum     = PINSEL_FUNC_2;
   PinCfg.OpenDrain   = PINSEL_PINMODE_NORMAL;
   PinCfg.Pinmode     = PINSEL_PINMODE_PULLUP;
   PinCfg.Portnum     = 0;

   PinCfg.Pinnum      = 10;
   PINSEL_ConfigPin(&PinCfg);

   PinCfg.Pinnum      = 11;
   PINSEL_ConfigPin(&PinCfg);

   // Configure I2C to run at 100000Hz
   I2C_Init(LPC_I2C2, 100000);

   // Enable the I2C device
   I2C_Cmd(LPC_I2C2, ENABLE);
}

uint8_t compass_trx(uint8_t send, uint8_t receive[])
{
   uint8_t data_in = 0x00;

   I2C_M_SETUP_Type i2c_m_setup;

   i2c_m_setup.sl_addr7bit = 0xD2 >> 1;		// address 105 LSb '1'
   i2c_m_setup.tx_data = &send;
   i2c_m_setup.tx_length = sizeof(send);
   i2c_m_setup.rx_data = receive;
   i2c_m_setup.rx_length = 6;//sizeof(receive);
   i2c_m_setup.retransmissions_max = 3;

   Status i2c_status = I2C_MasterTransferData(LPC_I2C2, &i2c_m_setup, I2C_TRANSFER_POLLING);
   
   return data_in;
}

/* LED GPIO
 *
 *	Initialises a GPIO for a LED
 *
 *	@param GPIO port number
 *	@param GPIO pin(s) number/map
 */
void led_init(uint8_t port, uint32_t pins)
{
	// Set up GPIO direction for output
	GPIO_SetDir(port, pins, 1);
}

/* LED on/off
 *
 *	Switches MBED LED on/off
 *
 *	@param LED pin
 *	@param GPIO pin(s) number/map
 *	@param boolean 1 - ON ; 0 - OFF
 */
void led(uint8_t port, uint32_t pins, uint8_t state)
{
	if(state)
		GPIO_SetValue(port, pins);
	else
		GPIO_ClearValue(port, pins);
}

/* Read Analogue sensor value
 *
 * Start ADC conversion, poll and return result
 *
 * @param ADC channel, 0-7
 * @return unsigned int 32 bit resolution
 */
 uint32_t read_analog(uint8_t channelNum)
 {	
	if(channelNum >= 0 && channelNum <= 7)
	{
		uint8_t i;

		// ADC conversion result
		uint32_t result=0;
		
		// Enable ADC channel
		ADC_ChannelCmd(LPC_ADC, channelNum, ENABLE);
		
		for(i=0;i<128;i++)
		{
			// Start Conversion
			ADC_StartCmd(LPC_ADC, ADC_START_NOW);
		
			// Wait for conversion to complete
			while(!ADC_ChannelGetStatus(LPC_ADC, channelNum, ADC_DATA_DONE));
		
			// Get result and store it before disabling channel
			result += ADC_ChannelGetData(LPC_ADC, channelNum);
		}
		// Conversion complete, disable channel to remove interference
		ADC_ChannelCmd(LPC_ADC, channelNum, DISABLE);
			
		// Return result
		return result/128;
	}
	 return 0;
}

/* Initialise system tick
 *
 * @param time interval in milliseconds
 * @param counter functional state, ENABLE / DISABLE
 * @param interrupt functional state, ENABLE / DISABLE
 */
void systick_init(uint32_t time, FunctionalState interrupt)
{
	// Initialise systick using internal clock, using time interval in milliseconds
	SYSTICK_InternalInit(time);
	//Enable/Disable System Tick interrupt
	SYSTICK_IntCmd(interrupt);
}

/* Delay in ms
*/
void delay_ms(uint32_t end_time)
{	
	//Enable System Tick Counter
	SYSTICK_Cmd(ENABLE);
	
	//while(SYSTICK_GetCurrentValue() > end_time);
	
}
/* Align robot with horizontal line
 *
 * Called only when one of sensors 0/4 is on a horizontal line
 */
void align()
{
	uint8_t speed = 10,		// Speed to move
			i;				// loop counter
	uint16_t sensors[5],	// Array to store sensor values
			 sensor_max = 2000, // Maximum raw value from a sensor
			 position;		// Position of the line relative to the sensors [0-4000]
	
	
	position = sensors_line_position(sensors);	// Read sensors
		
	for(i=0; i<2; i++)
	{
		// Place both sensors on the line.
		if (sensors[0] != sensor_max) {
			right_motor(speed);
			do {
				position = sensors_line_position(sensors);
			} while(sensors[0] != sensor_max);
		}
		else if (sensors[4] != sensor_max) {
			left_motor(speed);
			do {
				position = sensors_line_position(sensors);
			} while(sensors[4] != sensor_max);
		}
		
		speed = -speed;
		stop();
	}
	
	delay_ms(500);	// Delay half a second for any physical activity
	
	// Iteratively place each sensor on and off the line, until both align.
	// Note: an odd number of iterations is prefered.
	for(i=0; i<5; i++)
	{	
		// Sensor if on the line, move it off the line
		if(sensors[0] == sensor_max)
		{
			right_motor(-speed);
			do {
				position = sensors_line_position(sensors);
			} while(sensors[0] == sensor_max);
		} else {	// Sensor if off the line, move it on the line
			right_motor(speed);
			do {
				position = sensors_line_position(sensors);
			} while(sensors[0] != sensor_max);
		}
		right_motor(0);	// Stop moving
		
		// Sensor if on the line, move it off the line
		if (sensors[4] == sensor_max) {
			left_motor(-speed);
			do {
				position = sensors_line_position(sensors);
			} while(sensors[4] == sensor_max);
		} else {	// Sensor if off the line, move it on the line
			left_motor(speed);
			do {
				position = sensors_line_position(sensors);
			} while(sensors[4] != sensor_max);
		}
		left_motor(0);	// Stop moving
	}
	
	// Assert both sensors are on the line.
	if (sensors[0] != sensor_max) {
		right_motor(speed);
		do {
			position = sensors_line_position(sensors);
		} while(sensors[0] != sensor_max);
	} 
	if (sensors[4] != sensor_max) {
		left_motor(speed);
		do {
			position = sensors_line_position(sensors);
		} while(sensors[4] != sensor_max);
	}
	
	stop();
	_DBG("Whaaaaaa3 ");
	_DBD16(position);
	_DBG(" ");
	_DBD16(sensors[0]);
	_DBG(" ");
	_DBD16(sensors[4]);
	_DBG_("");
}

/* Move to next horizontal line PID
 *
 *
 *
 */
void nextLine()
{
	int16_t proportional,		// P
			last_proportional=0,// Previous proporitonal value
			derivative,			// D
			power_difference,
			max = 20;
	int32_t integral=0;			// I
	uint16_t sensors[5], 		// Array to hold sensor values
			 sensor_max = 2000, // Maximum raw value from a sensor
			 position;		// Position of the line relative to the sensors [0-4000]
	uint8_t bypass = 0;			// Flag to bypass current horizontal line

	position = sensors_line_position(sensors);	// Read sensors

	if(sensors[0] == sensor_max || sensors[4] == sensor_max)
		bypass = 1;			// Sensors are already on a line, allow while loop to run
			 
	// Loop until a horizontal line is reached
	while((sensors[0] != sensor_max || sensors[4] != sensor_max) || bypass)
	{
		position = sensors_line_position(sensors);	// Read sensors

		// The "proportional" term should be 0 when we are on the line.
		proportional = (int16_t)position - 2000;

		// Compute the derivative (change) and integral (sum) of the
		// position.
		derivative = proportional - last_proportional;
		integral += proportional;

		// Remember the last position.
		last_proportional = proportional;

		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the right.  If it is a negative number, the robot will
		// turn to the left, and the magnitude of the number determines
		// the sharpness of the turn.
		power_difference = proportional/20 + integral/10000 + derivative*3/2;

		// Compute the actual motor settings.  We never set either motor
		// to a negative value.
		if(power_difference > max)
			power_difference = max;
		if(power_difference < -max)
			power_difference = -max;

		if(power_difference < 0) {
			right_motor(max+power_difference);
			left_motor(max);
		} else {
			right_motor(max);
			left_motor(max-power_difference);
		}
		
		if(bypass && (sensors[0] != sensor_max && sensors[4] != sensor_max))
			bypass = 0;		// Sensor is off the first horizontal line, revoke bypass
	}
	stop();

	align();
}

/* Follow Wall PID
 *
 *
 * @param channelA, ADC channel to read first sensor
 * @param channelB, ADC channel to read second sensor
 */

void wallFollow(uint8_t channelA, uint8_t channelB)
{
	int16_t proportional,		// P
			last_proportional=0,// Previous proporitonal value
			derivative,			// D
			power_difference,
			max = 50,			// Maximum speed
			last_pd=0;			// Previous power_derivative
	int32_t integral=0;			// I
	int64_t temp = 0;			// Temp 64 bit
	int16_t sensors[2], 		// Array to hold sensor values
			 i;
	uint16_t count=0;

	// Loop until stopped by external interrupt
	while(1)
	{		
		sensors[0] = read_analog(channelA);
		sensors[1] = read_analog(channelB);

		temp = ((sensors[0] * sensors[1])/1000) - (int64_t)6250;
		proportional = temp/2;

		//proportional = (sensors[0] * sensors[1]) - ((int16_t)2000 * sensors[0]) - (sensors[1] - (int16_t)2000);

		// Compute the derivative (change) and integral (sum) of the
		// position.
		derivative = proportional - last_proportional;
		integral += proportional/1000;

		// Remember the last position.
		last_proportional = proportional;

		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the right.  If it is a negative number, the robot will
		// turn to the left, and the magnitude of the number determines
		// the sharpness of the turn.
		power_difference = proportional/1000 + integral/1000 + derivative*1/2;


		if(count == 1000)
		{
		_DBG("PID: ");
		_DBD16(sensors[0]);
		_DBG(" ");
		_DBD16(sensors[1]);
		_DBG(" ");
		_DBD16(proportional);
		_DBG(" ");
		_DBD16(derivative);
		_DBG(" ");
		_DBD16(integral);
		_DBG(" ");
		_DBD16(power_difference);
		_DBG_(" ");
		count=0;
		}

		// Compute the actual motor settings.  We never set either motor


		// to a negative value.
		if(power_difference > max)
			power_difference = max;
		if(power_difference < -max)
			power_difference = -max;

		if(power_difference < 0) {
			right_motor(max+power_difference);
			left_motor(max);
		} else {
			right_motor(max);
			left_motor(max-power_difference);
		}

		last_pd = power_difference;
		count++;
	}
	stop();
}

