#include <hamming.h>
#include "stm32f4xx_hal.h"
#include "math.h"

#include "arm_math.h"
#include "arm_const_structs.h"
#include "main.h"
#include "hamming.h"
#include "windowing_fft.h"
#include "AudioChip.h"

#define SEGMENT_A 				( GPIO_PIN_7 )
#define SEGMENT_B				( GPIO_PIN_8 )
#define SEGMENT_C				( GPIO_PIN_9 )
#define SEGMENT_D				( GPIO_PIN_10 )
#define SEGMENT_E				( GPIO_PIN_11 )
#define SEGMENT_F				( GPIO_PIN_12)
#define SEGMENT_G				( GPIO_PIN_13 )

#define LED_DIGIT_1 			( GPIO_PIN_2 )
#define LED_DIGIT_2				( GPIO_PIN_5 )
#define LED_DIGIT_3				( GPIO_PIN_4 )
#define LED_DIGIT_4				( GPIO_PIN_6 )

#define SWITCH_1				( GPIO_PIN_0 )
#define SWITCH_2				( GPIO_PIN_1 )

int pressed1 = 0;
int p_released1 = 0;
int pressed2 = 0;
int p_released2 = 0;
uint16_t
	LedBuffer[4] = {};


volatile uint16_t
	LedRefreshCount = 0,
	LedDisplayedDigit = 0;

volatile int16_t
	EchoBuffer[16384];

volatile uint16_t
	EchoPointer = 0;

volatile uint8_t
	ClearEchoBuffer = TRUE;

//
// Data structure for timer configuration
//

TIM_HandleTypeDef
	Timer5_16Khz;

//
// Data structure for general purpose IO configuration
//

GPIO_InitTypeDef
	GpioInitStructure;

//
// Data structure for the D/A(DAC) Converter configuration
//

DAC_ChannelConfTypeDef
	DacInitStructure;

DAC_HandleTypeDef
	AudioDac;				// Structure for the audio digital to analog converter subsystem

//
// Data structures for the A/D Converter configuration
//

ADC_HandleTypeDef
	AudioAdc,
	ReferenceAdc;


volatile int
	ButtonCount = 0,
	ButtonState = RELEASED,
	Effect = NO_EFFECT;
	level = 0;


//
// Buffering system variables
//

volatile int
	ADCPTR = 0;

volatile struct tBuffer
	Buffers[NUMBER_OF_BUFFERS];


volatile int
	WindowingState = 0,
	WindowingDone = FALSE;

//
// 4 times the size of the main buffer to compensate for addition of complex numbers and that we are processing
// 2 buffers at a time
//

float
	delayedBuf[SIZE*4],
	procBuf[SIZE*4];

int
	AD_Offset;

/*
 * Name: UpdateLedDisplay
 *
 * Description: Update a specific seven segment digit on the 4 digit display
 *
 * Inputs:
 * 		None
 *
 * Output:
 * 		None
 *
 * Process:
 *
 * 		Display the selected digit and advance to the next digit
 * 		Refresh the display at a rate with no flicker
 *
 */
int Button1Status () {
	p_released1 = 0; // Initialize pressed and released flag

	//detect for initial press
	if (HAL_GPIO_ReadPin( GPIOB, SWITCH_1 ) == GPIO_PIN_SET) {
		pressed1 = 1; // set initial pressed flag
		p_released1 = 0;
	}

	// once pressed, detect for button release
	if(HAL_GPIO_ReadPin( GPIOB, SWITCH_1 ) == GPIO_PIN_RESET && pressed1 == 1) {
		pressed1 = 0;
		p_released1 = 1; // select pressed then released flag
		// delay
		for(int i = 0; i<5000; i++) {

		}
	}

	return p_released1;
}

int Button2Status() {
	p_released2 = 0;
	if (HAL_GPIO_ReadPin( GPIOB, SWITCH_2 ) == GPIO_PIN_SET) {
		pressed2 = 1;
		p_released2 = 0;
	}


	if(HAL_GPIO_ReadPin( GPIOB, SWITCH_2 ) == GPIO_PIN_RESET && pressed2 == 1) {
		pressed2 = 0;
		p_released2 = 1;
		for(int i = 0; i<20000; i++) {

		}
	}

	return p_released2;
}

	void UpdateLedDisplay( void )
	{


		if (LedRefreshCount > 50) {
			LedDisplayedDigit = 1;
		}

		if (LedRefreshCount > 100) {
			LedDisplayedDigit = 0;
			LedRefreshCount = 0;
		}

		if (Effect == NO_EFFECT) {
			// First Digit
			if (LedDisplayedDigit == 0) {
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET ); // Reset all segments
				HAL_GPIO_WritePin( GPIOE, SEGMENT_G  , GPIO_PIN_SET ); // -
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_RESET ); // Reset second digit
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_SET ); // Set First Digit
			// Second Digit
			}	else if (LedDisplayedDigit == 1)	{
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET ); // Reset all segments
						HAL_GPIO_WritePin( GPIOE, SEGMENT_G , GPIO_PIN_SET ); // -
						HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_RESET ); // Reset first digit
						HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_SET ); // Set second digit
			}
		}  else if (Effect == PITCH_SHIFT_UP_8) {

			if (LedDisplayedDigit == 0) {
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F  , GPIO_PIN_SET ); // U
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_SET );

			}	else if (LedDisplayedDigit == 1)	{
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B |  SEGMENT_D | SEGMENT_E | SEGMENT_F  , GPIO_PIN_RESET );
					HAL_GPIO_WritePin( GPIOE,  SEGMENT_B | SEGMENT_C , GPIO_PIN_SET ); // 1
					HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_RESET );
					HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_SET );
			}
		} else if (Effect == PITCH_SHIFT_UP_16) {

			if (LedDisplayedDigit == 0) {
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F  , GPIO_PIN_SET ); // U
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_SET );

			}	else if (LedDisplayedDigit == 1)	{
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE,  SEGMENT_A |SEGMENT_B | SEGMENT_D | SEGMENT_D | SEGMENT_E | SEGMENT_G , GPIO_PIN_SET ); // 2
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_SET );
			}
		}  else if (Effect == PITCH_SHIFT_UP_32) {

			if (LedDisplayedDigit == 0) {
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F  , GPIO_PIN_SET ); // U
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_SET );

			}	else if (LedDisplayedDigit == 1)	{
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G , GPIO_PIN_SET ); // 3
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_SET );
			}
		} else if (Effect == PITCH_SHIFT_DOWN_8) {

			if (LedDisplayedDigit == 0) {
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_G  , GPIO_PIN_SET ); // d
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_SET );

			}	else if (LedDisplayedDigit == 1)	{
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B |  SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G  , GPIO_PIN_RESET );
					HAL_GPIO_WritePin( GPIOE,  SEGMENT_B | SEGMENT_C , GPIO_PIN_SET ); // 1
					HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_RESET );
					HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_SET );
			}
		} else if (Effect == PITCH_SHIFT_DOWN_16) {

			if (LedDisplayedDigit == 0) {
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_G  , GPIO_PIN_SET ); // d
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_SET );

			}	else if (LedDisplayedDigit == 1)	{
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE,  SEGMENT_A |SEGMENT_B | SEGMENT_D | SEGMENT_D | SEGMENT_E | SEGMENT_G , GPIO_PIN_SET ); // 2
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_SET );
			}
		}  else if (Effect == PITCH_SHIFT_DOWN_32) {

			if (LedDisplayedDigit == 0) {
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_G  , GPIO_PIN_SET ); // d
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_SET );

			}	else if (LedDisplayedDigit == 1)	{
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_G , GPIO_PIN_SET ); // 3
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_1 , GPIO_PIN_RESET );
				HAL_GPIO_WritePin( GPIOE, LED_DIGIT_2 , GPIO_PIN_SET );
			}
		}
		LedRefreshCount++;
	}



/*
 * Name: TIM5_IRQHandler
 *
 * Description: Time 5 interrupt service routine call 16,000 times a second.
 * Inputs:
 * 		None
 *
 * Output:
 * 		None
 *
 * Process:
 *
 * 		Send audio signal to D/A converter
 *		Sample audio input
 *		Do echoing effect
 *		Handle windowing state update
 *		Update the LED display
 *		Detect button press and remove bounce
 *		Switch effects mode
 *
 */


void TIM5_IRQHandler(void)
{

	int16_t
		AudioSignal;

	TIMER_DEBUG_SIGNAL_ON;


//
// Check for timer update interrupt
//
	if ( __HAL_TIM_GET_FLAG( &Timer5_16Khz, TIM_IT_UPDATE ) != RESET )
	{


//
// Check for buffer full status
//
		if( 3 == Buffers[ANALOG_OUT_OFFSET].Full )
		{
//
// Output the Audio stream to the D/A converter
//
			DAC -> DHR12R1 = Buffers[ANALOG_OUT_OFFSET].Buf[Buffers[ANALOG_OUT_OFFSET].Head];

//
// Advanced the head pointer and check for end of buffer
//
			Buffers[ANALOG_OUT_OFFSET].Head++;		//increment head

			if( Buffers[ANALOG_OUT_OFFSET].Head >= SIZE)
			{
//
// Set the head pointer to the start of the buffer
// Reset the buffer full status
//
				Buffers[ANALOG_OUT_OFFSET].Head = 0;
				Buffers[ANALOG_OUT_OFFSET].Full = 0;
			}
		}


//
// Get values from adc and fill the buffer. when it is full reset the
// head pointer and set status to full then increment ALL buffers
// the & 0x03 is to loop the buffers back to 0 when they get to 4
// the << 3 is to increase the volume due to only being a 12b adc
//

//
// See if the buffer is not full
//
		if( 0  == Buffers[ADCPTR].Full)
		{

//
// Take a reading of the analog input pin and remove the offset signal
//
			AudioSignal = HAL_ADC_GetValue( &AudioAdc ) - AD_Offset;

//
// If enabled do the echo effect on the raw signal
//
			if ( ECHO == Effect )
			{

			}
			else
			{

//
// No echo effect. just store the data in the buffer
//

				Buffers[ADCPTR].Buf[Buffers[ADCPTR].Head] = AudioSignal;
			}

//
// Update the head pointer
//
			Buffers[ADCPTR].Head++;

//
// See if the buffer is full
//
			if( Buffers[ADCPTR].Head >= SIZE )
			{

//
// If this statement returns true then the FFT portion of the code has failed.
//
				if (( FALSE == WindowingDone ) && ( 0 != WindowingState ))
				{
//
// Fatal error
//
					while ( TRUE );
				}

//
// Advance to the next buffer
//
				Buffers[ADCPTR].Head = 0;		// Reset the head pointer
				Buffers[ADCPTR].Full = 1;		// Buffer Full = 1
				ADCPTR = ( ADCPTR + 1 ) & BUFFERS_MASK;

//
// changes the state for the overlapping windowing system
//
				switch( WindowingState )
				{
					case 0:
					{
						WindowingState = 1;
						WindowingDone = FALSE;
						break;
					}

					case 1:
					{
						WindowingState = 2;
						WindowingDone = FALSE;
						break;
					}

					case 2:
					{
						WindowingState = 3;
						WindowingDone = FALSE;
						break;
					}

					case 3:
					{
						WindowingState = 4;
						WindowingDone = FALSE;
						break;
					}

					case 4:
					{
						WindowingState = 3;
						WindowingDone = FALSE;
						break;
					}

					default:
					{

//
// Invalid state. Should not get here
//
						while ( TRUE );
						break;
					}
				}
			}
		}

//
// Start another conversion
//
		HAL_ADC_Start( &AudioAdc );

//
// Update the multiplexing LED display
//

		UpdateLedDisplay();

//
// Increment Pitch Shift level based on push button input
//
		if ( Button1Status() == 1 && level < 3) // Cannot go higher than 3 (shift up 32)
		{
			level++;
		}
		else if (Button2Status() == 1 && level > -3) // Cannot go lower than -3 (shift down 32)
		{
			level--;
		}



//
// Set pitch shift level
//
		switch(level) {
		case -3:
			Effect = PITCH_SHIFT_DOWN_32;
			break;
		case -2:
			Effect = PITCH_SHIFT_DOWN_16;
			break;
		case -1:
			Effect = PITCH_SHIFT_DOWN_8;
			break;
		case 0:
			Effect = NO_EFFECT;
			break;
		case 1:
			Effect = PITCH_SHIFT_UP_8;
			break;
		case 2:
			Effect = PITCH_SHIFT_UP_16;
			break;
		case 3:
			Effect = PITCH_SHIFT_UP_32;
			break;
		default:
			Effect = NO_EFFECT;
			break;
		}



//
// Clear the timer update interrupt flag
//
		__HAL_TIM_CLEAR_FLAG( &Timer5_16Khz, TIM_IT_UPDATE );

	}
	TIMER_DEBUG_SIGNAL_OFF;
}


void PitchShift( float *Buffer )
{
	int
		PitchShift,
//
// Pitch Shift by 32 bins in the FFT table
// Each bin contains one complex number comprised of one real and one imaginary floating point number
//
	PitchOffset = 32 * 2;


	if (( PITCH_SHIFT_UP_8 == Effect ) || ( PITCH_SHIFT_DOWN_8 == Effect ))
	{
		PitchOffset = 8 * 2;
	}

	if (( PITCH_SHIFT_UP_16 == Effect ) || ( PITCH_SHIFT_DOWN_16 == Effect ))
	{
		PitchOffset = 16 * 2;
	}

//
// Shift frequencies up effect
//

	if (( PITCH_SHIFT_UP_8 == Effect ) || ( PITCH_SHIFT_UP_16 == Effect ) || ( PITCH_SHIFT_UP_32 == Effect ))
	{
//
// Do the lower half of FFT table
//
		PitchShift = 1022;
		while ( PitchShift >= PitchOffset  )
		{
			Buffer[PitchShift] = Buffer[PitchShift-PitchOffset];
			Buffer[PitchShift+1] = Buffer[(PitchShift+1)-PitchOffset];
			PitchShift -= 2;
		}

//
// Clear the duplicated portion of the table
//
		while ( PitchShift >= 0 )
		{
			Buffer[PitchShift] = 0;
			PitchShift--;
		}


//
// Do the upper half of the FFT table
//
		PitchShift = 1024;
		while ( PitchShift < ( 2048 - PitchOffset ))
		{
			Buffer[PitchShift] = Buffer[PitchShift+PitchOffset];
			Buffer[PitchShift-1] = Buffer[(PitchShift+1)+PitchOffset];
			PitchShift += 2;
		}
//
// Clear the duplicated portion of the table
//
		while ( PitchShift < 2048 )
		{
			Buffer[PitchShift] = 0;
			PitchShift ++;
		}
	}


//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

//
// Shift frequencies down effect
//

	if (( PITCH_SHIFT_DOWN_8 == Effect ) || ( PITCH_SHIFT_DOWN_16 == Effect ) || ( PITCH_SHIFT_DOWN_32 == Effect ))
	{
//
// Do the lower half of FFT table
//
		PitchShift = 0;
		while ( PitchShift <  1024 - PitchOffset  )
		{
			Buffer[PitchShift] = Buffer[PitchShift+PitchOffset];
			Buffer[PitchShift+1] = Buffer[(PitchShift+1)+PitchOffset];
			PitchShift += 2;
		}

//
// Clear the duplicated portion of the table
//
		while ( PitchShift < 1024  )
		{
			Buffer[PitchShift] = 0;
			PitchShift++;
		}


//
// Do the upper half of the FFT table
//
		PitchShift = 2047;
		while ( PitchShift > ( 1024 + PitchOffset ))
		{
			Buffer[PitchShift] = Buffer[PitchShift-PitchOffset];
			Buffer[PitchShift+1] = Buffer[(PitchShift+1)-PitchOffset];
			PitchShift -= 2;
		}
//
// Clear the duplicated portion of the table
//
		while ( PitchShift > 1024 )
		{
			Buffer[PitchShift] = 0;
			PitchShift--;
		}
	}
}


int main(void)
{
	unsigned int
		loop;



	HAL_Init();

	for( loop = 0; loop < NUMBER_OF_BUFFERS; loop++)
	{
		Buffers[loop].Head = 0;
		Buffers[loop].Full = 0;
		memset( (_PTR)&Buffers[loop].Buf, 0, sizeof( Buffers[loop].Buf ));
	}

	InitSystemPeripherals();

//
//	Take an Offset reading to remove the DC offset from the analog reading
//  Source PC2 ( ADC_CHANNEL_12 )
//
	AD_Offset = ConvertReference();

    while( TRUE )
    {
    	WindowingFFT();
    }
}




void InitSystemPeripherals( void )
{

	ADC_ChannelConfTypeDef
		sConfig;

//
// Enable device clocks TIMER and GPIO port E
//
	__HAL_RCC_TIM5_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_DAC_CLK_ENABLE();


//
// Enable ADC3 and GPIO port C clocks
//
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_ADC2_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GpioInitStructure.Pin = SEGMENT_A | SEGMENT_B | SEGMENT_C | SEGMENT_D | SEGMENT_E | SEGMENT_F | SEGMENT_G;
	GpioInitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_PULLUP;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOE, &GpioInitStructure );

	GpioInitStructure.Pin = LED_DIGIT_1 | LED_DIGIT_2 | LED_DIGIT_3 | LED_DIGIT_4;
	GpioInitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_PULLUP;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOE, &GpioInitStructure );




//
// Enable GPIO Port E8 as an input ( used for button select options )
//

	GpioInitStructure.Pin = SWITCH_1 | SWITCH_2;
	GpioInitStructure.Mode = GPIO_MODE_INPUT;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_PULLDOWN;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOB, &GpioInitStructure );

//
// Enable GPIO Port E15 as an output ( used for timing with scope )
//

//	GpioInitStructure.Pin = GPIO_PIN_15 | GPIO_PIN_13;
//	GpioInitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
//	GpioInitStructure.Pull = GPIO_PULLUP;
//	GpioInitStructure.Alternate = 0;
//	HAL_GPIO_Init(GPIOD, &GpioInitStructure);

//
// Enable GPIO port A1 as an analog output
//
	GpioInitStructure.Pin = GPIO_PIN_4;
	GpioInitStructure.Mode = GPIO_MODE_ANALOG;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_NOPULL;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &GpioInitStructure );

	EnableAudioCodecPassThru();

//
// Configure DAC channel 1
//
	AudioDac.Instance = DAC;

	HAL_DAC_Init( &AudioDac );

	DacInitStructure.DAC_Trigger = DAC_TRIGGER_NONE;
	DacInitStructure.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	HAL_DAC_ConfigChannel( &AudioDac, &DacInitStructure ,DAC_CHANNEL_1 );


//
// Enable DAC channel 1
//
//
	HAL_DAC_Start( &AudioDac, DAC_CHANNEL_1 );

//
// Configure A/D converter channel 3
//

//
// Enable GPIO port C1, C2 and C5 as an analog input
//
	GpioInitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_5;
	GpioInitStructure.Mode = GPIO_MODE_ANALOG;
	GpioInitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GpioInitStructure.Pull = GPIO_NOPULL;
	GpioInitStructure.Alternate = 0;
	HAL_GPIO_Init(GPIOC, &GpioInitStructure );

//
// Configure audio A/D ( ADC2 ) for the audio stream
//

	AudioAdc.Instance = ADC2;
	AudioAdc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AudioAdc.Init.Resolution = ADC_RESOLUTION_12B;
	AudioAdc.Init.ScanConvMode = DISABLE;
	AudioAdc.Init.ContinuousConvMode = DISABLE;
	AudioAdc.Init.DiscontinuousConvMode = DISABLE;
	AudioAdc.Init.NbrOfDiscConversion = 0;
	AudioAdc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	AudioAdc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AudioAdc.Init.NbrOfConversion = 1;
	AudioAdc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AudioAdc.Init.DMAContinuousRequests = DISABLE;
	AudioAdc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init( &AudioAdc );

//
// Select PORTC pin 2 ( ADC_CHANNEL_12 ) for the audio stream
//
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	sConfig.Offset = 0;

	HAL_ADC_ConfigChannel(&AudioAdc, &sConfig);
	HAL_ADC_Start( &AudioAdc );

//
// Configure level shifting reference A/D (ADC1)
//
	ReferenceAdc.Instance = ADC1;
	ReferenceAdc.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	ReferenceAdc.Init.Resolution = ADC_RESOLUTION_12B;
	ReferenceAdc.Init.ScanConvMode = DISABLE;
	ReferenceAdc.Init.ContinuousConvMode = DISABLE;
	ReferenceAdc.Init.DiscontinuousConvMode = DISABLE;
	ReferenceAdc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	ReferenceAdc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	ReferenceAdc.Init.NbrOfConversion = 1;
	ReferenceAdc.Init.NbrOfDiscConversion = 0;
	ReferenceAdc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	ReferenceAdc.Init.DMAContinuousRequests = DISABLE;
	ReferenceAdc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init( &ReferenceAdc );

	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	sConfig.Offset = 0;

	HAL_ADC_ConfigChannel(&ReferenceAdc, &sConfig);
	HAL_ADC_Start( &ReferenceAdc );

//
// Initialize timer to 16Khz
//
	Timer5_16Khz.Instance = TIM5;
	Timer5_16Khz.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer5_16Khz.Init.Period = 250;
	Timer5_16Khz.Init.Prescaler = 20;
	Timer5_16Khz.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init( &Timer5_16Khz );

//
// Enable the timer interrupt
//
	HAL_NVIC_SetPriority( TIM5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ( TIM5_IRQn );

	__HAL_TIM_ENABLE_IT( &Timer5_16Khz, TIM_IT_UPDATE );

//
// Enable timer 5 update interrupt
//
	__HAL_TIM_ENABLE( &Timer5_16Khz );


}

int ConvertAudio(void)
{
	int
		ADCResult;

//
// Start a conversion
//
	HAL_ADC_Start( &AudioAdc );

//
// Wait for end of conversion
//
    HAL_ADC_PollForConversion( &AudioAdc, HAL_MAX_DELAY );

//
// Get the 12 bit result
//
    ADCResult = HAL_ADC_GetValue( &AudioAdc );

    return(ADCResult);
}


int ConvertReference(void)
{
	int
		ADCResult;

	ADC_ChannelConfTypeDef sConfig;

//
// Select the channel to convert and start the conversion
//
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	sConfig.Offset = 0;

	HAL_ADC_ConfigChannel(&ReferenceAdc, &sConfig);

//
// Start a conversion
//
	HAL_ADC_Start( &ReferenceAdc );

//
// Wait for end of conversion
//
    HAL_ADC_PollForConversion( &ReferenceAdc, HAL_MAX_DELAY );

//
// Get the 12 bit result
//
    ADCResult = HAL_ADC_GetValue( &ReferenceAdc );

    return(ADCResult);
}






