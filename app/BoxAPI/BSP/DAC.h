// DAC.h

#ifndef DAC_h // include guard
#define DAC_h

#ifdef __cplusplus
extern "C"
{
#endif
#include "stm32f4xx_hal.h"
/*--------------------------------------------------*/
/*------------------Definitions---------------------*/
/*--------------------------------------------------*/

/*
 * Ping PD7 to check timing on the audio processing.
 * PD7 will be set high when the DAC interrupt is entered, and set low when exited.
 * This allows you to check that there is sufficient time for the rest of the system to operate,
 * and to ensure that the sample interrupts are not being missed or are overlapping.
 */
//#define DEBUG_PING

/*
 * Bytes used to control the MCP4822 DAC
 */
#define CH_A_OUT 0x10
#define CH_B_OUT 0x90

#define CH_A_OFF 0x00
#define CH_B_OFF 0x80

	// end an SPI transfer with this; this raises the SS signal
	void spi1_dac_finalize(void);

	// void spi1_dac_write(uint16_t);	// internal function, called by the two following...
	void spi1_dac_write_cha(uint16_t); // write 16 bits to DAC channel A
	void spi1_dac_write_chb(uint16_t); // write 16 bits to DAC channel B

//==================================================
//****************** IIR Filter ******************//
//==================================================
/*
#ifndef SAMPLE_RATE
#define SAMPLE_RATE		    16000 // set up the sampling Timer1 to 48000Hz, 44100Hz (or lower),
					  // runs at audio sampling rate in samples per Second.
					  // 384 = 3 x 2^7 divisors 2k, 3k, 4k, 6k, 8k, 12k, 16k, 24k, 48k
#endif

// IIR filter coefficient scaling
// multiply the coefficients by 32, assume a(1) is 32 * (1 + alpha), to get better accuracy in fixed format.
#define IIRSCALEFACTOR		32
#define IIRSCALEFACTORSHIFT	5

// IIR Resonance (maximum) at the corner frequency.
#define Q_MAXIMUM 			(6.0f)
#define Q_LINEAR			(M_SQRT1_2)

// float-fix conversion macros
// assuming we're using 8.8 for the coefficients.
#define float2int(a) ((int16_t)((a)*256.0))
#define int2float(a) ((double)(a)/256.0)

*/
/*--------------------------------------------------*/
/*--------------------Typedefs----------------------*/
/*--------------------------------------------------*/
/*
typedef union _DAC_value_t{
	uint16_t u16;
	int16_t  i16;
	uint8_t  u8[2];
} DAC_value_t;

typedef struct __attribute__ ((packed)) _DAC_command_t {
	DAC_value_t value;
} DAC_command_t;
*/
/*
 * Prototype for the analogue DSP function to be implemented.
 * Needs to at least provide *ch_A and *ch_B
 * Runs within Timer0/1 interrupt routine - time critical I/O.
 * Keep it very short and punchy!
 * Use the DEBUG pings to ensure you're not using too much time.
 */
/*
typedef void (analogue_DSP)( uint16_t* ch_A, uint16_t* ch_B);

*/
//==================================================
//****************** IIR Filter ******************//
//==================================================
/*
typedef struct __attribute__ ((packed)) _filter_t {
	uint16_t sample_rate;	// sample rate in Hz
	uint16_t cutoff;		// normalised cutoff frequency, 0-65536. maximum is sample_rate/2
	uint16_t peak;			// normalised Q factor, 0-65536. maximum is Q_MAXIMUM
	int16_t b0,b1,b2,a1,a2; // Coefficients in 8.8 format
	int16_t xn_1, xn_2;	//IIR state variables
	int16_t yn_1, yn_2; //IIR state variables
} filter_t;

*/
/*--------------------------------------------------*/
/*----------Public Function Definitions-------------*/
/*--------------------------------------------------*/

//==================================================
//****************** IIR Filter ******************//
//==================================================
// second order IIR -- "Direct Form I Transposed"
//  a(0)*y(n) = b(0)*x(n) + b(1)*x(n-1) +  b(2)*x(n-2)
//                   - a(1)*y(n-1) -  a(2)*y(n-2)
// assumes a(0) = IIRSCALEFACTOR = 32

// Compute filter function coefficients
// http://en.wikipedia.org/wiki/Digital_biquad_filter
// https://www.hackster.io/bruceland/dsp-on-8-bit-microcontroller
// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
/*
void setIIRFilterLPF( filter_t * const filter ); // Low Pass
void setIIRFilterBPF( filter_t * const filter ); // Band Pass
void setIIRFilterHPF( filter_t * const filter ); // High Pass

// returns y(n) filtered by the biquad IIR process in place of x(n)
void IIRFilter( filter_t * const filter, int16_t * const xn ) __attribute__ ((hot, flatten));
*/
#ifdef __cplusplus
}
#endif

#endif // DAC_h end include guard
