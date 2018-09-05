#ifndef _DFT32_H
#define _DFT32_H

#include <config.h>

#ifdef ICACHE_FLASH
#include <c_types.h> //If on ESP8266
#else
#include <stdint.h>
#endif

#ifndef OCTAVES
#define OCTAVES  5
#endif

#ifndef FIXBPERO
#define FIXBPERO 24
#endif


#define FIXBINS  (FIXBPERO*OCTAVES)
#define BINCYCLE (1<<OCTAVES)


#ifndef DFTIIR
#define DFTIIR 6
#endif

#ifndef CCEMBEDDED
void DoDFTProgressive32( float * outbins, float * frequencies, int bins,
const float * databuffer, int place_in_data_buffer, int size_of_data_buffer,
float q, float speedup );
#endif


int SetupDFTProgressive32(); 
void UpdateBins32( const uint16_t * frequencies );

void PushSample32( int16_t dat );

#ifndef CCEMBEDDED

void UpdateBinsForDFT32( const float * frequencies ); 
#endif


void UpdateOutputBins32();


extern uint16_t embeddedbins32[];


#endif

