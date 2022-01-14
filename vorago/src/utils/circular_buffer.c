/***************************************************************************************
 * @file     circular_buffer.c
 * @version  V1.0
 * @date     31. March 2016
 *
 * @note
 * VORAGO Technologies 
 *
 * @note
 * Copyright (c) 2013-2016 VORAGO Technologies. 
 *
 * @par
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND BY 
 * ALL THE TERMS AND CONDITIONS OF THE VORAGO TECHNOLOGIES END USER LICENSE AGREEMENT. 
 * THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VORAGO TECHNOLOGIES 
 * SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************************/
#include "circular_buffer.h"
#include "va416xx_debug.h"

#define NDEBUG

#ifdef NDEBUG
#undef  c_assert
#undef  dbgprintf
#undef  dbgprintln
#define c_assert(e)     ((void)0)
#define dbgprintf(...)  ((void)0)
#define dbgprintln(...) ((void)0)
#endif

void VOR_CircularBuffer_Initialize(VOR_CircularBuffer_t *pBuffer, void *pData, uint16_t item_size, uint16_t max_items) {
	pBuffer->max_items = max_items; 
	pBuffer->valid_items = 0; 
	pBuffer->data = pData; 
	pBuffer->item_size = item_size; 
	pBuffer->read_index = 0; 
	pBuffer->write_index = 0; 
}

bool VOR_CircularBuffer_IsEmpty(VOR_CircularBuffer_t *pBuffer) {
	if( pBuffer->valid_items == 0 ) {
		return true; 
	} else {
		return false; 
	}
}

bool VOR_CircularBuffer_IsFull(VOR_CircularBuffer_t *pBuffer) {
	if( pBuffer->valid_items == pBuffer->max_items ) {
		return true; 
	} else {
		return false; 
	}
}

int32_t VOR_CircularBuffer_Write(VOR_CircularBuffer_t *pBuffer, const void *pItem) {
  uint16_t i;
	if( pBuffer->valid_items == pBuffer->max_items ) {
		dbgprintln("The Buffer is Full!");
		return -1; 
	} else {
		uint8_t *wPtr = (uint8_t*)pBuffer->data + (pBuffer->write_index * pBuffer->item_size); 
    pBuffer->write_index++;
		pBuffer->valid_items++; 
    for(i=0; i<pBuffer->item_size; i++){
      wPtr[i] = ((uint8_t*)pItem)[i];
    }
		if( pBuffer->write_index == pBuffer->max_items ){
			pBuffer->write_index = 0; 
    }
	}
	return 0; 
}

int32_t VOR_CircularBuffer_Read(VOR_CircularBuffer_t *pBuffer, void *pItem) {
  uint16_t i;
	if( pBuffer->valid_items == 0 ) {
		dbgprintln("The Buffer is Empty!");
		return -1; 
	} else {
		uint8_t *rPtr = (uint8_t*)pBuffer->data + (pBuffer->read_index * pBuffer->item_size); 
    pBuffer->read_index++;
		pBuffer->valid_items--; 
    for(i=0; i<pBuffer->item_size; i++){
      ((uint8_t*)pItem)[i] = rPtr[i];
    }
		if( pBuffer->read_index == pBuffer->max_items ){
			pBuffer->read_index = 0; 
    }
	}
	return 0; 
}
