/***************************************************************************//**
  @file     common.h
  @brief    Common definitions for all modules
  @author   Nicolás Magliola
 ******************************************************************************/

#ifndef _COMMON_H_
#define _COMMON_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h> // Include the standard library header


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/



#ifdef DISABLE_PRINTF

#define printf(...) ((void)0)
#endif


#define BITS_IN_BYTE    8

#define BOOLEAN(a)		( (a) != 0 )
#define BITSET(by, bi)	(by) |= (1<<(bi))
#define BITCLR(by, bi)	(by) &= ~(1<<(bi))
#define LOBYTE(w)   	(0x00FFU&(w))
#define HIBYTE(w)   	(((uint16_t)(w))>>BITS_IN_BYTE)


#define NUMEL(array)        (sizeof(array)/sizeof((array)[0]))
#define MAXEL(a,b)        	(((a)>(b))?(a):(b))
#define MINEL(a,b)        	(((a)>(b))?(b):(a))
#define INCMOD(v, m)       (((v)+1 >= (m))? (v)=0 : ++(v)) // [0..m-1]
#define DECMOD(v, m)       (((v) == 0)? (v)=(m)-1 : --(v)) // [0..m-1]
#define INCTRUNC(d, m)	do { if ((d)<(m)) ++(d); } while (0)
#define DECTRUNC(d, m)	do { if ((d)>(m)) --(d); } while (0)


#ifndef FALSE
#define FALSE   0
#define TRUE    1
#endif // FALSE

#define _REG3_DEF(a,b,c)			(a ## b ## c)
#define REG3_DEF(a,b,c)				_REG3_DEF(a, b, c)
#define _REG2_DEF(a,b)				(a ## b)
#define REG2_DEF(a,b)				_REG2_DEF(a, b)


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

enum {IZQUIERDA, DERECHA, LADOS_CANT};


/*******************************************************************************
 ******************************************************************************/

#endif // _COMMON_H_
