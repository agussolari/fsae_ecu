/***************************************************************************//**
  @file     common.h
  @brief    General and common definitions, available for all modules
  @author   Nicolas Magliola
 ******************************************************************************/

#ifndef _COMMON_H_
#define _COMMON_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define MILISECONDS_PER_SECOND      1000L
#define MICROSECONDS_PER_MILISECOND 1000L
#define MICROSECONDS_PER_SECOND     1000000U
#define SECONDS_PER_MINUTE          60
#define MINUTES_PER_HOUR            60

#define BITS_PER_BYTE               8


// En un archivo de cabecera común, por ejemplo, `common.h`


#ifndef POSITIVE
enum {POSITIVE, NEGATIVE};
#endif // POSITIVE

#ifndef FALSE
enum {FALSE, TRUE};
#endif // FALSE

#ifndef LOW
enum {LOW, HIGH};
#endif // LOW


#define error_exception(error_id)
#define ASM_NOP()                   asm("NOP")
#define ASM_5_NOPs()                do {asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");} while (0)

#define BOOLEAN(a)                  ( (a) != 0 )
#define BITSET(var, nbit)           ((var) |= (1<<(nbit)))
#define BITCLR(var, nbit)           ((var) &= ~(1<<(nbit)))
#define BITCHK(var, nbit)           BOOLEAN((var) & (1<<(nbit)))
#define LO_NIBBLE(a)                ((a) & 0x0F)
#define HI_NIBBLE(a)                (((a)>>4) & 0x0F)
#define HI_BYTE(a)                  ((((uint16)(a)) >> 8) & 0x00FF)
#define LO_BYTE(a)                  (((uint16)(a)) & 0x00FF)
#define TOGGLE(v)                   ((v) = !(v))
#define ABS(v)                      (((v)<0)? (-v) : (v))
#define DIVROUND(a,b)               (((a)+((b)>>1))/(b))


#define NUMEL(array)                (sizeof(array)/sizeof((array)[0]))
#define MAXEL(a,b)                  (((a)>(b))?(a):(b))
#define MINEL(a,b)                  (((a)>(b))?(b):(a))
#define INCMOD(v, m)                (((v)+1 >= (m))? (v)=0 : ++(v))
#define DECMOD(v, m)                (((v) == 0)? (v)=(m)-1 : --(v))

#define _REG3_DEF(a,b,c)            (a ## b ## c)
#define REG3_DEF(a,b,c)             _REG3_DEF(a, b, c)

#define _REG2_DEF(a,b)              (a ## b)
#define REG2_DEF(a,b)               _REG2_DEF(a, b)


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/



/*******************************************************************************
 ******************************************************************************/

#endif // _COMMON_H_

