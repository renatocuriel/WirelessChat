/*
 * addresses.h
 *
 *  Created on: May 14, 2024
 *      Author: thede
 */

#ifndef INC_ADDRESSES_H_
#define INC_ADDRESSES_H_

#include <stdio.h>
#include <stdint.h>

#define C_BAE       0xB2
#define D_CALDERA   0xA3
#define L_CAPUTI    0x6B
#define W_COLBURN   0x6A
#define D_CURIEL    0x7F
#define N_DELAPENA  0x1B
#define A_DOSANJH   0x77
#define H_EVANS     0x87
#define M_FESLER    0x7B
#define J_GALICINAO 0x1D
#define T_GREEN     0xC9
#define A_GROTE     0x3D
#define M_HERRERA   0x85
#define B_KENNEDY   0x7D
#define J_KRAMMER   0x16
#define R_LEONTINI  0x98
#define J_MANESH    0x0B
#define S_MARTIN    0x30
#define N_MASTEN    0x0F
#define L_MCCARTHY  0x8C
#define P_MULPURU   0xC7
#define M_NOON      0x3E
#define J_PARK      0x26
#define L_PEDROZA   0x80
#define D_PETERS    0x49
#define M_PROVINCE  0x13
#define A_RAJESH    0x79
#define J_RAMIREZ   0xA2
#define D_ROBERDS   0xC1
#define D_ROLAND    0x6F
#define D_SANDALL   0xB0
#define S_SELTZER   0x60
#define J_SHAFFER   0x3F
#define M_WONG      0xA0
#define A_TAYLOR 0x69

#define NUM_ADDRESSES 35

typedef struct {
    uint8_t address;
    const char *name;
} AddressNamePair;

// Lookup table
 extern AddressNamePair lookupTable[];

void getNameFromAddress(uint8_t address, char* buf);

#endif /* INC_ADDRESSES_H_ */
