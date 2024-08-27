/*
 * addresses.c
 *
 *  Created on: May 14, 2024
 *      Author: thede
 */
#include <addresses.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

AddressNamePair lookupTable[] = {
    { C_BAE, "C_BAE" },
    { D_CALDERA, "D_CALDERA" },
    { L_CAPUTI, "L_CAPUTI" },
    { W_COLBURN, "W_COLBURN" },
    { D_CURIEL, "D_CURIEL" },
    { N_DELAPENA, "N_DELAPENA" },
    { A_DOSANJH, "A_DOSANJH" },
    { H_EVANS, "H_EVANS" },
    { M_FESLER, "M_FESLER" },
    { J_GALICINAO, "J_GALICINAO" },
    { T_GREEN, "T_GREEN" },
    { A_GROTE, "A_GROTE" },
    { M_HERRERA, "M_HERRERA" },
    { B_KENNEDY, "B_KENNEDY" },
    { J_KRAMMER, "J_KRAMMER" },
    { R_LEONTINI, "R_LEONTINI" },
    { J_MANESH, "J_MANESH" },
    { S_MARTIN, "S_MARTIN" },
    { N_MASTEN, "N_MASTEN" },
    { L_MCCARTHY, "L_MCCARTHY" },
    { P_MULPURU, "P_MULPURU" },
    { M_NOON, "M_NOON" },
    { J_PARK, "J_PARK" },
    { L_PEDROZA, "L_PEDROZA" },
    { D_PETERS, "D_PETERS" },
    { M_PROVINCE, "M_PROVINCE" },
    { A_RAJESH, "A_RAJESH" },
    { J_RAMIREZ, "J_RAMIREZ" },
    { D_ROBERDS, "D_ROBERDS" },
    { D_ROLAND, "D_ROLAND" },
    { D_SANDALL, "D_SANDALL" },
    { S_SELTZER, "S_SELTZER" },
    { J_SHAFFER, "J_SHAFFER" },
    { M_WONG, "M_WONG" },
	{A_TAYLOR, "A_TAYLOR"},
};

void getNameFromAddress(uint8_t address, char* buf)
{
	const char* name = NULL;
	for (uint8_t i = 0; i < NUM_ADDRESSES; i++)
	{
		if (lookupTable[i].address == address)
		{
			name = lookupTable[i].name;
			break;
		}
	}

	uint8_t len = strlen(name);
	memcpy(buf, name, len);
	buf[len] = '\0';
}
