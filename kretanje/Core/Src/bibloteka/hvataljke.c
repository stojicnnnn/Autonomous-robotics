/*
 * hvataljke.c
 *
 *  Created on: 17. mar 2024.
 *      Author: HP
 */

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdbool.h>
#include "biblioteka/hvataljke.h"
#include "biblioteka/uart.h"
#include "biblioteka/tajmer.h"
unsigned char Checksum;



void AX12Amove(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;

    const unsigned int length = 9;
    unsigned char packet[length];

	Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

    packet[0] = AX_START;
    packet[1] = AX_START;
    packet[2] = ID;
    packet[3] = AX_GOAL_LENGTH;
    packet[4] = AX_WRITE_DATA;
    packet[5] = AX_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Checksum;

    for (uint8_t i = 0; i < 9 ; i++)
           {
           uart_send_byte (packet[i]);
           }

}

void AX12Aturn(unsigned char ID, bool SIDE, int Speed)
{
		if (SIDE == LEFT)
		{
			char Speed_H,Speed_L;
			Speed_H = Speed >> 8;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables

			const unsigned int length = 9;
			unsigned char packet[length];

			Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;

		    packet[0] = AX_START;
		    packet[1] = AX_START;
		    packet[2] = ID;
		    packet[3] = AX_SPEED_LENGTH;
		    packet[4] = AX_WRITE_DATA;
		    packet[5] = AX_GOAL_SPEED_L;
		    packet[6] = Speed_L;
		    packet[7] = Speed_H;
		    packet[8] = Checksum;

		    for (uint8_t i = 0; i < 9 ; i++)
		              {
		              uart_send_byte (packet[i]);
		              }
		}

		else
		{
			char Speed_H,Speed_L;
			Speed_H = (Speed >> 8) + 4;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables

			const unsigned int length = 9;
			unsigned char packet[length];

			Checksum = (~(ID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;

		    packet[0] = AX_START;
		    packet[1] = AX_START;
		    packet[2] = ID;
		    packet[3] = AX_SPEED_LENGTH;
		    packet[4] = AX_WRITE_DATA;
		    packet[5] = AX_GOAL_SPEED_L;
		    packet[6] = Speed_L;
		    packet[7] = Speed_H;
		    packet[8] = Checksum;

		    for (uint8_t i = 0; i < 9 ; i++)
		              {
		              uart_send_byte (packet[i]);
		              }
		}
}

void AX12AsetEndless(unsigned char ID, bool Status)
{
	if ( Status )
	{
		const unsigned int length = 9;
		unsigned char packet[length];

		Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L)) & 0xFF;

	    packet[0] = AX_START;
	    packet[1] = AX_START;
	    packet[2] = ID;
	    packet[3] = AX_GOAL_LENGTH;
	    packet[4] = AX_WRITE_DATA;
	    packet[5] = AX_CCW_ANGLE_LIMIT_L;
	    packet[6] = 0; 						// full rotation
	    packet[7] = 0;						// full rotation
	    packet[8] = Checksum;

	    for (uint8_t i = 0; i < 9 ; i++)
	              {
	              uart_send_byte (packet[i]);
	              }
	}
	else
	{
		 AX12Aturn(ID,0,0);

		const unsigned int length = 9;
		unsigned char packet[length];

		Checksum = (~(ID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H)) & 0xFF;

	    packet[0] = AX_START;
	    packet[1] = AX_START;
	    packet[2] = ID;
	    packet[3] = AX_GOAL_LENGTH;
	    packet[4] = AX_WRITE_DATA;
	    packet[5] = AX_CCW_ANGLE_LIMIT_L;
	    packet[6] = AX_CCW_AL_L;
	    packet[7] = AX_CCW_AL_H;
	    packet[8] = Checksum;

	    for (uint8_t i = 0; i < 9 ; i++)
	              {
	              uart_send_byte (packet[i]);
	              }
	}
}

void hvataljka1_zatvori_biljke(uint32_t pocetno_vreme){
	//AX12AsetEndless(1, 0);
		  AX12Amove(1,100);
		  AX12Amove(1,100);
		  AX12Amove(1,100);
		  while(sistemsko_vreme < pocetno_vreme + 1000);
}

void hvataljka1_otvori(uint32_t pocetno_vreme){
	//AX12AsetEndless(1, 0);
		  AX12Amove(1,700);
		  AX12Amove(1,700);
		  AX12Amove(1,700);
		  while(sistemsko_vreme < pocetno_vreme + 300);
}

void hvataljka1_zatvori_saksije(uint32_t pocetno_vreme){
	//AX12AsetEndless(1, 0);
		  AX12Amove(1,200);
		  AX12Amove(1,200);
		  AX12Amove(1,200);
		  while(sistemsko_vreme < pocetno_vreme + 300);
}

void hvataljka2_zatvori_biljke(uint32_t pocetno_vreme){
	//AX12AsetEndless(3, 0);
		  AX12Amove(3,590);
		  AX12Amove(3,590);
		  AX12Amove(3,590);
		  while(sistemsko_vreme < pocetno_vreme + 500);
}

void hvataljka2_otvori(uint32_t pocetno_vreme){
	//AX12AsetEndless(3, 0);
		  AX12Amove(3,0);
		  AX12Amove(3,0);
		  AX12Amove(3,0);
		  while(sistemsko_vreme < pocetno_vreme + 300);
}

void hvataljka2_zatvori_saksije(uint32_t pocetno_vreme){
	//AX12AsetEndless(3, 0);
		  AX12Amove(3,440);
		  AX12Amove(3,440);
		  AX12Amove(3,440);
		   while(sistemsko_vreme < pocetno_vreme + 400);
}

void lift_dizanje_malo(uint32_t pocetno_vreme){
	//AX12AsetEndless(4, 1);
	set_det(2);
	AX12Aturn(4, 0, 911);

	while(sistemsko_vreme < pocetno_vreme + 100);

	AX12Aturn(4, 0, 0);
}

void lift_dizanje_vrh(uint32_t pocetno_vreme){
//	AX12AsetEndless(4, 1);
	set_det(2);
	AX12Aturn(4, 0, 911);
	AX12Aturn(4, 0, 911);
	AX12Aturn(4, 0, 911);
	while(sistemsko_vreme < pocetno_vreme + 1900);

	AX12Aturn(4, 0, 0);
	AX12Aturn(4, 0, 0);
	AX12Aturn(4, 0, 0);
}

void lift_spustanje(uint32_t pocetno_vreme){
//	AX12AsetEndless(4, 1);
	set_det(2);
	AX12Aturn(4, 1, 911);
	AX12Aturn(4, 1, 911);
	AX12Aturn(4, 1, 911);
	AX12Aturn(4, 1, 911);
	AX12Aturn(4, 1, 911);
	AX12Aturn(4, 1, 911);


	while(sistemsko_vreme < pocetno_vreme + 1200);

	AX12Aturn(4, 1, 0);
	AX12Aturn(4, 1, 0);
	AX12Aturn(4, 1, 0);
	AX12Aturn(4, 1, 0);
	AX12Aturn(4, 1, 0);

	AX12Aturn(4, 1, 0);

}


void rucica_napolje(uint32_t pocetno_vreme){
	AX12Amove(2,900);
	AX12Amove(2,900);
	AX12Amove(2,900);

	 while(sistemsko_vreme < pocetno_vreme + 300);
}

void rucica_unutra(uint32_t pocetno_vreme){
	AX12Amove(2,1023);
	AX12Amove(2,1023);
	AX12Amove(2,1023);
	 while(sistemsko_vreme < pocetno_vreme + 300);
}

void cekanje(uint32_t pocetno_vreme,uint32_t interval){
	while(sistemsko_vreme<=pocetno_vreme+interval);
}
