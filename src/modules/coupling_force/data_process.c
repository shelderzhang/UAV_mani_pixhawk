/*
 * data_process.c
 *
 *  Created on: 2016-1-21
 *      Author: gyzhang
 */
#include <stdio.h>
#include<modules/coupling_force/data_process.h>
void force_sensor_data_decode(float* resultData, char* buff)
{
	uint8_t i;
	uint8_t Index;
	uint8_t k ;
	uint16_t ADcounts[6];
	float m_dResultChValue[6];
	float m_dGain[6];
	float m_dAmpZero[6];
	float decoupled[6][6]=
		{
			{0.71643,0.34509,-0.30487,47.44053,-0.03246,-48.23680},
			{1.10584,-55.48118,0.74791,27.74202,1.31624,27.56464},
			{154.75180,-0.29425,154.29744,-2.49127,154.61993,0.04028},
			{-0.04923,-0.01943,-1.19667,0.01850,1.08773,0.01559},
			{1.33007,-0.01110,-0.65628,-0.00682,-0.62841,0.01825},
			{-0.00990,0.44900,0.00174,0.42497,0.00538,0.42179}

	    };

		m_dGain[0] = 123.519878;
		m_dGain[1] = 123.453219;
		m_dGain[2] = 123.580741;
		m_dGain[3] = 123.490896;
		m_dGain[4] = 123.395254;
		m_dGain[5] = 123.554657;

	  m_dAmpZero[0] = 32715.000000;
      m_dAmpZero[1] = 32693.000000;
		m_dAmpZero[2] = 32689.000000;
		m_dAmpZero[3] = 32681.000000;
		m_dAmpZero[4] = 32703.000000;
		m_dAmpZero[5] = 32735.000000;

	if((buff[0] == 0xAA) && (buff[1] == 0x55))
		{
			Index = 6;
			char CheckSum = 0x00;
		    for(i = 0x00;i < 0x0c;i++)
			{
				CheckSum += buff[Index];//
				Index++;
			}
			if(CheckSum == buff[Index])//
			{

                Index = 6;

                for(k = 0x00;k < 6; k++)
                {
                    ADcounts[k] = buff[Index]*256 + buff[Index+1]; //AD
                    m_dResultChValue[k] =   1000*( ADcounts[k]-m_dAmpZero[k])/65535*5/m_dGain[k]/2.503526f;
//                    m_dResultChValue[k] =   1000*5*( ADcounts[k]-m_dAmpZero[k])/(65535*m_dGain[k]*2.5);
                    Index = Index + 2;
                }
                for(i = 0x00;i < 6; i++)
                {
                	resultData[i] = m_dResultChValue[0]*decoupled[i][0]+m_dResultChValue[1]*decoupled[i][1]
			                   +m_dResultChValue[2]*decoupled[i][2]+m_dResultChValue[3]*decoupled[i][3]
			                   +m_dResultChValue[4]*decoupled[i][4]+m_dResultChValue[5]*decoupled[i][5];
                }
				resultData[0]=resultData[0]+2.1482f;
                resultData[1]=resultData[1]+1.1133f;
                resultData[2]=resultData[2]+1.1122f;
                resultData[3]=resultData[3]+0.0261f;
                resultData[4]=resultData[4]+0.022f;
                resultData[5]=resultData[5]+0.0208f;
			}

		}

}


