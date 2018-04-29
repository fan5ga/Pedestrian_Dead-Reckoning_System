#include<iostream>

#include "imu.h"

#include "NewSerial/Serial.h"
#include "NewSerial/stdafx.h"

//thalx includes 

using namespace std;

#define BUFFSIZE 200







int _tmain(void)
{

	IMU_DataTypeDef IMUx;
	IMU_DataTypeDef *IMU = &IMUx;
	//char buffer[BUFFSIZE];
	IMU_InitData(IMU);
	IMU_SetSensitivity(IMU);
	IMU_MergeScaleStrength(IMU);

	try 
	{
		
		tstring commPortName(TEXT("COM3"));
		Serial serial(commPortName);
		std::cout << "Port opened" << std::endl;
		serial.flush();


		unsigned char buffer[40];
		char oneByte[2];

		int tCount = 0;
		unsigned char lastByte = 0x00;
		while (1)
		{

			int charRead = serial.read(oneByte, 1, NULL);
			if (oneByte[0] == 0x0a && lastByte == 0x0d)
			{

				if (tCount != 25)
				{
					memset(buffer, 0, sizeof(buffer));
					tCount = 0;
					continue;
				}
				tCount = 0;
				
				int16_t data[10];

				data[0] = (int16_t)(buffer[4] << 8) | buffer[5];   /* acc.X */
				data[1] = (int16_t)(buffer[6] << 8) | buffer[7];  /* acc.Y */
				data[2] = (int16_t)(buffer[8] << 8) | buffer[9];  /* accZ */
				data[3] = (int16_t)(buffer[10] << 8) | buffer[11];   /* gyr.X */
				data[4] = (int16_t)(buffer[12] << 8) | buffer[13];   /* gyr.Y */
				data[5] = (int16_t)(buffer[14] << 8) | buffer[15];   /* gyr.Z */
				data[6] = (int16_t)(buffer[16] << 8) | buffer[17];   /* mag */
				data[7] = (int16_t)(buffer[18] << 8) | buffer[19];   /* mag.Z */
				data[8] = (int16_t)(buffer[20] << 8) | buffer[21];   /* mag */
				data[9] = (int16_t)(buffer[22] << 8) | buffer[23];   /* ICTEM */
				//printf("%05d, %05d,%05d\r\n", data[0], data[1], data[2]);

				IMU->accData[0] = data[0];    /* Acc.X */
				IMU->accData[1] = data[1];    /* Acc.Y */
				IMU->accData[2] = data[2];    /* Acc.Z */
				IMU->gyrData[0] = data[3];    /* Gyr.X */
				IMU->gyrData[1] = data[4];    /* Gyr.Y */
				IMU->gyrData[2] = data[5];    /* Gyr.Z */
				IMU->magData[0] = data[6];  /* Mag.X */
				IMU->magData[1] = data[7];  /* Mag.Y */
				IMU->magData[2] = data[8];  /* Mag.Z */

				IMU->ictempData = data[6];    /* ICTemp */


				IMU_GetScaleData(IMU);

				printf("Acc : %.3lf, %.3lf, %.3lf\r\n", IMU->accData[0], IMU->accData[1], IMU->accData[2]);
				printf("Gyro: %.3lf, %.3lf, %.3lf\r\n", IMU->gyrData[0], IMU->gyrData[1], IMU->gyrData[2]);
				printf("Mag : %.3lf, %.3lf, %.3lf\r\n\r\n", IMU->magData[0], IMU->magData[1], IMU->magData[2]);
				


				memset(buffer, 0, sizeof(buffer));
			}
			else
			{
				lastByte = oneByte[0];
				if (tCount<25)
				{
					buffer[tCount] = oneByte[0];
				}
				
				tCount++;
			}
		}
	}catch(const char *msg)
	{
		std::cout << msg << std::endl;
	}

	
}