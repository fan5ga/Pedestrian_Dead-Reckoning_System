/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    imu.c
  * @author  KitSprout
  * @date    12-Nov-2016
  * @brief   
  * 
  */

/* Includes --------------------------------------------------------------------------------*/

#include "imu.h"

#include <stdio.h>
#include <string.h>

/** @addtogroup STM32_Module
  * @{
  */

/* Private typedef -------------------------------------------------------------------------*/
/* Private define --------------------------------------------------------------------------*/
#define IMU_SPIx                SPI1
#define IMU_SPIx_CLK_ENABLE()   __HAL_RCC_SPI1_CLK_ENABLE()
#define IMU_SPIx_SPEED_HIGH     SPI_BAUDRATEPRESCALER_2
#define IMU_SPIx_SPEED_LOW      SPI_BAUDRATEPRESCALER_256

#define IMU_SCK_PIN             GPIO_PIN_5
#define IMU_SCK_GPIO_PORT       GPIOA
#define IMU_SCK_AF              GPIO_AF5_SPI1

#define IMU_SDO_PIN             GPIO_PIN_6
#define IMU_SDO_GPIO_PORT       GPIOA
#define IMU_SDO_AF              GPIO_AF5_SPI1

#define IMU_SDI_PIN             GPIO_PIN_7
#define IMU_SDI_GPIO_PORT       GPIOA
#define IMU_SDI_AF              GPIO_AF5_SPI1
#define __MPU92
#define __USE_MAGNETOMETER

/* Private macro ---------------------------------------------------------------------------*/
/* Private variables -----------------------------------------------------------------------*/
//static SPI_HandleTypeDef IMU_HandleStruct;

IMU_DataTypeDef IMU;

/* Private function prototypes -------------------------------------------------------------*/
void IMU_InitData( IMU_DataTypeDef *imux );
void IMU_SetSensitivity(IMU_DataTypeDef *IMUx );
void IMU_MergeScaleStrength( IMU_DataTypeDef *imux );
//static void IMU_MergeScaleCalib( IMU_DataTypeDef *imux );

/* Private functions -----------------------------------------------------------------------*/

/**
  * @brief  IMU_SetSpeed
  * @param  speedSel: 
  * @retval None
  */


/**
  * @brief  IMU_GetRawData
  * @param  imux: 
  * @retval status
  */
//int8_t IMU_GetRawData( IMU_DataTypeDef *imux )
//{
//  int8_t status;
//  int16_t data[10];
//
//#if defined(__MPU92)
//  //status = MPU92_GetRawData(data);
//  imux->gyrRaw[0] = data[0];    /* Gyr.X */
//  imux->gyrRaw[1] = data[1];    /* Gyr.Y */
//  imux->gyrRaw[2] = data[2];    /* Gyr.Z */
//  imux->accRaw[0] = data[3];    /* Acc.X */
//  imux->accRaw[1] = data[4];    /* Acc.Y */
//  imux->accRaw[2] = data[5];    /* Acc.Z */
//  imux->ictempRaw = data[6];    /* ICTemp */
//
//#if defined(__USE_MAGNETOMETER)
//  if (status == 1) {
//    imux->magRaw[0] = data[7];  /* Mag.X */
//    imux->magRaw[1] = data[8];  /* Mag.Y */
//    imux->magRaw[2] = data[9];  /* Mag.Z */
//  }
//#endif
//#endif
//
//#if defined(__LPS22)
//#if defined(__USE_BAROMETER)
//  status = LPS22_GetRawData(data);
//#endif
//#endif
//  return status;
//}

/**
  * @brief  IMU_GetCalibData
  * @param  imux: 
  * @retval None
  */
void IMU_GetCalibData( IMU_DataTypeDef *imux )
{
  double tmp[3] = {0};

  //IMU_GetRawData(imux);

  imux->gyrData[0] = (imux->gyrRaw[0] - imux->gyrOffset[0]) * imux->gyrCalib[0];   /* Gyr.X */
  imux->gyrData[1] = (imux->gyrRaw[1] - imux->gyrOffset[1]) * imux->gyrCalib[1];   /* Gyr.Y */
  imux->gyrData[2] = (imux->gyrRaw[2] - imux->gyrOffset[2]) * imux->gyrCalib[2];   /* Gyr.Z */

  imux->accData[0] = (imux->accCalib[0] * imux->accRaw[0] + imux->accCalib[1] * imux->accRaw[1] + imux->accCalib[2] * imux->accRaw[2]) + imux->accOffset[0];  /* Acc.X */
  imux->accData[1] = (imux->accCalib[3] * imux->accRaw[0] + imux->accCalib[4] * imux->accRaw[1] + imux->accCalib[5] * imux->accRaw[2]) + imux->accOffset[1];  /* Acc.X */
  imux->accData[2] = (imux->accCalib[6] * imux->accRaw[0] + imux->accCalib[7] * imux->accRaw[1] + imux->accCalib[8] * imux->accRaw[2]) + imux->accOffset[2];  /* Acc.X */

  imux->ictempData = imux->ictempRaw * imux->ictempScale + imux->ictempOffset;

#if defined(__USE_MAGNETOMETER)
  tmp[0] = imux->magRaw[0] - imux->magOffset[0];   /* Mag.X */
  tmp[1] = imux->magRaw[1] - imux->magOffset[1];   /* Mag.Y */
  tmp[2] = imux->magRaw[2] - imux->magOffset[2];   /* Mag.Z */

  imux->magData[0] = imux->magCalib[0] * tmp[0] + imux->magCalib[1] * tmp[1] + imux->magCalib[2] * tmp[2];
  imux->magData[1] = imux->magCalib[3] * tmp[0] + imux->magCalib[4] * tmp[1] + imux->magCalib[5] * tmp[2];
  imux->magData[2] = imux->magCalib[6] * tmp[0] + imux->magCalib[7] * tmp[1] + imux->magCalib[8] * tmp[2];
#endif

#if defined(__USE_BAROMETER)
  imux->baroData[0] = imux->baroData[0] - imux->TempOffset;   /* Baro.T */
  imux->baroData[1] = imux->baroData[1] - imux->PresOffset;   /* Baro.P */
#endif
}

/**
  * @brief  IMU_GetScaleData
  * @param  pIMU: 
  * @retval None
  */
void IMU_GetScaleData( IMU_DataTypeDef *imux )
{
  //IMU_GetCalibData(imux);

  imux->gyrData[0] = imux->gyrData[0] * imux->gyrScale[0];    /* Gyr.X */
  imux->gyrData[1] = imux->gyrData[1] * imux->gyrScale[1];    /* Gyr.Y */
  imux->gyrData[2] = imux->gyrData[2] * imux->gyrScale[2];    /* Gyr.Z */

  imux->accData[0] = imux->accData[0] * imux->accScale[0];    /* Acc.X */
  imux->accData[1] = imux->accData[1] * imux->accScale[1];    /* Acc.Y */
  imux->accData[2] = imux->accData[2] * imux->accScale[2];    /* Acc.Z */

#if defined(__USE_MAGNETOMETER)
  imux->magData[0] = imux->magData[0] * imux->magScale[0];    /* Mag.X */
  imux->magData[1] = imux->magData[1] * imux->magScale[1];    /* Mag.Y */
  imux->magData[2] = imux->magData[2] * imux->magScale[2];    /* Mag.Z */
#endif
}

/**
  * @brief  IMU_InitData
  * @param  imu: 
  * @retval None
  */
void IMU_InitData( IMU_DataTypeDef *imux )
{
  //memset(imux, 0, sizeof(IMU_DataTypeDef));

  imux->gyrCalib[0] = 1.0f;
  imux->gyrCalib[1] = 1.0f;
  imux->gyrCalib[2] = 1.0f;

  imux->accStrength = 1.0f;
  imux->accCalib[0] = 1.0f;
  imux->accCalib[4] = 1.0f;
  imux->accCalib[8] = 1.0f;

  imux->magStrength = 1.0f;
  imux->magCalib[0] = 1.0f;
  imux->magCalib[4] = 1.0f;
  imux->magCalib[8] = 1.0f;


}

/**
  * @brief  IMU_SetSensitivity
  * @param  IMUx: 
  * @retval None
  */


void MPU9250_GetSensitivity(double *sensitivity)
{
	double scale;

	/* Set gyroscope sensitivity (dps/LSB) */

	scale = 8.0; //8g
	sensitivity[0] = scale / 32768.0;

	scale = 1000; //1000dps gyro
	sensitivity[1] = scale / 32768.0;


	sensitivity[2] = 0.3;  /* +-1200uT */


	sensitivity[3] = 1.0 / 333.87;
	sensitivity[4] = 21.0f;

}

void IMU_SetSensitivity(IMU_DataTypeDef *IMUx )
{
  double scale[5];

#if defined(__MPU92)
  MPU9250_GetSensitivity(scale);


  /* Set accelerometer sensitivity (g/LSB) */
  IMUx->accScale[0] = scale[0];
  IMUx->accScale[1] = scale[0];
  IMUx->accScale[2] = scale[0];


  /* Set gyroscope sensitivity (dps/LSB) */
  IMUx->gyrScale[0] = scale[1];
  IMUx->gyrScale[1] = scale[1];
  IMUx->gyrScale[2] = scale[1];

  /* Set magnetometer sensitivity (uT/LSB) */
  IMUx->magScale[0] = scale[2];
  IMUx->magScale[1] = scale[2];
  IMUx->magScale[2] = scale[2];

  /* Set ictemperature sensitivity (degC/LSB) */
  IMUx->ictempScale  = scale[3];
  IMUx->ictempOffset = scale[4];
#endif

#if defined(__LPS22)
  LPS22_GetSensitivity(&IMUx->InitLPS, scale);

  /* Set barometer sensitivity (degC/LSB, hPa/LSB) */
  IMUx->baroScale[0] = scale[0];
  IMUx->baroScale[1] = scale[1];
#endif
}

/**
  * @brief  IMU_MergeScaleStrength
  * @param  pIMU: 
  * @retval None
  */
void IMU_MergeScaleStrength( IMU_DataTypeDef *imux )
{


	/* Merge accelerometer scale and sensitivity (g/LSB) */
	imux->accScale[0] = imux->accScale[0] * imux->accStrength;
	imux->accScale[1] = imux->accScale[1] * imux->accStrength;
	imux->accScale[2] = imux->accScale[2] * imux->accStrength;


	/* Merge magnetometer scale and sensitivity (uT/LSB) */
	imux->magScale[0] = imux->magScale[0] * imux->magStrength;
	imux->magScale[1] = imux->magScale[1] * imux->magStrength;
	imux->magScale[2] = imux->magScale[2] * imux->magStrength;


}

/**
  * @brief  IMU_MergeScaleCalib
  * @param  pIMU: 
  * @retval None
  */
//static void IMU_MergeScaleCalib( IMU_DataTypeDef *imux )
//{
//  /* Merge gyroscope scale and calibration (dps/LSB) */
//  imux->gyrCalib[0] = imux->gyrCalib[0] * imux->gyrScale[0];
//  imux->gyrCalib[1] = imux->gyrCalib[1] * imux->gyrScale[1];
//  imux->gyrCalib[2] = imux->gyrCalib[2] * imux->gyrScale[2];

//  /* Merge accelerometer scale and sensitivity (g/LSB) */
//  imux->accCalib[0] = imux->accCalib[0] * imux->accScale[0];
//  imux->accCalib[1] = imux->accCalib[1] * imux->accScale[1];
//  imux->accCalib[2] = imux->accCalib[2] * imux->accScale[2];
//  imux->accCalib[3] = imux->accCalib[3] * imux->accScale[0];
//  imux->accCalib[4] = imux->accCalib[4] * imux->accScale[1];
//  imux->accCalib[5] = imux->accCalib[5] * imux->accScale[2];
//  imux->accCalib[6] = imux->accCalib[6] * imux->accScale[0];
//  imux->accCalib[7] = imux->accCalib[7] * imux->accScale[1];
//  imux->accCalib[8] = imux->accCalib[8] * imux->accScale[2];

//  /* Merge magnetometer scale and sensitivity (uT/LSB) */
//  imux->magCalib[0] = imux->magCalib[0] * imux->magScale[0];
//  imux->magCalib[1] = imux->magCalib[1] * imux->magScale[1];
//  imux->magCalib[2] = imux->magCalib[2] * imux->magScale[2];
//  imux->magCalib[3] = imux->magCalib[3] * imux->magScale[0];
//  imux->magCalib[4] = imux->magCalib[4] * imux->magScale[1];
//  imux->magCalib[5] = imux->magCalib[5] * imux->magScale[2];
//  imux->magCalib[6] = imux->magCalib[6] * imux->magScale[0];
//  imux->magCalib[7] = imux->magCalib[7] * imux->magScale[1];
//  imux->magCalib[8] = imux->magCalib[8] * imux->magScale[2];
//}

/**
  * @brief  IMU_PrintData
  * @param  pIMU: 
  * @retval None
  */
void IMU_PrintData( IMU_DataTypeDef *imux )
{
  printf("\r\n");
  printf("- Print IMU Data -----------------\r\n");
  printf("G_raw : %f, %f, %f\r\n", imux->gyrRaw[0], imux->gyrRaw[1], imux->gyrRaw[2]);
  printf("A_raw : %f, %f, %f\r\n", imux->accRaw[0], imux->accRaw[1], imux->accRaw[2]);
#if defined(__USE_MAGNETOMETER)
  printf("M_raw : %f, %f, %f\r\n", imux->magRaw[0], imux->magRaw[1], imux->magRaw[2]);
#endif
  printf("T_raw : %f\r\n", imux->ictempRaw);
#if defined(__USE_BAROMETER)
  printf("B_raw : %f, %f\r\n", imux->baroRaw[0], imux->baroRaw[1]);
#endif

  printf("\r\n");
  printf("G_offset : %f, %f, %f\r\n", imux->gyrOffset[0], imux->gyrOffset[1], imux->gyrOffset[2]);
  printf("A_offset : %f, %f, %f\r\n", imux->accOffset[0], imux->accOffset[1], imux->accOffset[2]);
#if defined(__USE_MAGNETOMETER)
  printf("M_offset : %f, %f, %f\r\n", imux->magOffset[0], imux->magOffset[1], imux->magOffset[2]);
#endif
  printf("T_offset : %f\r\n", imux->ictempOffset);

  printf("\r\n");
  printf("G_data : %f, %f, %f\r\n", imux->gyrData[0], imux->gyrData[1], imux->gyrData[2]);
  printf("A_data : %f, %f, %f\r\n", imux->accData[0], imux->accData[1], imux->accData[2]);
#if defined(__USE_MAGNETOMETER)
  printf("M_data : %f, %f, %f\r\n", imux->magData[0], imux->magData[1], imux->magData[2]);
#endif
  printf("T_data : %f\r\n", imux->ictempData);
#if defined(__USE_BAROMETER)
  printf("B_data : %f, %f\r\n", imux->baroData[0], imux->baroData[1]);
#endif

  printf("\r\n");
  printf("G_calib : %f, %f, %f\r\n", imux->gyrCalib[0], imux->gyrCalib[1], imux->gyrCalib[2]);
  printf("A_calib : %f, %f, %f\r\n", imux->accCalib[0], imux->accCalib[1], imux->accCalib[2]);
  printf("          %f, %f, %f\r\n", imux->accCalib[3], imux->accCalib[4], imux->accCalib[5]);
  printf("          %f, %f, %f\r\n", imux->accCalib[6], imux->accCalib[7], imux->accCalib[8]);
#if defined(__USE_MAGNETOMETER)
  printf("M_calib : %f, %f, %f\r\n", imux->magCalib[0], imux->magCalib[1], imux->magCalib[2]);
  printf("          %f, %f, %f\r\n", imux->magCalib[3], imux->magCalib[4], imux->magCalib[5]);
  printf("          %f, %f, %f\r\n", imux->magCalib[6], imux->magCalib[7], imux->magCalib[8]);
#endif

  printf("\r\n");
  printf("G_scale : %f, %f, %f\r\n", imux->gyrScale[0], imux->gyrScale[1], imux->gyrScale[2]);
  printf("A_scale : %f, %f, %f\r\n", imux->accScale[0], imux->accScale[1], imux->accScale[2]);
#if defined(__USE_MAGNETOMETER)
  printf("M_scale : %f, %f, %f\r\n", imux->magScale[0], imux->magScale[1], imux->magScale[2]);
#endif
  printf("T_scale : %f\r\n", imux->ictempScale);
#if defined(__USE_BAROMETER)
  printf("B_scale : %f, %f\r\n", imux->baroScale[0], imux->baroScale[1]);
#endif

  printf("\r\n");
#if defined(__USE_MAGNETOMETER)
  printf("A_strength : %f\r\n", imux->accStrength);
  printf("M_strength : %f\r\n", imux->magStrength);
#endif

  printf("----------------------------------\r\n");
  printf("\r\n\r\n");
}

/*************************************** END OF FILE ****************************************/
