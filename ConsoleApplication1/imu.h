/**
  *      __            ____
  *     / /__ _  __   / __/                      __  
  *    / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
  *   / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
  *  /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
  *                    /_/   github.com/KitSprout    
  * 
  * @file    imu.h
  * @author  KitSprout
  * @date    12-Nov-2016
  * @brief   
  * 
  */

/* Define to prevent recursive inclusion ---------------------------------------------------*/
#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------------------*/
#include "stdint.h"


//#include "modules\lps22hb.h"

/* Exported types --------------------------------------------------------------------------*/
typedef float float32_t;
typedef struct {
  double gyrRaw[3];      /* x = raw[0], y = raw[1], z = raw[2] */
  double accRaw[3];
  double magRaw[3];
  double ictempRaw;
  double baroRaw[2];     /* t = raw[0], p = raw[1] */

  double gyrData[3];
  double accData[3];
  double magData[3];
  double ictempData;
  double baroData[2];

  double accMotion[3];

  double gyrScale[3];
  double accScale[3];
  double magScale[3];
  double ictempScale;
  double baroScale[2];

  double gyrCalib[3];
  double accCalib[9];
  double magCalib[9];    /* c11 = calib[0], c12 = calib[1], c13 = calib[2],
                               c21 = calib[3], c22 = calib[4], c23 = calib[5],
                               c31 = calib[6], c32 = calib[7], c33 = calib[8] */

  double gyrOffset[3];
  double accOffset[3];
  double magOffset[3];
  double ictempOffset;

  double accStrength;
  double magStrength;

}  IMU_DataTypeDef;

typedef struct {
  IMU_DataTypeDef *Data;

#if defined(__MPU92)
  MPU_ConfigTypeDef InitMPU;
#endif

#if defined(__LPS22)
  MPU_ConfigTypeDef InitLPS;
#endif

} IMU_InitTypeDef;

/* Exported constants ----------------------------------------------------------------------*/
/* Exported functions ----------------------------------------------------------------------*/  
//void    IMU_Config( void );
//int8_t  IMU_Init( IMU_InitTypeDef *IMUx );
void IMU_InitData(IMU_DataTypeDef *imux);
void MPU9250_GetSensitivity(double *sensitivity);
void IMU_SetSensitivity(IMU_DataTypeDef *IMUx);
void IMU_MergeScaleStrength(IMU_DataTypeDef *imux);

void    IMU_GetCalibData( IMU_DataTypeDef *IMUx );
void    IMU_GetScaleData( IMU_DataTypeDef *IMUx );
void    IMU_PrintData( IMU_DataTypeDef *IMUx );

#ifdef __cplusplus
}
#endif

#endif

/*************************************** END OF FILE ****************************************/
