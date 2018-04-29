
#include<stdio.h>
#include<iostream>
#include<math.h>
#include<stdlib.h>
#include<string.h>
#include<windows.h>


#define OPENCV
#ifdef OPENCV
#include <imgproc.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
#endif


//parsing codes includes
#include<iostream>

#include "imu.h"

#include "NewSerial/Serial.h"
#include "NewSerial/stdafx.h"
//define keyboard button
#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8000) ? 1:0)
//define buufer (paring)
#define BUFFSIZE 200
float fuzzify_MF(float x, float a, float b, float c, float d);
float Fuzzy(float input);
double MovAvgFilter(double data, char nn, char num);
void StepAnalPDRSystem(double ang);
int SIGN(double sign);
void MatrixAddition(double* A, double* B, int mm, int nn, double* C);
void MatrixTranspose(double* A, int mm, int nn, double* C);
void MatrixSubtraction(double* A, double* B, int mm, int nn, double* C);
void MatrixMultiply(double* A, double* B, int mm, int pp, int nn, double* C);
int MatrixInversion(double* A, int nn, double* AInverse);
double EulerEKF_PSI(double z, double rates);
double EulerEKF_PSIG(double z, double rates);
void EulerEKFG_NEW(double z[], char m1, char n1, double rates[], char m2, char n2, double init, double *angle);
void stepDetection();

#define BUFFER_SIZE 200
#define YES 1
#define NO 0
#define GRAVITY 9.81

#define D2R  3.14159/180.0
#define R2D  180.0/3.14159
#define dt 0.01

//#define GyroZ_e0  0.05806; 
//#define GyroY_e0  0.07561;
//#define GyroX_e0  0.07869;
#define GyroZ_e0  0; 
#define GyroY_e0  0;
#define GyroX_e0  0;

bool NINE_AXIS_ENABLE = 0;
bool SIX_AXIS_ENABLE = 0;
bool HDR_ENABLE = 0;
bool HDR_DISABLE = 0;
bool CALI_END = 0;

//define steps number

// buffers of IMU
double gyro1[BUFFER_SIZE][3];   // Gyroscop buffer
double acc1[BUFFER_SIZE][3];    // Acceleromter buffer
double com1[BUFFER_SIZE][3];    // Magnetometer buffer
char str1[200];                 // IMU sensor data buffer

								// parameters of the parsing IMU process
unsigned int star1 = 1;           // IMU parsing : the number of "*" in raw data
unsigned int comma1 = 0;        // IMU parsing : the number of ',' in raw data
unsigned int char_N = 0;          // IMU parsing : the number of character between two ","
unsigned int star1_last = 0;    // IMU parsing : last number of '*' in IMU raw data


char Calibrated = 0;          // caliburate process flag
double com_max[3] = { -200, -200, -200 };
double com_min[3] = { 200, 200, 200 };
double comLowX_max = 0;         // Calibration: the maximum of Magnetometer sensor X axis data
double comLowX_min = 0;         // Calibration: the minimum of Magnetometer sensor X axis data
double comLowY_max = 0;         // Calibration: the maximum of Magnetometer sensor Y axis data
double comLowY_min = 0;         // Calibration: the minimum of Magnetometer sensor Y axis data
double comLowZ_max = 0;         // Calibration: the maximum of Magnetometer sensor Z axis data
double comLowZ_min = 0;         // Calibration: the minimum of Magnetometer sensor Z axis data
double com_hord_factor_X = 0;        // Calibration: calibration X axis value
double com_hord_factor_Y = 0;      // Calibration: calibration Y axis value
double com_hord_factor_Z = 0;        // Calibration: calibration Z axis value

char location[200] = { 0 };          // location buffer

									 // step detect parameters
double max_thread = 1.1*GRAVITY;  // the max peak threshold value of Step detecting process
double min_thread = 0.95*GRAVITY; // the min peak threshold value of Step detecting process
double max_min_detect = 0;        // Max and min peak detect value
double min_detect = 0;            // Min peak value
double max_detect = 0;            // Max peak value
double pre_max = 1.1*GRAVITY;     // Previous max peak value
double pre_min = 0.95*GRAVITY;    // Previous min peak value
char min_flag = 0;                // Min peak flag
char max_flag = 0;                // Max peak flag
char zero_flag = 0;               // Zero crossing flag
unsigned char max_check = 0;      // Max peak check flag
unsigned char min_check = 0;      // Min peak check flag
unsigned char sgn[2] = { 0 };       // sgn buffer
double step_length = 0;           // The distance of the detected step
double distance = 0;              // Total distance
double diffmaxmin = 0;            // The distance between max peak and mean peak 

double K = 0.425;                 // K value
double crawl_K = 0.19;            // K value for crawling

								  // pamameters of the move average filter
double acc_3D_filter[3] = { 0 };              // Accelerometer 3-axis fusion data filter buffer
char acc_3D_num[9] = { 0 };                   // Accelerometer 3-axis fusion data filter buffer
char IMU_buffer_ready[9] = { 0 };             // IMU data(9 data) ready flag array
double acc_3D_filter_sum[9] = { 0 };          // IMU filter parameter
double acc_3D_filter_buffer[9][20] = { 0 };   // IMU filter buffer

											  // steps detecting parameters
unsigned int steps;             // Steps number

								//x, y direction parameters
double heading = 0;         // Heading degree
double heading_psi = 0;         // PSI angle
double heading_PDR = 0;         // PDR heading
double heading_temp = 0;        // Temporary heading value
double psi_init = 0;              // PSI init flag

								  // Euler variable
double p = 0;            // EulerEKF_PSI function paramter
double q = 0;            // EulerEKF_PSI function paramter
double r = 0;            // EulerEKF_PSI function paramter
double px_pre = 0;       // X axis distance
double py_pre = 0;       // Y axis distance

						 // hdr
double ic = 0.001;       // HDR process paramter
double threshold_H = 10; // HDR process paramter
double HDR_I_0 = 0;      // HDR process paramter
double HDR_I_1 = 0;      // HDR process paramter
double HDR_I_2 = 0;      // HDR process paramter

						 // phi buffer
double phi_buf[2] = { 0 };     // PSI buffer     
int psi_up_cnt = 0;          // EulerEKF_PSI function paramter
int psi_up_cnt_i = 0;        // EulerEKF_PSI function paramter
int psi_up_cnt_i_flag = 20;  // EulerEKF_PSI function paramter    
double psi_gy_r = 0;         // PSI parameter

							 // int test_num = 9;               
char fuzzy_applied = 1;           // Flag of fuzzy condition
unsigned int direciton_begin = 0; // Direction begin flag
char crawling = 0;                // Flag of crawling

								  // button                          
char button_N = 0;                // The times of button pressed
char step_detection_begin = 1;    // Step detection process flag
								  //void imu_parsing(void);
char char_to_hex(char aaa);
int main(void)
{
	int a = 0;
	double x_tmp2 = 0;
	double y_tmp2 = 0;
#ifdef OPENCV
	int x_tmp = 0;
	int y_tmp = 0;
	Mat graph(400, 400, CV_8UC3, Scalar::all(255));
	Point org(graph.cols / 2, graph.rows / 2);
	namedWindow("graph", 1);
	char *imageName = "graph.bmp";
#endif


	IMU_DataTypeDef IMUx;
	IMU_DataTypeDef *IMU = &IMUx;
	//char buffer[BUFFSIZE];
	IMU_InitData(IMU);
	IMU_SetSensitivity(IMU);
	IMU_MergeScaleStrength(IMU);

	try
	{


		FILE *fp;
		fp = fopen("7.csv", "wt");
		FILE *fp_imu;
		fp_imu = fopen("0525_fast.txt","r");
		FILE *fp_imu_cali;
		fp_imu_cali = fopen("0525_cali.txt","r");

		//tstring commPortName(TEXT("COM3"));
		//Serial serial(commPortName);
		//std::cout << "Port opened" << std::endl;
		//serial.flush();


		// HDR Algorithm ENABLE OR DISABLE select
		bool ERROR_INPUT = 1;
		while (ERROR_INPUT)
		{
			std::cout << "Select Using HDR\n - 1: HDR ENABLE\n - 2: HDR DISABLE" << std::endl;
			char str;
			std::cin >> str;
			if (str == '1')
			{
				HDR_ENABLE = 1;
				HDR_DISABLE = 0;
				printf("HDR ENABLE !\n");
				break;
			}
			else if (str == '2')
			{
				HDR_ENABLE = 0;
				HDR_DISABLE = 1;
				printf("HDR DISABLE !\n");
				break;
			}
			else
			{
				std::cout << "Error Input!!" << std::endl;

				continue;
			}
		}

		//9 axis or 6 axis modify
		//while (ERROR_INPUT)
		//{
		//	std::cout << "Select Using Compass\n - 1: COMPASS ENABLE\n - 2: COMPASS DISABLE" << std::endl;
		//	char str;
		//	std::cin >> str;
		//	if (str == '2')
		//	{
		//		SIX_AXIS_ENABLE = 1;
		//		NINE_AXIS_ENABLE = 0;
		//		printf("USING GYRO, ACC\n");
		//		break;
		//	}
		//	else if (str == '1')
		//	{
		//		SIX_AXIS_ENABLE = 0;
		//		NINE_AXIS_ENABLE = 1;
		//		std::cout << "COMPASS ENABLE !\nCOMPASS calibration begins!!\nTo end calibration, Please press 'C'..." << std::endl;
		//		break;
		//	}
		//	else
		//	{
		//		std::cout << "Error Input!!" << std::endl;

		//		continue;
		//	}
		//}

		int ccc = 0;
		//serial.write("s");
		//serial.write("d");
		unsigned char buffer[40];
		char oneByte[2];

		int tCount = 0;
		unsigned char lastByte = 0x00;
		while (1)
		{
			int charRead;
			if (Calibrated == 0)
			{
				oneByte[0] = fgetc(fp_imu_cali);
				ccc++;
				if (oneByte[0] == EOF)
				{
					CALI_END = 1;
					oneByte[0] = fgetc(fp_imu);
					if (CALI_END == 1)
					{
						Calibrated = 1;
						com_hord_factor_X = (com_min[0] + com_max[0]) / 2;
						com_hord_factor_Y = (com_min[1] + com_max[1]) / 2;
						com_hord_factor_Z = (com_min[2] + com_max[2]) / 2;
						printf("X : %.3lf, Y : %.3lf, Z : %.3lf\n", com_hord_factor_X, com_hord_factor_Y, com_hord_factor_Z);
					}

				}
			}
			else
			{
				oneByte[0] = fgetc(fp_imu);
				if(oneByte[0] == EOF)
				{
					imwrite(imageName, graph);
					return 0;
				}
			}
			//charRead = serial.read(oneByte, 1, NULL);
			if (oneByte[0] == '\n' )
			{

				if (tCount != 48)
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
				data[6] = (int16_t)(buffer[16] << 8) | buffer[17];   /* mag x*/
				data[7] = (int16_t)(buffer[18] << 8) | buffer[19];   /* mag.y */
				data[8] = (int16_t)(buffer[20] << 8) | buffer[21];   /* mag z*/
				data[9] = (int16_t)(buffer[22] << 8) | buffer[23];   /* ICTEM */
																	 //printf("%05d, %05d,%05d\r\n", data[0], data[1], data[2]);
				IMU->accData[0] = data[0];    /* Acc.X */
				IMU->accData[1] = data[1];    /* Acc.Y */
				IMU->accData[2] = data[2];    /* Acc.Z */
				IMU->gyrData[0] = data[3];    /* Gyr.X */
				IMU->gyrData[1] = data[4];    /* Gyr.Y */
				IMU->gyrData[2] = data[5];    /* Gyr.Z */
				IMU->magData[0] = data[7];  /* Mag.X */
				IMU->magData[1] = data[8];  /* Mag.Y */
				IMU->magData[2] = data[6];  /* Mag.Z */

				IMU->ictempData = data[6];    /* ICTemp */


				IMU_GetScaleData(IMU);
				a++;
				//printf("Acc : %.3lf, %.3lf, %.3lf]\r\n", IMU->accData[0], IMU->accData[1], IMU->accData[2]);
				//printf("Gyro: %.3lf, %.3lf, %.3lf\r\n", IMU->gyrData[0], IMU->gyrData[1], IMU->gyrData[2]);
				//printf("Mag : %.3lf, %.3lf, %.3lf\r\n\r\n", IMU->magData[0], IMU->magData[1], IMU->magData[2]);
				//if (a % 10 == 1) {
				//	printf("%.3lf, %.3lf, %.3lf,", IMU->gyrData[0], IMU->gyrData[1], IMU->gyrData[2]);
				//	printf("*%.3lf, %.3lf, %.3lf,", IMU->accData[0], IMU->accData[1], IMU->accData[2]);
				//	printf("%.3lf, %.3lf, %.3lf\r\n", IMU->magData[0], IMU->magData[1], IMU->magData[2]);
				//}
				//fprintf(fp, "%.4lf, %.4lf, %.4lf, %.10f, %.4lf, %.4lf, %.4lf, %.4lf, %.4lf \n", IMU->gyrData[0], IMU->gyrData[1], IMU->gyrData[2],
				//	IMU->accData[0], IMU->accData[1], IMU->accData[2], IMU->magData[0], IMU->magData[1], IMU->magData[2]);
				//continue;
				if (Calibrated == 0)
				{
					double com_tmp = 0;
					for (int i = 0; i < 3; i++)
					{
						com_tmp = IMU->magData[i];
						if (com_tmp < com_min[i])
						{
							com_min[i] = com_tmp;
						}
						if (com_tmp > com_max[i])
						{
							com_max[i] = com_tmp;
						}
					}

					//if (CALI_END == 1)
					//{
					//	Calibrated = 1;
					//	com_hord_factor_X = (com_min[0] + com_max[0]) / 2;
					//	com_hord_factor_Y = (com_min[1] + com_max[1]) / 2;
					//	com_hord_factor_Z = (com_min[2] + com_max[2]) / 2;
					//	printf("X : %.3lf, Y : %.3lf, Z : %.3lf\n", com_hord_factor_X, com_hord_factor_Y, com_hord_factor_Z);
					//}

					continue;


				}

				if (KEY_DOWN('6') && SIX_AXIS_ENABLE == 0)
				{
					SIX_AXIS_ENABLE = 1;
					NINE_AXIS_ENABLE = 0;
					std::cout << "COMPASS DISABLED !"<< std::endl;

				}
				else if (KEY_DOWN('9') && NINE_AXIS_ENABLE == 0)
				{
					SIX_AXIS_ENABLE = 0;
					NINE_AXIS_ENABLE = 1;
					std::cout << "COMPASS ENABLED !" << std::endl;

				}



				gyro1[star1 - 1][0] = MovAvgFilter(IMU->gyrData[0], 10, 0);
				gyro1[star1 - 1][1] = MovAvgFilter(IMU->gyrData[1], 10, 1);
				gyro1[star1 - 1][2] = MovAvgFilter(IMU->gyrData[2], 10, 2);
				acc1[star1 - 1][0] = IMU->accData[0];
				acc1[star1 - 1][1] = IMU->accData[1];
				acc1[star1 - 1][2] = IMU->accData[2];
				com1[star1 - 1][0] = MovAvgFilter(IMU->magData[0], 10, 6);
				com1[star1 - 1][1] = MovAvgFilter(IMU->magData[1], 10, 7);
				com1[star1 - 1][2] = MovAvgFilter(IMU->magData[2], 10, 8);

				memset(buffer, 0, sizeof(buffer));

				stepDetection();
				if (x_tmp2 != px_pre || y_tmp2 != py_pre)
				{
					steps = steps++;
					y_tmp2 = py_pre;
					x_tmp2 = px_pre;
					printf("%.5f,%.5f\n", px_pre, py_pre);

					fprintf(fp, "%.5f,%.5f\n", px_pre, py_pre);


				}



#ifdef OPENCV




				if (x_tmp != px_pre)
				{
					Point a(int(py_pre * 4), int(px_pre * 4));
					Point a_2(int(y_tmp * 4), int(x_tmp * 4));
					line(graph, a + org, a_2 + org, Scalar(0, 0, 0), 1.5);
					y_tmp = py_pre;
					x_tmp = px_pre;

					//imshow("graph", graph);
					//waitKey(2);
				}


#endif
				//printf("%.5f,%.5f\n", px_pre, py_pre);
				//fprintf(fp, "%.5f,%.5f\n", px_pre, py_pre);

			}
			else
			{
				lastByte = oneByte[0];
				if (tCount<25 && oneByte[0] != 9 && tCount%2 == 0)
				{
					buffer[tCount/2] = char_to_hex(lastByte) | char_to_hex(oneByte[0]);
				}
				if(oneByte[0] != 9)tCount++;

				
			}
		}
	}
	catch (const char *msg)
	{
		std::cout << msg << std::endl;
	}
	return 0;


}
char char_to_hex(char aaa)
{
	if (aaa >= 48 && aaa <= 57)
		return aaa - 48;
	else if (aaa >= 65 && aaa <= 70)
		return aaa - 65 + 10;
}




// Fuzzy algorithm of Moving Filter
float fuzzify_MF(float x, float a, float b, float c, float d) //x=crisp input
{
	float dom;
	if (x >a && x <b)
	{
		dom = (x - a) / (b - a);
	}
	else if (x>c && x<d)
	{
		dom = (d - x) / (d - c);
	}
	else if (x >= b && x <= c)
	{
		dom = 1.0;
	}
	else
	{
		dom = 0;
	}
	return dom;
}

// Fuzzy algorithm
float Fuzzy(float input)
{
	float dom[10] = { 0 };
	float mf_in[10][4];
	mf_in[0][0] = 10; 	mf_in[0][1] = 10; 	mf_in[0][2] = 10; 	mf_in[0][3] = 11;
	mf_in[1][0] = 11;	mf_in[1][1] = 12;	mf_in[1][2] = 12;	mf_in[1][3] = 13;
	mf_in[2][0] = 13;	mf_in[2][1] = 14;	mf_in[2][2] = 14;	mf_in[2][3] = 15;
	mf_in[3][0] = 15;	mf_in[3][1] = 16;	mf_in[3][2] = 16;	mf_in[3][3] = 17;
	mf_in[4][0] = 17;	mf_in[4][1] = 19;	mf_in[4][2] = 19;	mf_in[4][3] = 21;
	mf_in[5][0] = 21;	mf_in[5][1] = 23;	mf_in[5][2] = 23;	mf_in[5][3] = 25;
	mf_in[6][0] = 25;	mf_in[6][1] = 26;	mf_in[6][2] = 26;	mf_in[6][3] = 27;
	mf_in[7][0] = 27;	mf_in[7][1] = 28;	mf_in[7][2] = 28;	mf_in[7][3] = 29;
	mf_in[8][0] = 29;	mf_in[8][1] = 30;	mf_in[8][2] = 30;	mf_in[8][3] = 31;
	mf_in[9][0] = 31;	mf_in[9][1] = 36;	mf_in[9][2] = 36;	mf_in[9][3] = 36;

	float mf_out[10][4];
	mf_out[0][0] = 0.35;	mf_out[0][1] = 0.35;	mf_out[0][2] = 0.35;	mf_out[0][3] = 0.36;
	mf_out[1][0] = 0.36;	mf_out[1][1] = 0.38;	mf_out[1][2] = 0.38;	mf_out[1][3] = 0.40;
	mf_out[2][0] = 0.38;	mf_out[2][1] = 0.42;	mf_out[2][2] = 0.42;	mf_out[2][3] = 0.46;
	mf_out[3][0] = 0.42;	mf_out[3][1] = 0.46;	mf_out[3][2] = 0.46;	mf_out[3][3] = 0.50;
	mf_out[4][0] = 0.42;	mf_out[4][1] = 0.46;	mf_out[4][2] = 0.46;	mf_out[4][3] = 0.50;
	mf_out[5][0] = 0.42;	mf_out[5][1] = 0.46;	mf_out[5][2] = 0.46;	mf_out[5][3] = 0.50;
	mf_out[6][0] = 0.42;	mf_out[6][1] = 0.46;	mf_out[6][2] = 0.46;	mf_out[6][3] = 0.50;
	mf_out[7][0] = 0.42;	mf_out[7][1] = 0.46;	mf_out[7][2] = 0.46;	mf_out[7][3] = 0.50;
	mf_out[8][0] = 0.46;	mf_out[8][1] = 0.50;	mf_out[8][2] = 0.50;	mf_out[8][3] = 0.54;
	mf_out[9][0] = 0.54;	mf_out[9][1] = 0.56;	mf_out[9][2] = 0.56;	mf_out[9][3] = 0.56;

	float mf[10][4];
	mf[0][0] = mf_out[0][0];	mf[0][3] = mf_out[0][3];	mf[1][0] = mf_out[1][0];	mf[1][3] = mf_out[1][3];
	mf[2][0] = mf_out[2][0];	mf[2][3] = mf_out[2][3];	mf[3][0] = mf_out[3][0];	mf[3][3] = mf_out[3][3];
	mf[4][0] = mf_out[4][0];	mf[4][3] = mf_out[4][3];	mf[5][0] = mf_out[5][0];	mf[5][3] = mf_out[5][3];
	mf[6][0] = mf_out[6][0];	mf[6][3] = mf_out[6][3];	mf[7][0] = mf_out[7][0];	mf[7][3] = mf_out[7][3];
	mf[8][0] = mf_out[8][0];	mf[8][3] = mf_out[8][3];	mf[9][0] = mf_out[9][0];	mf[9][3] = mf_out[9][3];

	for (int i = 0; i<10; i++)
	{
		dom[i] = fuzzify_MF(input, mf_in[i][0], mf_in[i][1], mf_in[i][2], mf_in[i][3]);
	}
	for (int i = 0; i<10; i++)
	{
		if ((dom[i]) != 0)
		{
			if ((dom[i]) == 1)
			{
				mf[i][1] = mf_out[i][1];
				mf[i][2] = mf_out[i][2];
			}
			else
			{
				mf[i][1] = dom[i] * (mf_out[i][1] - mf_out[i][0]) + mf_out[i][0];
				mf[i][2] = dom[i] * (mf_out[i][2] - mf_out[i][3]) + mf_out[i][3];
			}
		}
		else
		{
			mf[i][1] = mf_out[i][0];
			mf[i][2] = mf_out[i][3];
		}
	}
	double x_sum = 0;
	long int x_cnt = 0;
	double y[10] = { 0 };
	double y_max = 0;
	for (double x = 0; x<mf_out[9][3]; x += 0.001)
	{
		if ((dom[0]) != 0)
		{
			if ((x - mf[0][1]) >= 0 && (x - mf[0][2] <0))
			{
				y[0] = dom[0];
			}
			else if ((x - mf[0][2]) >= 0 && (x - mf[0][3] <0))
			{
				y[0] = dom[0] / (mf[0][3] - mf[0][2])*(mf[0][3] - x);
			}
			else y[0] = 0;
		}
		if ((dom[1]) != 0)
		{
			if ((x - mf[1][0]) >= 0 && (x - mf[1][1] <0))
			{
				y[1] = dom[1] / (mf[1][1] - mf[1][0])*(x - mf[1][0]);
			}
			else if ((x - mf[1][1]) >= 0 && (x - mf[1][2] <= 0))
			{
				y[1] = dom[1];
			}
			else if ((x - mf[1][2]) > 0 && (x - mf[1][3] <0))
			{
				y[1] = dom[1] / (mf[1][3] - mf[1][2])*(mf[1][3] - x);
			}
			else y[1] = 0;
		}
		if ((dom[2]) != 0)
		{
			if ((x - mf[2][0]) >= 0 && (x - mf[2][1] <0))
			{
				y[2] = dom[2] / (mf[2][1] - mf[2][0])*(x - mf[2][0]);
			}
			else if ((x - mf[2][1]) >= 0 && (x - mf[2][2] <= 0))
			{
				y[2] = dom[2];
			}
			else if ((x - mf[2][2]) > 0 && (x - mf[2][3] <0))
			{
				y[2] = dom[2] / (mf[2][3] - mf[2][2])*(mf[2][3] - x);
			}
			else y[2] = 0;
		}
		if ((dom[3]) != 0)
		{
			if ((x - mf[3][0]) >= 0 && (x - mf[3][1] <0))
			{
				y[3] = dom[3] / (mf[3][1] - mf[3][0])*(x - mf[3][0]);
			}
			else if ((x - mf[3][1]) >= 0 && (x - mf[3][2] <= 0))
			{
				y[3] = dom[3];
			}
			else if ((x - mf[3][2]) > 0 && (x - mf[3][3] <0))
			{
				y[3] = dom[3] / (mf[3][3] - mf[3][2])*(mf[3][3] - x);
			}
			else y[3] = 0;
		}
		if ((dom[3]) != 0)
		{

			if ((x - mf[3][0]) >= 0 && (x - mf[3][1] <0))
			{
				y[3] = dom[3] / (mf[3][1] - mf[3][0])*(x - mf[3][0]);
			}
			else if ((x - mf[3][1]) >= 0 && (x - mf[3][2] <0))
			{
				y[3] = dom[3];
			}
			else y[3] = 0;
		}
		if ((dom[4]) != 0)
		{
			if ((x - mf[4][0]) >= 0 && (x - mf[4][1] <0))
			{
				y[4] = dom[4] / (mf[4][1] - mf[4][0])*(x - mf[4][0]);
			}
			else if ((x - mf[4][1]) >= 0 && (x - mf[4][2] <= 0))
			{
				y[4] = dom[4];
			}
			else if ((x - mf[4][2]) > 0 && (x - mf[4][3] <0))
			{
				y[4] = dom[4] / (mf[4][3] - mf[4][2])*(mf[4][3] - x);
			}
			else y[4] = 0;
		}
		if ((dom[5]) != 0)
		{
			if ((x - mf[5][0]) >= 0 && (x - mf[5][1] <0))
			{
				y[5] = dom[5] / (mf[5][1] - mf[5][0])*(x - mf[5][0]);
			}
			else if ((x - mf[5][1]) >= 0 && (x - mf[5][2] <= 0))
			{
				y[5] = dom[5];
			}
			else if ((x - mf[5][2]) > 0 && (x - mf[5][3] <0))
			{
				y[5] = dom[5] / (mf[5][3] - mf[5][2])*(mf[5][3] - x);
			}
			else y[5] = 0;
		}
		if ((dom[6]) != 0)
		{
			if ((x - mf[6][0]) >= 0 && (x - mf[6][1] <0))
			{
				y[6] = dom[6] / (mf[6][1] - mf[6][0])*(x - mf[6][0]);
			}
			else if ((x - mf[6][1]) >= 0 && (x - mf[6][2] <= 0))
			{
				y[6] = dom[6];
			}
			else y[6] = 0;
		}
		if ((dom[7]) != 0)
		{
			if ((x - mf[7][0]) >= 0 && (x - mf[7][1] <0))
			{
				y[7] = dom[7] / (mf[7][1] - mf[7][0])*(x - mf[7][0]);
			}
			else if ((x - mf[7][1]) >= 0 && (x - mf[7][2] <= 0))
			{
				y[7] = dom[7];
			}
			else y[7] = 0;
		}
		if ((dom[8]) != 0)
		{
			if ((x - mf[8][0]) >= 0 && (x - mf[8][1] <0))
			{
				y[8] = dom[8] / (mf[8][1] - mf[8][0])*(x - mf[8][0]);
			}
			else if ((x - mf[8][1]) >= 0 && (x - mf[8][2] <= 0))
			{
				y[8] = dom[8];
			}
			else y[8] = 0;
		}
		if ((dom[9]) != 0)
		{
			if ((x - mf[9][0]) >= 0 && (x - mf[9][1] <0))
			{
				y[9] = dom[9] / (mf[9][1] - mf[9][0])*(x - mf[9][0]);
			}
			else if ((x - mf[9][1]) >= 0 && (x - mf[9][2] <= 0))
			{
				y[9] = dom[9];
			}
			else y[9] = 0;
		}
		for (int i = 0; i<10; i++)
		{
			if (y_max <= y[i])
				y_max = y[i];
		}
		for (double y_i = 0; y_i <= y_max; y_i += 0.001)
		{
			x_sum += x;
			x_cnt++;
		}
		y_max = 0;
	}
	x_sum /= x_cnt;
	return x_sum;
}

// move average filter data:data; 
// nn:the filter length; 
// num: the IMU data serial number, gyro is 0-2, acc is 3-5, com is 6-8
double MovAvgFilter(double data, char nn, char num)
{
	acc_3D_num[num]++;
	if (IMU_buffer_ready[num] == 0)
	{
		acc_3D_filter_buffer[num][acc_3D_num[num] - 1] = data;
		acc_3D_filter_sum[num] = acc_3D_filter_sum[num] + data;
	}
	else if (IMU_buffer_ready[num] == 1)
	{
		acc_3D_filter_sum[num] = acc_3D_filter_sum[num] + data - acc_3D_filter_buffer[num][acc_3D_num[num] - 1];
		acc_3D_filter_buffer[num][acc_3D_num[num] - 1] = data;
	}
	if (acc_3D_num[num] == nn)
	{
		IMU_buffer_ready[num] = 1;
	}
	acc_3D_num[num] = acc_3D_num[num] % nn;
	if (IMU_buffer_ready[num] == 1)
	{
		return acc_3D_filter_sum[num] / nn;
	}
	else
		return data;
}

// steps detecting process
void StepAnalPDRSystem(double ang)
{
	if (isnan(ang))
	{
		ang = 0;
	}
	heading_temp = ang*R2D;
	if (heading_temp< 0)
	{
		heading_temp = heading_temp + ((int)(abs(heading_temp) / 360 + 1)) * 360;
	}
	else if (heading_temp >= 360)
	{
		heading_temp = heading_temp - ((int)(abs(heading_temp) / 360)) * 360;;
	}
	if (acc_3D_filter[2] < min_thread && min_flag == 0)
	{
		min_flag = 1;
	}
	else if (acc_3D_filter[2] > max_thread  && min_flag == 1)
	{
		if ((abs(acc1[star1 - 1][0]) > 1.2 && crawling == 0) || crawling == 1)
		{
			min_detect = pre_min;
			if (max_flag == 0)
			{
				max_min_detect = min_detect;
			}
			max_flag = 1;
		}
	}
	else if (max_flag == 1 && acc_3D_filter[2] < GRAVITY)
	{
		max_detect = pre_max;
		pre_min = min_thread;
		pre_max = max_thread;
		max_min_detect = max_detect;
		zero_flag = 1;
		min_flag = 0;
		max_flag = 0;
	}
	if (acc_3D_filter[2] < min_thread) // % min peak
	{
		if (acc_3D_filter[2] < pre_min)
		{
			pre_min = acc_3D_filter[2];
		}
		else if (acc_3D_filter[2] > pre_min)
		{
			pre_min = pre_min;
		}
	}
	else if (acc_3D_filter[2] > max_thread) // % max peak
	{
		if (acc_3D_filter[2] > pre_max)
		{
			pre_max = acc_3D_filter[2];
		}
		else if (acc_3D_filter[2] < pre_max)
		{
			pre_max = pre_max;
		}
	}
	if (zero_flag == 1)
	{
		zero_flag = 0;

		diffmaxmin = max_detect - min_detect;

		if (1)
		{
			K = 0;
			K = Fuzzy(max_detect);
		}
		if (crawling == 0)
		{
			step_length = pow(diffmaxmin, 0.25)*K;
		}
		else
		{
			step_length = pow(diffmaxmin, 0.25)*crawl_K;
		}
		//steps = (steps + 1) % 5;
	}
	else
	{
		step_length = 0;
	}

	px_pre = px_pre + step_length*cos(ang);
	py_pre = py_pre - step_length*sin(ang);
}


// SIGN function for direction  detecting
int SIGN(double sign)
{
	if (sign>0) return 1;
	else if (sign < 0) return -1;
	else return 0;
}

// matrix addition function 
// in put A,B matrix
// mm: rows
// nn: columns
// C : result buffer
void MatrixAddition(double* A, double* B, int mm, int nn, double* C)
{
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A+B (m x n)
	int i, j;
	for (i = 0; i<mm; i++)
		for (j = 0; j<nn; j++)
			C[nn*i + j] = A[nn*i + j] + B[nn*i + j];
}

// matrix transpose function 
// in put A matrix
// mm: rows
// nn: columns
// C : result buffer
void MatrixTranspose(double* A, int mm, int nn, double* C)
{
	// A = input matrix (m x n)
	// m = number of rows in A
	// n = number of columns in A
	// C = output matrix = the transpose of A (n x m)
	int i, j;
	for (i = 0; i<mm; i++)
		for (j = 0; j<nn; j++)
			C[mm*j + i] = A[nn*i + j];
}

// matrix subtraction function 
// in put A,B matrix
// mm: rows
// nn: columns
// C : result buffer
void MatrixSubtraction(double* A, double* B, int mm, int nn, double* C)
{
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A-B (m x n)
	int i, j;
	for (i = 0; i<mm; i++)
		for (j = 0; j<nn; j++)
			C[nn*i + j] = A[nn*i + j] - B[nn*i + j];
}

// matrix multiply function 
// in put A,B matrix
// mm: rows
// pp: columns of A & rows of B
// nn: columns of B
// C : result buffer
void MatrixMultiply(double* A, double* B, int mm, int pp, int nn, double* C)
{
	// A = input matrix (m x p)
	// B = input matrix (p x n)
	// m = number of rows in A
	// p = number of columns in A = number of rows in B
	// n = number of columns in B
	// C = output matrix = A*B (m x n)
	int i, j, k;
	for (i = 0; i<mm; i++)
		for (j = 0; j<nn; j++)
		{
			C[nn*i + j] = 0;
			for (k = 0; k<pp; k++)
				C[nn*i + j] = C[nn*i + j] + A[pp*i + k] * B[nn*k + j];
		}
}

// matrix inversion function 
// in put A matrix
// mm: rows
// nn: columns 
// AInverse : result buffer
int MatrixInversion(double* A, int nn, double* AInverse)
{
	// A = input matrix (n x n)
	// n = dimension of A
	// AInverse = inverted matrix (n x n)
	// This function inverts a matrix based on the Gauss Jordan method.
	// The function returns 1 on success, 0 on failure.
	int i, j, iPass, imx, icol, irow;
	double det, temp, pivot, factor;
	//double ac[4]={0,0,0,0};
	//double* ac = (double*)calloc(n*n, sizeof(double));
	//double* ac;
	det = 1;
	factor = 0;
	for (i = 0; i < nn; i++)
	{
		for (j = 0; j < nn; j++)
		{
			AInverse[nn*i + j] = 0;  //ac[nn*i+j] = A[nn*i+j];
		}
		AInverse[nn*i + i] = 1;
	}
	// The current pivot row is iPass.
	// For each pass, first find the maximum element in the pivot column.
	for (iPass = 0; iPass < nn; iPass++)
	{
		imx = iPass;
		for (irow = iPass; irow < nn; irow++)
		{
			if (abs(A[nn*irow + iPass]) > abs(A[nn*imx + iPass])) imx = irow;
		}
		// Interchange the elements of row iPass and row imx in both A and AInverse.
		if (imx != iPass)
		{
			for (icol = 0; icol < nn; icol++)
			{
				temp = AInverse[nn*iPass + icol];
				AInverse[nn*iPass + icol] = AInverse[nn*imx + icol];
				AInverse[nn*imx + icol] = temp;
				if (icol >= iPass)
				{
					temp = A[nn*iPass + icol];
					A[nn*iPass + icol] = A[nn*imx + icol];
					A[nn*imx + icol] = temp;
				}
			}
		}
		// The current pivot is now A[iPass][iPass].
		// The determinant is the product of the pivot elements.
		pivot = A[nn*iPass + iPass];
		det = det * pivot;
		if (det == 0)
		{
			//    free(ac);
			return 0;
		}
		for (icol = 0; icol < nn; icol++)
		{
			// Normalize the pivot row by dividing by the pivot element.
			AInverse[nn*iPass + icol] = AInverse[nn*iPass + icol] / pivot;
			if (icol >= iPass) A[nn*iPass + icol] = A[nn*iPass + icol] / pivot;
		}
		for (irow = 0; irow < nn; irow++)
			// Add a multiple of the pivot row to each row.  The multiple factor
			// is chosen so that the element of A on the pivot column is 0.
		{
			if (irow != iPass) factor = A[nn*irow + iPass];
			for (icol = 0; icol < nn; icol++)
			{
				if (irow != iPass)
				{
					AInverse[nn*irow + icol] -= factor * AInverse[nn*iPass + icol];
					A[nn*irow + icol] -= factor * A[nn*iPass + icol];
				}
			}
		}
	}
	return 0;
}

// Moving direction detecting(heading direction)
double EulerEKF_PSI(double z, double rates)
{
	static char flag2 = 0;
	static double x_2 = 0;
	if (flag2 == 0)
	{
		x_2 = rates;
		flag2 = 1;
	}
	static double H_2 = 1;
	static double Q_2 = 0.09;
	static double R_2 = 0.0034;

	static double P_2 = 100;
	double A = x_2;
	A = 1 + A*dt;
	double xp;
	double r = rates;
	double xdot = r;
	xp = x_2 * xdot*dt;
	double Pp = A*P_2*A + Q_2;
	double K = Pp*H_2 / (H_2*Pp*H_2 + R_2);
	x_2 = xp + K*(z - H_2*xp);
	P_2 = Pp - K*H_2*Pp;
	return x_2;
}
double EulerEKF_PSIG(double z, double rates)
{
	static char flag2 = 0;
	static double x_2 = 0;
	if (flag2 == 0)
	{
		x_2 = rates;
		flag2 = 1;
	}
	static double H_2 = 1;
	static double Q_2 = 0.5;
	static double R_2 = 0.01;

	static double P_2 = 100;
	double A = x_2;
	A = 1 + A*dt;
	double xp;
	double r = rates;
	double xdot = r;
	xp = x_2 * xdot*dt;
	double Pp = A*P_2*A + Q_2;
	double K = Pp*H_2 / (H_2*Pp*H_2 + R_2);
	x_2 = xp + K*(z - H_2*xp);
	P_2 = Pp - K*H_2*Pp;
	return x_2;
}

//function of kalman filter
void EulerEKFG_NEW(double z[], char m1, char n1, double rates[], char m2, char n2, double init, double *angle)
{
	static char flag = 0;

	static double H[] = { 1, 0, 0, 0, 1, 0 };//2row*3column
	static double Q[] = { 0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05 };//3*3
	static double R[] = { 0.01, 0, 0, 0.01 };//2*2;
	static double P[] = { 10, 0, 0, 0, 10, 0, 0, 0, 10 };//
	static double x[3] = { 0 };
	if (flag == 0)
	{
		x[0] = rates[0];
		x[1] = rates[1];
		x[2] = init;
		flag = 1;
	}
	// fx
	double phi = x[0];
	double theta = x[1];
	//  double p, q, r;
	//  p = rates[0];
	//  q = rates[1];
	//  r = rates[2];
	double xdot[3] = { 0 };
	xdot[0] = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
	xdot[1] = q*cos(phi) - r*sin(phi);
	xdot[2] = q*sin(phi) / cos(theta) + r*cos(phi) / cos(theta);
	double xp[3];
	xp[0] = x[0] + xdot[0] * dt;
	xp[1] = x[1] + xdot[1] * dt;
	xp[2] = x[2] + xdot[2] * dt;

	//ajo
	double A[9] = { 0 };//3*3
	A[0] = q*cos(phi)*tan(theta) - r*sin(phi)*tan(theta);
	A[1] = q*sin(phi) / pow(cos(theta), 2) + r*cos(phi) / pow(cos(theta), 2);
	A[2] = 0;
	A[3] = -q*sin(phi) - r*cos(phi);
	A[4] = 0;
	A[5] = 0;
	A[6] = q*cos(phi) / cos(theta) - r*sin(phi) / cos(theta);
	A[7] = q*sin(phi) / cos(theta)*tan(theta) + r*cos(phi) / cos(theta)*tan(theta);
	A[8] = 0;
	for (char i = 0; i<9; i++)
	{
		A[i] = A[i] * dt + ((i == 0 || i == 4 || i == 8) ? 1 : 0);
	}

	double tmp[9];
	MatrixTranspose(A, 3, 3, tmp);
	double Pp_tmp[9];
	MatrixMultiply(A, P, 3, 3, 3, Pp_tmp);
	double Pp_tmp2[9];
	MatrixMultiply(Pp_tmp, tmp, 3, 3, 3, Pp_tmp2);
	double Pp[9];
	MatrixAddition(Pp_tmp2, Q, 3, 3, Pp);

	double H2[6];
	MatrixTranspose(H, 2, 3, H2);
	double K_tmp[6];
	MatrixMultiply(Pp, H2, 3, 3, 2, K_tmp);//3*2

	double K_tmp2[6];
	MatrixMultiply(H, Pp, 2, 3, 3, K_tmp2);
	double K_tmp3[4];
	MatrixMultiply(K_tmp2, H2, 2, 3, 2, K_tmp3);
	double K_tmp4[4];
	MatrixAddition(K_tmp3, R, 2, 2, K_tmp4);//tmp3 = H*Pp*H' + R
	double K_tmp5[4];
	MatrixInversion(K_tmp4, 2, K_tmp5);
	double K[6];
	MatrixMultiply(K_tmp, K_tmp5, 3, 2, 2, K);//K  value3*2

											  //if(abs(g3d) > 5)
	{
		double x_tmp[2];
		MatrixMultiply(H, xp, 2, 3, 1, x_tmp);//2*1
		double x_tmp2[2];
		MatrixSubtraction(z, x_tmp, 2, 1, x_tmp2);
		double x_tmp3[3];
		MatrixMultiply(K, x_tmp2, 3, 2, 1, x_tmp3);
		MatrixAddition(xp, x_tmp3, 3, 1, x);//3*1
	}
	double tmp6[9];
	MatrixMultiply(K, H, 3, 2, 3, tmp6);
	double tmp7[9];
	MatrixMultiply(tmp6, Pp, 3, 3, 3, tmp7);
	MatrixSubtraction(Pp, tmp7, 3, 3, P);

	double psi = x[2];
	int psi_tmp = psi / 3.1415926;
	psi = psi - psi_tmp * 3.1415926;
	angle[0] = x[0];
	angle[1] = x[1];
	angle[2] = x[2];
}

// Steps detection (include direction and distance)
void stepDetection()
{
	if (Calibrated == 1 && step_detection_begin == 1)
	{
		double AccZ_X = 0 - acc1[star1 - 1][2] * GRAVITY;       //Z
		double AccX_Y = acc1[star1 - 1][1] * GRAVITY;       //Y
		double AccY_Z = 0 - acc1[star1 - 1][0] * GRAVITY;           //X
																	//    
																	//    double AccZ_X = 0 - acc1[star1-1][1]*GRAVITY;       //-Y
																	//    double AccX_Y = acc1[star1-1][2]*GRAVITY;           //Z
																	//    double AccY_Z = acc1[star1-1][0]*GRAVITY;           //X

		if (abs(AccY_Z) >= abs(AccX_Y) && abs(AccY_Z)>abs(AccZ_X))
		{
			crawling = 0;
			double GyroZ_X = 0 - gyro1[star1 - 1][2] - GyroZ_e0;
			double GyroY_Y = gyro1[star1 - 1][1] - GyroY_e0;
			double GyroX_Z = 0 - gyro1[star1 - 1][0] - GyroX_e0;
			double comZ_X = com1[star1 - 1][2] - (com_hord_factor_Z);     // com_hord_factor_Z
			double comX_Y = com1[star1 - 1][0] - (com_hord_factor_X); // com_hord_factor_X
			double comY_Z = 0 - com1[star1 - 1][1] + (com_hord_factor_Y);     // com_hord_factor_Y  

			double gyro3d = sqrt(GyroZ_X*GyroZ_X + GyroY_Y*GyroY_Y + GyroX_Z*GyroX_Z);


			if (HDR_ENABLE == 1)
			{
				double HDR_GyroZ_X = GyroZ_X + HDR_I_0;
				double HDR_GyroY_Y = GyroY_Y + HDR_I_1;
				double HDR_GyroX_Z = GyroX_Z + HDR_I_2;

				if ((0 - threshold_H)<HDR_GyroZ_X && HDR_GyroZ_X<threshold_H
					&& (0 - threshold_H)<HDR_GyroY_Y && HDR_GyroY_Y<threshold_H
					&& (0 - threshold_H)<HDR_GyroX_Z && HDR_GyroX_Z<threshold_H)
				{
					HDR_I_0 = HDR_I_0 - SIGN(HDR_GyroZ_X) *ic;
					HDR_I_1 = HDR_I_1 - SIGN(HDR_GyroY_Y) *ic;
					HDR_I_2 = HDR_I_2 - SIGN(HDR_GyroX_Z) *ic;
				}

				p = HDR_GyroZ_X*D2R;
				q = HDR_GyroY_Y*D2R;
				r = HDR_GyroX_Z*D2R;
			}
			else if (HDR_DISABLE == 1)
			{
				p = GyroZ_X*D2R;
				q = GyroY_Y*D2R;
				r = GyroX_Z*D2R;
			}

			double psi_c = atan2(comY_Z, comZ_X);
			double ini = 0;
			//double ini = psi_c;

			//EulerAccel
			double tmp_sin = AccZ_X / 9.81*D2R;
			double theta_a;
			theta_a = asin(tmp_sin);
			double phi_a;
			phi_a = asin((0 - (AccX_Y*D2R)) / (9.81*cos(theta_a)));
			double para1[2] = { 0 };
			para1[0] = phi_a;
			para1[1] = theta_a;

			if (isnan(para1[0]) || isnan(para1[1]))
			{
				para1[0] = phi_buf[0];
				para1[1] = phi_buf[1];
			}
			else
			{
				phi_buf[0] = para1[0];
				phi_buf[1] = para1[1];
			}

			double para2[3];
			para2[0] = p;
			para2[1] = q;
			para2[2] = r;

			double angle[3] = { 0 };
			EulerEKFG_NEW(para1, 2, 1, para2, 1, 3, ini, angle);

			double phi = angle[0];
			double theta = angle[1];
			double psi = angle[2];

			if (NINE_AXIS_ENABLE)
			{
				psi = EulerEKF_PSI(psi_c, psi);
				if (KEY_DOWN('1'))
				{
					SIX_AXIS_ENABLE = 0;
					NINE_AXIS_ENABLE = 1;
					printf("Compass Enable\n");
				}
			}
			else if (SIX_AXIS_ENABLE)
			{
				psi = EulerEKF_PSIG(psi, 0);
				if (KEY_DOWN('2'))
				{
					SIX_AXIS_ENABLE = 1;
					NINE_AXIS_ENABLE = 0;
					printf("Compass Disable\n");
				}

			}


			double phi_a1 = phi*R2D;
			double theta_a1 = theta*R2D;
			double psi_gy_d = psi * R2D;

			psi_gy_d = psi * R2D;
			if (psi_gy_d < 0)
			{
				psi_gy_d = psi_gy_d + ((int)(abs(psi_gy_d) / 360 + 1)) * 360;
			}
			else if (psi_gy_d >= 360)
			{
				psi_gy_d = psi_gy_d - ((int)(abs(psi_gy_d) / 360)) * 360;;
			}
			psi_gy_r = psi_gy_d*D2R;
			heading_psi = psi_gy_r;



			if (direciton_begin<16)
			{
				direciton_begin = (direciton_begin + 1) % 300;
				if (direciton_begin == 15)
				{
					psi_init = heading_psi;
				}
			}
			//directiion modify
			if (steps < 5)
			{
				psi_init = psi_gy_r;
			}
		}
		else if (abs(AccZ_X) >= abs(AccX_Y) && abs(AccZ_X) >= abs(AccY_Z))
		{
			crawling = 1;
		}

		//walk distance
		double tmp_1 = (acc1[star1 - 1][0] * acc1[star1 - 1][0])
			+ (acc1[star1 - 1][1] * acc1[star1 - 1][1]) +
			+(acc1[star1 - 1][2] * acc1[star1 - 1][2]);
		double tmp_2 = pow(tmp_1, 0.5) * GRAVITY;
		double tmp_3 = MovAvgFilter(tmp_2, 1, 5);


		if (IMU_buffer_ready[5] == 1)
		{
			acc_3D_filter[2] = tmp_3;
			if (acc_3D_filter[0] != 0 && acc_3D_filter[1] != 0 && acc_3D_filter[2] != 0)
			{
				StepAnalPDRSystem(heading_PDR*D2R - psi_gy_r + psi_init);
			}
			acc_3D_filter[0] = acc_3D_filter[1];
			acc_3D_filter[1] = acc_3D_filter[2];
		}
	}

}

