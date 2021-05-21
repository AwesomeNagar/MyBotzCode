/******************************************************************************/
/*                                                                            */
/* main.c --                                       */
/*                                                                            */
/******************************************************************************/
/* Author: Suhas Nagar                                                       */
/* Copyright 2017, Digilent Inc.                                              */
/******************************************************************************/
/* Module Description:                                                        */
/*                                                                            */
/* This file contains code for running a demonstration of the PmodAD1 when    */
/* used with the PmodAD1 IP core. This demo initializes the PmodAD1 IP core   */
/* and then polls its sample register, printing the analog voltage last       */
/* sampled by each of the AD1's two channels over UART.                       */
/*                                                                            */
/* Messages printed by this demo can be received by using a serial terminal   */
/* configured with the appropriate Baud rate. 115200 for Zynq systems, and    */
/* whatever the AXI UARTLITE IP is configured with for MicroBlaze systems,    */
/* typically 9600 or 115200 Baud.                                             */
/*                                                                            */
/******************************************************************************/
/* Revision History:                                                          */
/*                                                                            */
/*    08/15/2017(ArtVVB):   Created                                           */
/*    02/10/2018(atangzwj): Validated for Vivado 2017.4                       */
/*    07/12/2019(MaximV):   Modefied to work for avbotz                       */
/*                                                                            */
/******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include "PmodAD1.h"
#include "sleep.h"
#include "xil_cache.h"
#include "xil_io.h"
#include "xil_types.h"
#include "xparameters.h"
#include "math.h"
#include "stdlib.h"
//705khz sampale rate aprox
//Size of sampling array
#define SAMPLERATE 286000
//Velocity of sound in water to approximate distances
#define VINWATER 1500
//Distance between the square configuration of phones
#define PHONEDIST 0.0127
//memory offset to prevent crashes
#define MEMOFSET  0x03ffffff;
int sampsize = 512;

//DeviceDeclarations
PmodAD1 myDevice0;
PmodAD1 myDevice1;
const float ReferenceVoltage = 3.3;
const int Sampales = 1000000;

//Frequency array to store different possible frequencies
int freq[1] = {25000
	//,30000
	//,35000
	//,40000
	};

void Initialize();
void DemoRun();
void DemoCleanup();
void EnableCaches();
void DisableCaches();
void DataCollect();
void DataReturnRaw();
void DataReturnFloat();


void Initialize() {
	sampsize = (round)(SAMPLERATE/freq[0]*50);
   EnableCaches();
   AD1_begin(&myDevice0, XPAR_PMODAD1_0_AXI_LITE_SAMPLE_BASEADDR);
   AD1_begin(&myDevice1, XPAR_PMODAD1_1_AXI_LITE_SAMPLE_BASEADDR);

   // Wait for AD1 to finish powering on
   usleep(1); // 1 us (minimum)
}


void DataCollect(){
	//printf("Start\r\n");
	u32  WriteAdress;
	u32 temp; //Used as temporary variable for memory manipulation
	WriteAdress = XPAR_MIG_7SERIES_0_BASEADDR+MEMOFSET;//Initialize Base addresses of Ram
	int n = WriteAdress;
	printf ("%X Start Address.\r\n", n); //Debug Synch Start end addresses
	for(u32 alpha = 0; alpha< Sampales; alpha++){
		temp = Xil_In32(XPAR_PMODAD1_0_AXI_LITE_SAMPLE_BASEADDR);//Reading First Pmod
		Xil_Out32(WriteAdress,temp);//Saving data to ram
		//printf("%08lX;",temp); //Debugging write and read
		temp = Xil_In32(XPAR_PMODAD1_1_AXI_LITE_SAMPLE_BASEADDR);//Reading Second Pmod
		Xil_Out32(WriteAdress+0x20,temp);//Saving Data to ram
		//printf("%08lX\n\r",temp); //Debugging write and read
		WriteAdress = WriteAdress+0x40;//Incrementing Write addresses by 64
		//usleep(1);
	}
	n = WriteAdress;
	printf ("%X EndAdress.\r\n", n);
	//printf("Done\r\n");
}
void DataReturnRaw(){
	printf("Beggining Data Return from ram\r\n");
	u32  WriteAdress;
	u32 temp; //Used as temporary variable for memory manipulation
	WriteAdress = XPAR_MIG_7SERIES_0_BASEADDR+MEMOFSET;//Initialize Base Adress of Ram
	//printf ("%lX Start Address.\r\n", n);//DEBUG
	int tempf;
	//float conversionFactor = ReferenceVoltage / ((1 << 12) - 1);
	for(u32 alpha = 0; alpha< Sampales; alpha++){
		temp = Xil_In32(WriteAdress);//Reading First Pmod
		tempf = ((int)(temp & 0xFFF));
		printf("%d;" ,tempf); //Returning Data via urt
		tempf = ((int)((temp >> 16) & 0xFFF));
		printf("%d;" ,tempf); //Returning Data via urt
		temp = Xil_In32(WriteAdress+0x20);//Reading Second Pmod
		tempf = ((int)(temp & 0xFFF));
		printf("%d;" ,tempf); //Returning Data via urt
		tempf = ((int)((temp >> 16) & 0xFFF));
		printf("%d\n" ,tempf); //Returning Data via urt

		WriteAdress = WriteAdress+0x40;//Incrementing Write Adress by 64
	}
	//printf ("%lX EndAdress.\r\n", WriteAdress);//DEBUG
}

void DemoRun() {
   AD1_RawData RawData;
   AD1_PhysicalData PhysicalData;

   while (1) {
      AD1_GetSample(&myDevice0, &RawData); // Capture raw samples

      // Convert raw samples into floats scaled to 0 - VDD
      AD1_RawToPhysical(ReferenceVoltage, RawData, &PhysicalData);

      printf("%.02f;", PhysicalData[0]);
      printf("%.02f;", PhysicalData[1]);

      AD1_GetSample(&myDevice1, &RawData); // Capture raw samples

      // Convert raw samples into floats scaled to 0 - VDD
      AD1_RawToPhysical(ReferenceVoltage, RawData, &PhysicalData);

      printf("%.02f;", PhysicalData[0]);
      printf("%.02f\r\n", PhysicalData[1]);


      // Do this 10x per second
      //usleep(1);
   }
}


void DemoCleanup() {
   DisableCaches();
}

void EnableCaches() {
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_ICACHE
   Xil_ICacheEnable();
#endif
#ifdef XPAR_MICROBLAZE_USE_DCACHE
   Xil_DCacheEnable();
#endif
#endif
}

void DisableCaches() {
#ifdef __MICROBLAZE__
#ifdef XPAR_MICROBLAZE_USE_DCACHE
   Xil_DCacheDisable();
#endif
#ifdef XPAR_MICROBLAZE_USE_ICACHE
   Xil_ICacheDisable();
#endif
#endif
}





float quadForm(float s, float b, float c){
	//Mathematically solve for the value of x, or the horizontal displacement from one hydrophone to the pinget
	float M = (pow(s,2)-b*c)*(b+c)/(2*c*s);
	float L = (pow(s,2)-pow(c,2))/(2*c);
	float AA = pow(c,2)+pow(b,2)-pow(s,2);
	float BB = 2*c*(M*b-L*s);
	float CC = (pow(c,2))*(pow(M,2)-pow(L,2));
	//Value of x determined by quadratic formula

   //NOTE THIS CAN BE CHANGED TO A -BB+... instead of -BB-...
	return (-BB-pow(pow(BB,2)-4*AA*CC,0.5))/(2*AA);
}
float quadForma(float s, float b, float c){
   //Mathematically solve for the value of x, or the horizontal displacement from one hydrophone to the pinget
   float M = (pow(s,2)-b*c)*(b+c)/(2*c*s);
   float L = (pow(s,2)-pow(c,2))/(2*c);
   float AA = pow(c,2)+pow(b,2)-pow(s,2);
   float BB = 2*c*(M*b-L*s);
   float CC = (pow(c,2))*(pow(M,2)-pow(L,2));
   //Value of x determined by quadratic formula

   //NOTE THIS CAN BE CHANGED TO A -BB+... instead of -BB-...
   return (-BB+pow(pow(BB,2)-4*AA*CC,0.5))/(2*AA);
}





float yfromx(float s, float b, float c, float x){
	//Given a value of x, find the value of y using math
	float M = (pow(s,2)-b*c)*(b+c)/(2*c*s);
	return (b*x/c+M-s);
}
float distance(float x1, float y1, float x2, float y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
float* DVLProcess(){
	u32  WriteAdress;
	u32 temp; //Used as temporary variable for memory manipulation
	WriteAdress = XPAR_MIG_7SERIES_0_BASEADDR+MEMOFSET;//Initialize Base Adress of Ram
	int tempf;
	int bin = round(1.0*freq[0]/SAMPLERATE*sampsize);
	float* maxamp = malloc(sizeof(float)*3);
	float* retang = malloc(sizeof(float)*3);
	memset(maxamp, 0, sizeof(float)*3);
	float omega = (2*M_PI/sampsize)*bin;
	float cosine = cos(omega);
	float sine = sin(omega);
	float coeff = 2*cosine;
	float q1[3];
	float q2[3];
	float q0[3];
	//float conversionFactor = ReferenceVoltage / ((1 << 12) - 1);
	for(int alpha = 0; alpha< Sampales; alpha++){
		if (alpha % sampsize == 0){
			int b = 1;
			for (int i = 0; i < 3; i++){
				if (q1[i]*q1[i]+q2[i]*q2[i]-q1[i]*q2[i]*coeff <= maxamp[i]){
				   b = 0;
				}
			}
			if (b==1){
				for (int i = 0; i < 3; i++){
					float real = q1[i]-q2[i]*cosine;
					float imag = q2[i]*sine;
					maxamp[i] = real*real+imag*imag;
					retang[i] = atan((imag)/(real));
				}
			}
			q1[0] = 0; q1[1] = 0;q1[2] = 0;
			q2[0] = 0; q2[1] = 0;q2[2] = 0;
			q0[0] = 0; q0[1] = 0;q0[2] = 0;
		}
		temp = Xil_In32(WriteAdress);//Reading First Pmod
		tempf = ((int)((temp & 0xFFF))); //First Hydrophone
		q0[0] = coeff*q1[0]-q2[0]+1.0*tempf/10;
		q2[0] = q1[0];
		q1[0] = q0[0];
		tempf = ((int)(((temp >> 16) & 0xFFF))); //Second Hydrophone
		q0[1] = coeff*q1[1]-q2[1]+1.0*tempf/10;
		q2[1] = q1[1];
		q1[1] = q0[1];
		temp = Xil_In32(WriteAdress+0x20);//Reading First Pmod
		tempf = ((int)((temp & 0xFFF)));  //Third Hydrophone
		q0[2] = coeff*q1[2]-q2[2]+1.0*tempf/10;
		q2[2] = q1[2];
		q1[2] = q0[2];
		tempf = ((int)(((temp >> 16) & 0xFFF))); //Fourth Hydrophone
		WriteAdress = WriteAdress+0x40;//Incrementing Write Adress by 64
	}
	free(maxamp);
	return (retang);

}


float retRealAngle(){
	 float* phase = DVLProcess();
	 printf("Phases: %f ", phase[0]);
	 printf("%f ", phase[1]);
	 printf("%f\n", phase[2]);

	 if (phase[1]-phase[0] > 1.24){
		 phase[0]+=M_PI;
	 }else if (phase[1]-phase[0] < -1.24){
		 phase[0] -= M_PI;
	 }
	 if (phase[2]-phase[1] > 1.24){
	 	phase[2]-=M_PI;
	 }else if (phase[2]-phase[1] < -1.24){
	 	phase[2] += M_PI;

	 }
	 printf("Phases: %f ", phase[0]);
	 printf("%f ", phase[1]);
	 printf("%f\n", phase[2]);
	 float b = phase[1]-phase[0];
	 float c = phase[2]-phase[1];
	 b/=(M_PI*freq[0]*2);
	 c/=(M_PI*freq[0]*2);
	 b *= VINWATER;
	 c *= VINWATER;
	 float x2 = quadForm(0.0127, b, c);
	 float y2 = yfromx(0.0127, b, c, x2);
	 float x3 = quadForma(0.0127, b, c);
	 float y3 = yfromx(0.0127, b, c, x3);
	 free(phase);
	 if (isnan(x2) || isnan(y2)){
		 if (isnan(x3) || isnan(y3)){
			 return 1000;
		 }else{
			 return atan(x3/y3);
		 }
	 }
	 if (isnan(x3) || isnan(y3)){
		 return atan(x2/y2);
	 }

	 float b1 = -distance(y2, x2, 0,0) + distance(y2, x2, 0.0127, 0);
	 float c1 = distance(y2, x2, 0.0127, 0) - distance(y2, x2, 0.0127, -0.0127);
	 float x21 = quadForm(0.0127,b1,c1);
	 float y21 = yfromx(0.0127,b1,c1,x21);
	 float b2 = -distance(y3, x3, 0,0) + distance(y3, x3, 0.0127, 0);
	 float c2 = distance(y3, x3, 0.0127, 0) - distance(y3, x3, 0.0127 , -0.0127);
	 float x31 = quadForma(0.0127,b2,c2);
	 float y31 = yfromx(0.0127,b2,c2,x31);
	 float a1 = atan(x2/y2);
	 float a2 = atan(x21/y21);
	 float a3 = atan(x3/y3);
	 float a4 = atan(x31/y31);
	 printf("Possible angles: %f , ",(a1*180/M_PI));
	 printf("%f\n",(a3*180/M_PI));
	 if (isnan(a4) && isnan(a2)){
		 return a1;
	 }
	 if (isnan(a4) || fabs((a1-a2)/a1) <= fabs((a3-a4)/a3)){
		 return(a1*180/M_PI);
	 }else{
		 return(a3*180/M_PI);
	 }
}
int main() {
   Initialize();
//   DataCollect();
   float totalang = 0;
   int samps = 0;
   printf("===================================================================================");
   while(1){
	   DataCollect(); //Collects x sampales as defined above and stores in ram
	   printf("We think the angle is: ");
	   float ang = retRealAngle();
	   printf("%0.9f\n",ang);
	   if(ang != 1000){
		   totalang+=ang;
		   samps++;
	   }
	   printf("Average Angle: %0.9f\n",totalang/samps);
//	   //retRealAngle();
//	   DataReturnRaw(); //Returns data back as unformated hexadecimal - possible to speed up if only 12bits are returned instead of 16 per adc
	   //DataReturnFloat(); //Returns data back as float - (REQUIRES PmodAD1 DRIVER)
	   //DemoRun();
	   printf("\n");

   }
	 //  DataReturnRaw();
   DemoCleanup();
   return 0;
}

