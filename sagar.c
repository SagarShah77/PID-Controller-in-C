/*
 * PIDController.c
 *
 *  Created on: Apr 13, 2017
 *      Author: sagar
 */

#define number_IError    4     // number of integration terms for IError[n]
#define alpha1           1.0  // weighting for kp
#define alpha2           1.0  // weighting for ki
#define alpha3           1.0  // weighting for kd
#define minError        -1000.0// min error limit
#define maxError        1000.0 // max error limit

int indices = 1000;

float Error[1000];           // for error
float DError[1000];          // derivative of error
float FDError[1000];         // forward difference computation for the derivative of error
float BDError[1000];         // backward difference computation for the derivative of error
float CDError[1000];         // central difference computation for the derivative of error
float IError[1000];          // integral of error (squared each individual error)
float SumError[1000];        // summation of pid errors
float cntrlOut[1000];

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "vehicleModel.h"


float calculateBD(float Err, int position)
{
	Error[position] = Err;
	float Kernel[2] = {-1,1};
	if (position==0)
		BDError[0] = Error[0]*Kernel[0]+Error[0]*Kernel[1];
	else
		BDError[position] = Error[position]*Kernel[1]+Error[position-1]*Kernel[0];
	return BDError[position];
}

float calculateCD(float Err, int position)
{
	Error[position] = Err;
	float Kernel[3] = {-0.5,0,0.5};
	if (position==0)
		CDError[0] = Error[0]*Kernel[0]+Error[0]*Kernel[1]+Error[1]*Kernel[2];
	else if (position==9)
		CDError[9] = Error[8]*Kernel[0]+Error[9]*Kernel[1]+Error[9]*Kernel[2];
	else
		CDError[position] = Error[position+1]*Kernel[2]+Error[position]*Kernel[1]+Error[position-1]*Kernel[0];
	return CDError[position];
}

float calculateIntegral(float Err, int position)
{
	Error[position] = Err;
	if (position==0)
		IError[0] = Error[0]*Error[0];
	else if (position==1)
		IError[1] = Error[1]*Error[1]+Error[0]*Error[0];
	else if (position==2)
		IError[2] = Error[2]*Error[2]+Error[1]*Error[1]+Error[0]*Error[0];
	else
	{
		IError[position] =  Error[position]*Error[position]+Error[position-1]*Error[position-1]+
				    Error[position-2]*Error[position-2]+Error[position-3]*Error[position-3];
	}

	return IError[position];
}

int main(int argc, char **argv) {

        int position = 0;
	int prevAngle;
	
	int Kp = 300;
	int Kd = 600;	
	int ki = 400;
	FILE * fp;
	fp = fopen("C:\\Anaconda\\Lib\\site-packages\\matplotlib\\mpl-data\\sample_data\\Lab2.csv", "w");
	if(fp == NULL){
		printf("Couldn't open file\n");
		return 1;
	}

	fprintf(fp, "%s,%s,%s\n",("Error"),("cntrlOut"),("SumOut"));
	for (position=0;position<indices;position++)
		{
			if (Error[position]>0.5)
			{
				Kp+=100;
				Kd+=100;
				ki+=100;
			}
			Error[position] = 1-cntrlOut[position];
			fprintf(fp, "%f,", (Error[position]));
			fprintf(fp, "%f,", (cntrlOut[position]));
			SumError[position] =  Kp*alpha1*Error[position]+Kd*alpha2*calculateCD(Error[position],position)		
			+ki*alpha3*calculateIntegral(Error[position], position);
			fprintf(fp, "%f\n", (SumError[position]));
			if (cntrlOut[position]>1)
				cntrlOut[position+1] = cntrlOut[position] - sin(RADDEG*((SumError[position]*SLOPEM
				+SLOCON)*STPMOT))*SPDVHL*ANGSMP;
			else 
				cntrlOut[position+1] = cntrlOut[position] + sin(RADDEG*((SumError[position]*SLOPEM
				+SLOCON)*STPMOT))*SPDVHL*ANGSMP;
						
		}
	fclose(fp);	

	return 0;
}
