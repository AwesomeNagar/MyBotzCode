#include <stdio.h>
#include <math.h>
//Size of sampling array
#define SIZE 245
//Velocity of sound in water to approximate distances
#define VINWATER 1531
//Distance between the square configuration of phones
#define PHONEDIST 0.0127

//Frequency array to store different possible frequencies
int freq[1] = {//25,
	//30,
	//35,
	25000};

float sample1[SIZE];
float sample2[SIZE];
float sample3[SIZE];


void initSample(){
	for (int i = 0; i < SIZE; ++i)
	{
		//Read in the hydrophone data to the sample arrays
		scanf ("%f", &sample1[i]);
		scanf ("%f", &sample2[i]);
		scanf ("%f", &sample3[i]);

		//Commented out for testing purposes
		// sample1[i] = sin(M_PI*2*i);
		// sample2[i] = sin(M_PI*2*i+M_PI/8000);
		// sample3[i] = sin(M_PI*2*i+M_PI/4000);

	}
}
float DFT(float samp[]){
	float imgpart = 0;
	float realpart = 0;
	for (int i = 0; i < SIZE; ++i)
	{
		float angle = -2*M_PI*freq[0]*i/SIZE;
		realpart += cos(angle)*samp[i];
		imgpart += sin(angle)*samp[i];
	}
	//Get the phase shift using invtan
	return atan(imgpart/realpart);
}
   
float quadForm(float s, float b, float c){
	//Mathematically solve for the value of x, or the horizontal displacement from one hydrophone to the pinget
	float M = (pow(s,2)-b*c)*(b+c)/(2*c*s);
	float L = (pow(s,2)-pow(c,2))/(2*c);
	float AA = pow(c,2)+pow(b,2)-pow(s,2);
	float BB = 2*c*(M*b-L*s);
	float CC = (pow(c,2))*(pow(M,2)-pow(L,2));
	//Value of x determined by quadratic formula
	//NOTE: THERE ARE TWO SOLUTIONS TO THIS EQUATION, BUT IT IS HIGHLY LIKELY THAT IT WOULD BE THE LARGEST ONE
	return (-BB+pow(pow(BB,2)-4*AA*CC,0.5))/(2*AA);
}

float yfromx(float s, float b, float c, float x){
	//Given a value of x, find the value of y using math
	float M = (pow(s,2)-b*c)*(b+c)/(2*c*s);
	return (b*x/c+M-s);
}

int main()
{
	printf("%d",VINWATER);
   initSample();

   //Find the different phase shifts between when each phone received the signal
   float angle1 = DFT(sample1);
   float angle2 =DFT(sample2);
   float angle3 = DFT(sample3);
   float dif1;
   float dif2;  

   //Get the pairwise differences in phaseshifts ~ necessary for the math derivations
   if (angle1 <= angle2 && angle2 <= angle3)
   {
   		dif2 = angle3-angle2;
   		dif1 = angle2-angle1;
   }else if (angle3 <= angle1 && angle1 <= angle2)
   {
   		dif2 = angle2-angle1;
   		dif1 = angle1-angle3;
   }else if (angle1 <= angle3 && angle3 <= angle2)
   {
   		dif2 = angle2-angle3;
   		dif1 = angle3-angle1;
   }else if (angle2 <= angle1 && angle1 <= angle3)
   {
   		dif2 = angle3-angle1;
   		dif1 = angle1-angle2;
   }else if (angle2 <= angle3 && angle3 <= angle1)
   {
   		dif2 = angle1-angle3;
   		dif1 = angle3-angle2;
   }else if (angle3 <= angle2 && angle2 <= angle1)
   {
   		dif2 = angle1-angle2;
   		dif1 = angle2-angle3;
   }
  
  //Transform phase shift differences into time differences
   dif2 /= (freq[0]*M_PI*2);
   dif1 /= (freq[0]*M_PI*2);

   //Use kinematics d = vt to find the distance between when the phones received the phase shifts
   dif2 *= VINWATER;
   dif1 *= VINWATER;
    printf("\r\n");
   printf("%.9f",dif1);
   printf("\r\n");
   printf("%.9f",dif2);
   printf("\r\n");
   float x = quadForm(PHONEDIST,dif1,dif2);
   float y = yfromx(PHONEDIST,dif1,dif2,x);
   printf("%.9f",x);
   printf("\r\n");
   printf("%.9f", y);   
   return 0;
}
