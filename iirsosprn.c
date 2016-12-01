// iirsosprn.c generic iir filter using cascaded second order sections
// input from PRBS generator function, output to line out
// float coefficients read from included .cof file
 
#include "DSK6713_AIC23.h"	//codec-DSK interface support
Uint32 fs=DSK6713_AIC23_FREQ_16KHZ;	//set sampling rate

#define DSK6713_AIC23_INPUT_MIC 0x0015
#define DSK6713_AIC23_INPUT_LINE 0x0011
Uint16 inputsource=DSK6713_AIC23_INPUT_LINE;

#include "1.cof"
#include "2.cof"
#include "3.cof"
#include "4.cof"
#include "5.cof"
#include "6.cof"

float w1[NUM1_SECTIONS][2] = {0};
float w2[NUM2_SECTIONS][2] = {0};
float w3[NUM3_SECTIONS][2] = {0};
float w4[NUM4_SECTIONS][2] = {0};
float w5[NUM5_SECTIONS][2] = {0};
float w6[NUM6_SECTIONS][2] = {0};

float lpg = 1;
float bp1g = 1;
float bp2g = 1;
float bp3g = 1;
float bp4g = 1;
float hpg = 1;

#include "noise_gen.h"	            //support file for noise sequence generation 
int fb;                            //feedback variable
shift_reg sreg;    
int signal = 0;                //shift register 
#define NOISELEVEL 8000            //scale factor for +/- 1 noise sequence

int prand(void) 	                  //pseudo-random noise generation
{
  int prnseq;			            
  if(sreg.bt.b0)
    prnseq = -NOISELEVEL;          //scaled negative noise level
  else
    prnseq = NOISELEVEL;           //scaled positive noise level
  fb =(sreg.bt.b0)^(sreg.bt.b1);   //XOR bits 0,1
  fb^=(sreg.bt.b11)^(sreg.bt.b13); //with bits 11,13 -> fb
  sreg.regval<<=1;                 //shift register 1 bit to left
  sreg.bt.b0=fb;                   //close feedback path
  return prnseq;
}


void resetreg(void)
{
  sreg.regval = 0xFFFF;        //shift register to nominal values
  fb = 1;                      //initial feedback value
  return;
}


interrupt void c_int11()	 //interrupt service routine
{
  int section;   // index for section number
  float input, in1, in2, in3, in4, in5, in6;   // input to each section
  float wn,yn, yn1, yn2, yn3, yn4, yn5, yn6;   // intermediate and output values in each stage

  if (signal == 0)
  {
	input =((float)prand());   // input to first section read from codec
  }
  else 
  {
	input = input_left_sample();
  }

	in1 = input;
	in2 = input;
	in3 = input;
	in4 = input;
	in5 = input;
	in6 = input;


  //Low Pass Filter 1
	for (section=0 ; section< NUM1_SECTIONS ; section++)
  {
    wn = in1 - a1[section][0]*w1[section][0] - a1[section][1]*w1[section][1];
    yn1 = b1[section][0]*wn + b1[section][1]*w1[section][0] + b1[section][2]*w1[section][1];
    w1[section][1] = w1[section][0];
    w1[section][0] = wn;
    in1 = yn1;              // output of current section will be input to next
  }

  // BPF 2
	for (section=0 ; section< NUM2_SECTIONS ; section++)
  {
    wn = in2 - a2[section][0]*w2[section][0] - a2[section][1]*w2[section][1];
    yn2 = b2[section][0]*wn + b2[section][1]*w2[section][0] + b2[section][2]*w2[section][1];
    w2[section][1] = w2[section][0];
    w2[section][0] = wn;
    in2 = yn2;              // output of current section will be input to next
  }

  // BPF 3
	for (section=0 ; section< NUM3_SECTIONS ; section++)
  {
    wn = in3 - a3[section][0]*w3[section][0] - a3[section][1]*w3[section][1];
    yn3 = b3[section][0]*wn + b3[section][1]*w3[section][0] + b3[section][2]*w3[section][1];
    w3[section][1] = w3[section][0];
    w3[section][0] = wn;
    in3 = yn3;              // output of current section will be input to next
  }
  // BPF 4
	for (section=0 ; section< NUM4_SECTIONS ; section++)
  {
    wn = in4 - a4[section][0]*w4[section][0] - a4[section][1]*w4[section][1];
    yn4 = b4[section][0]*wn + b4[section][1]*w4[section][0] + b4[section][2]*w4[section][1];
    w4[section][1] = w4[section][0];
    w4[section][0] = wn;
    in4 = yn4;              // output of current section will be input to next
  }
  // BPF 5
	for (section=0 ; section< NUM5_SECTIONS ; section++)
  {
    wn = in5 - a5[section][0]*w5[section][0] - a5[section][1]*w5[section][1];
    yn5 = b5[section][0]*wn + b5[section][1]*w5[section][0] + b5[section][2]*w5[section][1];
    w5[section][1] = w5[section][0];
    w5[section][0] = wn;
    in5 = yn5;              // output of current section will be input to next
  }
  // High Pass Filter 6

  for (section=0 ; section< NUM6_SECTIONS ; section++)
  {
    wn = in6 - a6[section][0]*w6[section][0] - a6[section][1]*w6[section][1];
    yn6 = b6[section][0]*wn + b6[section][1]*w6[section][0] + b6[section][2]*w6[section][1];
    w6[section][1] = w6[section][0];
    w6[section][0] = wn;
    in6 = yn6;              // output of current section will be input to next
  }

  yn = (yn1*lpg)+(yn2*bp1g)+(yn3*bp2g)+(yn4*bp3g)+(yn5*bp4g)+(yn6*hpg);

  output_left_sample((short)yn); // before writing to codec
  return;                       //return from ISR
}

void main()
{
  resetreg();
  comm_intr();                  //init DSK, codec, McBSP
  while(1);                     //infinite loop
}
