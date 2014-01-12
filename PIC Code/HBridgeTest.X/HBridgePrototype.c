/********************************************************************
* FileName:        HBridgePrototype.c
* Processor:       PIC18F4520
* Compiler:        MPLAB C18 v.3.06
*
* This program uses PWM to control 2 motors connected through a motor
* controller
*
* Author               Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// Chris Nelson       12.15.2013


/** Header Files ***********************************************/
#include <p18f4520.h>
#include <adc.h>
#include <stdio.h>
#include <timers.h>
#include <pwm.h>
#include <delays.h>

/** Define Constants Here ******************************************/
#define   JMAX 1023
#define   JHALF 512
#define   JYBUFFER 75
#define   JXBUFFER 50
//#define   LMOTORDIRECTION PORTBbits.RB1
//#define   RMOTORDIRECTION PORTBbits.RB0
//Debug pins
#define   LMOTORDIRECTION PORTCbits.RC0
#define   RMOTORDIRECTION PORTCbits.RC3

/** Local Function Prototypes **************************************/
void low_isr(void);
void high_isr(void);
void startTheBuzzer(void);
void stopTheBuzzer(void);
void setMotors(int,int);

// ============================================================
// Configuration Bits
#pragma config OSC = INTIO67
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config BOREN = OFF
#pragma config XINST = OFF

/** Declare Interrupt Vector Sections ****************************/
#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
   _asm goto high_isr _endasm
}

#pragma code low_vector=0x18
void interrupt_at_low_vector(void)
{
   _asm goto low_isr _endasm
}

/** Global Variables *********************************************/
	int Xresult, Yresult,ML_PWM,MR_PWM,ML_Dir,MR_Dir;

/*****************************************************************
* Function:        void main(void)
******************************************************************/
#pragma code

void main (void)
{
 // configure A/D convertor
    // config 1 = Setup the timing to a conservative value (you don't need to ever change this)
    // config 2 = Use channel 0, not interrupts off, use the power and ground as referrences
    // portconfig = 0x0E setup only analog 0 as a possible analog input pin

   OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_12_TAD,
            ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS,
            0x0E);

  ADCON0 = 0x01;        //Turn On ADC
  ADCON1 = 0x0B;        //Select Vref+ = VDD, Vref- = VSS, AN0-AN3 = Analog Input
//    ADCON2 = 0xA9;        //### Acquisition delay 12 TAD, A/D conversion clock 8TOSC, Right Justified
  TRISA = 0xFF;  //port A all input
  TRISC = 0x00;  //port C all output
  TRISB = 0x00;  //port B all output

  //Initialize PWM channels
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_16);

    //PWM period = [(period)+1]*4*Tosc*TMR2prescaler
    OpenPWM1(178);
    OpenPWM2(178);

    SetDCPWM1(0);  //(0%-100% => 0-1023)
    SetDCPWM2(0);

    //Initialize digital outputs
    PORTB = 0x00;

  //Set initial variable values:
  ML_Dir = 1; //forward
  MR_Dir = 1;
  ML_PWM = 0; //stopped
  MR_PWM = 0;


  //set initial duty cycle
    SetDCPWM1(ML_PWM);
    SetDCPWM2(MR_PWM);

    //Set inital direction
    LMOTORDIRECTION = ML_Dir;
    RMOTORDIRECTION = MR_Dir;

  	while (1)
    {
    //Collect analog inputs from joystick
  	  SetChanADC( ADC_CH2 );	// Select the pin
  		ConvertADC(); 		    	// Start conversion
  		while( BusyADC() ); 		// Wait for completion
  		Xresult = ReadADC(); 		// Read result

  		SetChanADC( ADC_CH3 );		// Select the pin
  		ConvertADC(); 		      	// Start conversion
  		while( BusyADC() ); 	  	// Wait for completion
  		Yresult = ReadADC();

    //Set motor speed and direction
  		setMotors(Xresult,Yresult);
    }

}

/*****************************************************************
* Function:        void setMotors(int,int)
* Input:           X: 0-1023 left-right value
                   Y: 0-1023 backward-forward value
* Output:          None
* Overview:        Sets PMW1 and 2's duty cycles as well as 2 Motor direction pins
                   Duty cycle (speed) is calculated using the following:
                   Y -= 512
                   Left Motor = X * ((2*abs(Y)+1023)/1023)
                   Right Motor = (1023-X) * ((2*abs(Y)+1023)/1023)

                   This function assumes the following motor controller
                   connections:

                          Duty cycle          Direction:
                   Left:  PWM1 (RC2 or 17)    RC3 (33)
                   Right: PWM2 (RC1 or 16)    RC0 (34)

******************************************************************/
void setMotors(int X,int Y){
    int M_Coeff;

    //Center possible analog values around 0
    Y = Y - JHALF;
    //X = X - JHALF;
    //If joystick is forward, go forwards
	  if(Y > JYBUFFER) {
	    ML_Dir = 0;
	    MR_Dir = 0;
	    M_Coeff = (Y*2 +JMAX)/JMAX; //Used for determining Speed proportion

    //If joystick is backwards, go backwards
	  } else if(Y < (-1*JYBUFFER)){
	    ML_Dir = 1;
	    MR_Dir = 1;
	    M_Coeff = (Y*(-2)+JMAX)/JMAX; //Used for determining Speed proportion
      //Make sure direction is forward for turning when Y value is below buffer
	  } else if((X-JHALF) > JXBUFFER){
             M_Coeff = 1;
             ML_Dir = 0;
             MR_Dir = 0;
          } else if((X-JHALF)< (-1*JXBUFFER)){
             M_Coeff = 1;
             ML_Dir = 0;
             MR_Dir = 0;
        //If joystick is in the center, don't move
          }else{
             M_Coeff = 0;
             ML_Dir = 0;
	     MR_Dir = 0;
          }

      //find pwm values
   // if(X >= ){
      ML_PWM = X * M_Coeff;
      MR_PWM = (JMAX-X)*M_Coeff;
      /*
       }else{
      ML_PWM = -2*X * M_Coeff;
      MR_PWM = (JMAX+2*X)*M_Coeff;
    }
       
       
       */
    

    //Make sure PWM values are valid
    if( ML_PWM > 1023){
        ML_PWM = 1023;
    }
    if (MR_PWM > 1023){
        MR_PWM = 1023;
    }
    //set duty cycle
    SetDCPWM1(ML_PWM);
    SetDCPWM2(MR_PWM);

    //Set direction
    LMOTORDIRECTION = ML_Dir;
    RMOTORDIRECTION = MR_Dir;
}


/*****************************************************************
* Function:        void high_isr(void)
* Input:
* Output:
* Overview:
******************************************************************/
#pragma interrupt high_isr			// declare function as high priority isr
void high_isr(void)
{
}

/******************************************************************
* Function:        void low_isr(void)
* Input:
* Output:
* Overview:
********************************************************************/
#pragma interruptlow low_isr		// declare function as low priority isr
void low_isr(void)
{
}

