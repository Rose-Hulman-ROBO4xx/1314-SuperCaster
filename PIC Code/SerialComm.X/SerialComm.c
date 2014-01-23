/********************************************************************
* FileName:        SerialComm.c
* Processor:       PIC18F4520
* Compiler:        MPLAB C18 v.3.06
*
* 
*
* Author               Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
// Chris Nelson       1.15.2014


/** Header Files ***********************************************/
#include <p18f4520.h>
#include <adc.h>
#include <stdio.h>
#include <timers.h>
#include <pwm.h>
#include <delays.h>
#include <usart.h>
#include "LCD Module.h"



/** Define Constants Here ******************************************/
#define   JMAX 1024
#define   JHALF 512
#define   JYBUFFER 75
//#define   JXBUFFER 50
//#define   JYBUFFER 400
#define   ANGMAX   360
#define   STOPTURNSPEED 512
//#define   LMOTORDIRECTION PORTBbits.RB1
//#define   RMOTORDIRECTION PORTBbits.RB0
//Debug pins
#define   LMOTORDIRECTION PORTCbits.RC0
#define   RMOTORDIRECTION PORTCbits.RC3

//unsigned char lastByte;
int lastVal=0;
int rxAngle=0;
int rxSpeed=0;

/** Local Function Prototypes **************************************/
void low_isr(void);
void high_isr(void);
void startTheBuzzer(void);
void stopTheBuzzer(void);
void setMotorsVector(int,int);
int getAngle();

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
        char line1[20];
	char line2[20];
        int uartFlag;
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


    //Setup serial communications
    RCONbits.IPEN = 1;
    PIE1bits.RCIE = 1;
    IPR1bits.RCIP = 1;
    INTCONbits.GIE = 1;        
    
    OpenUSART(USART_TX_INT_OFF &
        USART_RX_INT_ON &
        USART_ASYNCH_MODE &
        USART_EIGHT_BIT &
        USART_CONT_RX &
        USART_BRGH_HIGH, 25);

    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_12_TAD,
            ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS,
            0x0E);

    ADCON0 = 0x01;        //Turn On ADC
    ADCON1 = 0x0B;        //Select Vref+ = VDD, Vref- = VSS, AN0-AN3 = Analog Input
    //    ADCON2 = 0xA9;        //### Acquisition delay 12 TAD, A/D conversion clock 8TOSC, Right Justified
    TRISA = 0xFF;  //port A all input
    TRISC = 0x80;  //port C
    TRISB = 0x00;  //port B all output

    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 0;
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

    XLCDInit();
    XLCDClear();
    while(1){
        if(uartFlag == 1){
            uartFlag = 0;
            XLCDClear();
            //uartFlag++;
            sprintf(line1,"Angle: %i",rxAngle,rxAngle);
            XLCDL1home();
            XLCDPutRamString(line1);
            sprintf(line2,"Speed: %i",rxSpeed);
            XLCDL2home();
            XLCDPutRamString(line2);
            //Set motor speed and direction
            setMotorsVector(rxAngle,rxSpeed);
        }
        //PIC Serial loopback test
        /*if(PORTAbits.RA4 ==0){ 
            printf("%i",5);
            printf("%c",'A');
            printf("%i",10);
            printf("%c",'S');
        }*/
    
    /*
    Delay10KTCYx(50);*/
    }

}
/*****************************************************************
* Function:        int getAngle()
* Input:           void
* Output:          integer as angle between 0 and 360
* Overview:        Reads rotary encoder angle and returns integer angle
******************************************************************/
int getAngle(){
    return 180;
}


/*****************************************************************
* Function:        void setMotorsVector(int,int)
* Input:           Ang: 0-360 desired angle in degrees
                   Mag: 0-1024 desired speed (backward and forward value)
* Output:          None
* Overview:        Sets PMW1 and 2's duty cycles as well as 2 Motor direction pins
                   Duty cycle (speed) is calculated using a proportional angle
                   control

                   This function assumes the following motor controller
                   connections:

                          Duty cycle          Direction:
                   Left:  PWM1 (RC2 or 17)    RC3 (33)
                   Right: PWM2 (RC1 or 16)    RC0 (34)

******************************************************************/

void setMotorsVector(int Ang,int Mag){
    float L_Coeff = 0;
    float R_Coeff = 0;
    int currAng=0;
    int dAng=0;

    //Center possible analog values around 0
    Mag = Mag - JHALF;
    //Adjust Mag to -1024 to 1024 (For PWM duty cycle)
    Mag = 2*Mag;
    //find current supercaster angle:
    currAng = getAngle();
    dAng = currAng-Ang; //Find amount of degrees currently off from target angle

 //Determine wheel porportions based upon current/desired angles
    //Caster must turn to the right:
    if (currAng > Ang){
        L_Coeff = 1;
        R_Coeff = 1-(dAng/ANGMAX);  //As the angle is closer to being correct, it TURNS slower
    //Caster must turn to the left:
    }else if(currAng < Ang){
        L_Coeff = 1+(dAng/ANGMAX); //Add because dAng will be negative
        R_Coeff = 1;
    //Default to straight
    }else{
        L_Coeff = 1;
        R_Coeff = 1;
    }

//Determine motor directions and calculate speed values
    //If input Mag > 512+buffer, go forwards
      if(Mag > JYBUFFER) {
        ML_Dir = 0;
        MR_Dir = 0;
        //find pwm values
        ML_PWM = Mag * L_Coeff;
        MR_PWM = Mag * R_Coeff;
        
    //If input Mag < 512-buffer, go backwards
      } else if(Mag < (-1*JYBUFFER)){
        ML_Dir = 1;
        MR_Dir = 1;
        //find pwm values
        ML_PWM = -1*Mag * L_Coeff;
        MR_PWM = -1*Mag * R_Coeff;

    //If not otherwise moving...
      }else{
         //rotate caster appropriately:
         if(currAng>Ang){
             ML_Dir = 1;
             MR_Dir = 0;
             ML_PWM = STOPTURNSPEED;
             MR_PWM = STOPTURNSPEED;
         }else if(currAng<Ang){
             ML_Dir = 0;
             MR_Dir = 1;
             ML_PWM = STOPTURNSPEED;
             MR_PWM = STOPTURNSPEED;
         }else{
             //default direction is forward (in-between buffers) and not moving
             ML_Dir = 0;
             MR_Dir = 0;
             ML_PWM = 0;
             MR_PWM = 0;
         }
      }

    //Make sure PWM values are valid
    if( ML_PWM > 1024){
        ML_PWM = 1024;
    }
    if (MR_PWM > 1024){
        MR_PWM = 1024;
    }
    //set duty cycle
    SetDCPWM1(ML_PWM);
    SetDCPWM2(MR_PWM);

    //Set direction
    LMOTORDIRECTION = ML_Dir;
    RMOTORDIRECTION = MR_Dir;
    return;
}

/*****************************************************************
* Function:        void high_isr(void)
* Input:
* Output:
* Overview:
******************************************************************/
#pragma interrupt high_isr	// declare function as high priority isr
void high_isr(void)
{
    if(PIR1bits.RCIF){
        char newByte;
        
        newByte = RCREG;
        PIR1bits.RCIF = 0; //reset flag
        
        if((char)(newByte) == 'A'){
            rxAngle = lastVal;
            lastVal = 0;
        }else if((char)(newByte)== 'S'){
            rxSpeed = lastVal;
            lastVal = 0;
            uartFlag = 1;
        }else{
            lastVal = lastVal*10 +(newByte-48);
        }
    }
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

