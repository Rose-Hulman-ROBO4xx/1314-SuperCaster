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
#include <portb.h>



/** Define Constants Here ******************************************/
#define   JMAX 1024
#define   JHALF 512
#define   JYBUFFER 75
//#define   JXBUFFER 50
//#define   JYBUFFER 400
#define   ANGMAX   360
#define   STOPTURNSPEED 0
//#define   LMOTORDIRECTION PORTBbits.RB1
//#define   RMOTORDIRECTION PORTBbits.RB0
//Debug pins
#define   LMOTORDIRECTION PORTCbits.RC0
#define   RMOTORDIRECTION PORTCbits.RC3

//unsigned char lastByte;
int lastVal=0;
int rxAngle=0;
int rxSpeed=0;
char BytetoSend = '0';
//_Bool rotEncoder[8];

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
    int myAng=0;

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
    TRISC = 0x80;  //port C RC7 is serial comm input
    TRISB = 0xFF;  //port B all output

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
    BytetoSend = 'a';
    while(1){
        /*
        myAng = getAngle();
        BytetoSend = myAng+48;
        printf('%c',BytetoSend);
        XLCDClear();
        sprintf(line1,"Angle: %i",(myAng));
        XLCDL1home();
        XLCDPutRamString(line1);*/

        printf('%c',BytetoSend);

        if(uartFlag == 1){
            uartFlag = 0;

            //Uncomment for LCD debugging
            /*
            XLCDClear();
            //uartFlag++;
            sprintf(line1,"Angle: %i",getAngle());
            XLCDL1home();
            XLCDPutRamString(line1);
            sprintf(line2,"Speed: %i",rxSpeed);
            XLCDL2home();
            XLCDPutRamString(line2);*/

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
        Delay10KTCYx(10);
    }

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
   // printf("%i",currAng);
    //currAng = Ang;




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
    /*
    XLCDClear();
    //uartFlag++;
    sprintf(line1,"PWM: %i, %i",ML_PWM,MR_PWM);
    XLCDL1home();
    XLCDPutRamString(line1);
    sprintf(line2,"Dir: %i, %i --%i",ML_Dir,MR_Dir,Ang);
    XLCDL2home();
    XLCDPutRamString(line2);*/

    return;
}
/*****************************************************************
* Function:        int getAngle()
* Input:           void
* Output:          integer as angle between 0 and 360
* Overview:        Reads rotary encoder angle and returns integer angle
******************************************************************/
int getAngle(){
    int i = 0;
    int encVal = 0;
    int angle =0;
    int pos = 0;
    int rotEncoder[8];

    rotEncoder[0] = PORTBbits.RB0;
    rotEncoder[1] = PORTBbits.RB1;
    rotEncoder[2] = PORTBbits.RB2;
    rotEncoder[3] = PORTBbits.RB3;
    rotEncoder[4] = PORTBbits.RB4;
    rotEncoder[5] = PORTBbits.RB5;
    rotEncoder[6] = PORTBbits.RB6;
    rotEncoder[7] = PORTBbits.RB7;


    // binary to integer conversion
    for(i=0; i<8;i++){
        encVal += rotEncoder[i]*(2^i);
    }
    /*XLCDClear();
    //sprintf(line2,"%i %i %i %i %i %i %i %i",PORTBbits.RB0,PORTBbits.RB1,PORTBbits.RB2,PORTBbits.RB3,PORTBbits.RB4,PORTBbits.RB5,PORTBbits.RB6,PORTBbits.RB7);
    sprintf(line1,"encVal: %c",(encVal+48));
    XLCDL1home();
    XLCDPutRamString(line1);
    XLCDL2home();
    XLCDPutRamString(line2);*/

    //printf('%c',(encVal+48));

    switch (encVal) {
        case 127:
            pos = 0;
        break;
        case 63:
            pos = 1;
        break;
        case 62:
            pos = 2;
        break;
        case 58:
            pos = 3;
        break;
        case 56:
            pos = 4;
        break;
        case 184:
            pos = 5;
        break;
        case 152:
            pos = 6;
        break;
        case 24:
            pos = 7;
        break;
        case 8:
            pos = 8;
        break;
        case 72:
            pos =9;
        break;
        case 73:
            pos =10;
        break;
        case 77:
            pos =11;
        break;
        case 79:
            pos =12;
        break;
        case 15:
            pos =13;
        break;
        case 47:
            pos = 14;
        break;
        case 175:
            pos = 15;
        break;
        case 191:
            pos =16;
        break;
        case 159:
            pos =17;
        break;
        case 31:
            pos =18;
        break;
        case 29:
            pos =19;
        break;
        case 28:
            pos =20;
        break;
        case 92:
            pos =21;
        break;
        case 76:
            pos =22;
        break;
        case 12:
            pos =23;
        break;
        case 4:
            pos =24;
        break;
        case 36:
            pos =25;
        break;
        case 164:
            pos =26;
        break;
        case 166:
            pos =27;
        break;
        case 167:
            pos =28;
        break;
        case 135:
            pos =29;
        break;
        case 151:
            pos =30;
        break;
        case 215:
            pos =31;
        break;
        case 223:
            pos =32;
        break;
        case 207:
            pos =33;
        break;
        case 143:
            pos =34;
        break;
        case 142:
            pos =35;
        break;
        case 14:
            pos =36;
        break;
        case 46:
            pos =37;
        break;
        case 38:
            pos = 38;
        break;
        case 6:
            pos = 39;
        break;
        case 2:
            pos =40;
        break;
        case 18:
            pos =41;
        break;
        case 82:
            pos =42;
        break;
        case 83:
            pos =43;
        break;
        case 211:
            pos =44;
        break;
        case 195:
            pos =45;
        break;
        case 203:
            pos =46;
        break;
        case 235:
            pos =47;
        break;
        case 239:
            pos =48;
        break;
        case 231:
            pos =49;
        break;
        case 199:
            pos =50;
        break;
        case 71:
            pos =51;
        break;
        case 7:
            pos =52;
        break;
        case 23:
            pos =53;
        break;
        case 19:
            pos =54;
        break;
        case 3:
            pos =55;
        break;
        case 1:
            pos =56;
        break;
        case 9:
            pos =57;
        break;
        case 41:
            pos = 58;
        break;
        case 169:
            pos = 59;
        break;
        case 233:
            pos = 60;
        break;
        case 225:
            pos = 61;
        break;
        case 229:
            pos = 62;
        break;
        case 245:
            pos = 63;
        break;
        case 247:
            pos = 64;
        break;
        case 243:
            pos = 65;
        break;
        case 227:
            pos = 66;
        break;
        case 163:
            pos = 67;
        break;
        case 131:
            pos = 68;
        break;
        case 139:
            pos = 69;
        break;
        case 137:
            pos = 70;
        break;
        case 129:
            pos = 71;
        break;
        case 128:
            pos = 72;
        break;
        case 132:
            pos = 73;
        break;
        case 148:
            pos = 74;
        break;
        case 212:
            pos = 75;
        break;
        case 244:
            pos = 76;
        break;
        case 240:
            pos = 77;
        break;
        case 242:
            pos = 78;
        break;
        case 250:
            pos = 79;
        break;
        case 251:
            pos = 80;
        break;
        case 249:
            pos = 81;
        break;
        case 241:
            pos = 82;
        break;
        case 209:
            pos = 83;
        break;
        case 193:
            pos = 84;
        break;
        case 197:
            pos = 85;
        break;
        case 196:
            pos = 86;
        break;
        case 192:
            pos = 87;
        break;
        case 64:
            pos = 88;
        break;
        case 66:
            pos = 89;
        break;
        case 74:
            pos = 90;
        break;
        case 106:
            pos = 91;
        break;
        case 122:
            pos = 92;
        break;
        case 120:
            pos = 93;
        break;
        case 121:
            pos = 94;
        break;
        case 125:
            pos = 95;
        break;
        case 253:
            pos = 96;
        break;
        case 252:
            pos = 97;
        break;
        case 248:
            pos = 98;
        break;
        case 232:
            pos = 99;
        break;
        case 224:
            pos = 100;
        break;
        case 226:
            pos = 101;
        break;
        case 98:
            pos = 102;
        break;
        case 96:
            pos = 103;
        break;
        case 32:
            pos = 104;
        break;
        case 33:
            pos = 105;
        break;
        case 37:
            pos = 106;
        break;
        case 53:
            pos = 107;
        break;
        case 61:
            pos = 108;
        break;
        case 60:
            pos = 109;
        break;
        case 188:
            pos = 110;
        break;
        case 190:
            pos = 111;
        break;
        case 254:
            pos = 112;
        break;
        case 126:
            pos = 113;
        break;
        case 124:
            pos = 114;
        break;
        case 116:
            pos = 115;
        break;
        case 112:
            pos = 116;
        break;
        case 113:
            pos = 117;
        break;
        case 49:
            pos = 118;
        break;
        case 48:
            pos = 119;
        break;
        case 16:
            pos = 120;
        break;
        case 144:
            pos = 121;
        break;
        case 146:
            pos = 122;
        break;
        case 154:
            pos = 123;
        break;
        case 158:
            pos = 124;
        break;
        case 30:
            pos = 125;
        break;
        case 94:
            pos = 126;
        break;
        case 95:
            pos = 127;
        break;
        default:
            pos = 64;
        break;
    }

    angle = pos*(360/127);
        /*
    sprintf(line2,"pos: %i ang: %i",pos,angle);
    XLCDL2home();
    XLCDPutRamString(line2);*/

    return angle;
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

