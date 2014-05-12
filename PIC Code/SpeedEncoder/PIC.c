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

// Configuration Bits
#pragma config OSC = INTIO67
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config BOREN = OFF
#pragma config XINST = OFF

/** Define Constants Here ******************************************/
#define   JMAX 1024
#define   JHALF 512
#define   JYBUFFER 50
//#define   JXBUFFER 50
//#define   JYBUFFER 400
#define   ANGMAX   360
#define   ANGBUFF   5
#define   ANGCOEF   8
#define   MAXSPEED 1023
#define   STOPTURNSPEED MAXSPEED*0.25
//#define   LMOTORDIRECTION PORTBbits.RB1
//#define   RMOTORDIRECTION PORTBbits.RB0
//Debug pins
#define   LMOTORDIRECTION PORTCbits.RC0
#define   RMOTORDIRECTION PORTCbits.RC3

//#define   BRAKEPIN PORTDbits.RD2
#define   SEGMENTS   32 
#define   TIMER0START   64755 //triggers every .1 second
#define TIMER0ZERO 0
//PID controller values
#define A_ERROR_LIMIT 120 //in rpm, can be adjusted
#define KP	1
#define KI	.1
#define KD	1
#define PIDTIMER	100

/** Local Function Prototypes **************************************/
void low_isr(void);
void high_isr(void);
void setMotorsVector(int,int);
void getIR(void);
int getAngle();
int PIDContrl();

// ============================================================


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
//unsigned char lastByte;
int brakeFlag=0;
int lastVal=0;
int rxAngle=0;
int rxSpeed=0;
char BytetoSend = '0';
//_Bool rotEncoder[8];
int Xresult, Yresult,ML_PWM,MR_PWM,ML_Dir,MR_Dir,currAng;
char line1[20];
char line2[20];
int uartFlag;

int previous_time=0;
int previous_count = 0;
float current_speed=0;

int a_error = 0;
int p_error = 0;
int d_error = 0;
int PIDcount = 0;
int Mag = 0;


/*****************************************************************
* Function:        void main(void)
******************************************************************/
#pragma code

void main (void)
{
    int i = 0;
 // configure A/D convertor
    // config 1 = Setup the timing to a conservative value (you don't need to ever change this)
    // config 2 = Use channel 0, not interrupts off, use the power and ground as referrences
    // portconfig = 0x0E setup only analog 0 as a possible analog input pin
    //int myAng=0;

    //Setup serial communications
    RCONbits.IPEN = 1;
    PIE1bits.RCIE = 1;
    IPR1bits.RCIP = 1;
    INTCONbits.GIE = 1;

    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 0;

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
    ADCON2 = 0x00;

    //    ADCON2 = 0xA9;        //### Acquisition delay 12 TAD, A/D conversion clock 8TOSC, Right Justified
    TRISA = 0xFF;  //port A all input
    TRISC = 0x80;  //port C RC7 is serial comm input
    TRISB = 0xFF;  //port B all input
    TRISD = 0x00;

    //Initialize outputs
    PORTC = 0x00;
    PORTD = 0x00;

    //Initialize PWM channels
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_16);
    

    //PWM period = [(period)+1]*4*Tosc*TMR2prescaler
    OpenPWM1(178);
    OpenPWM2(178);

    SetDCPWM1(0);  //(0%-100% => 0-1023)
    SetDCPWM2(0);

    //Set initial variable values:
    ML_Dir = 0; //forward
    MR_Dir = 0;
    ML_PWM = 0; //stopped
    MR_PWM = 0;

    //Set inital direction
    LMOTORDIRECTION = ML_Dir;
    RMOTORDIRECTION = MR_Dir;

    //PID Speed control/////////////
    //Open timer
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_128 );
    WriteTimer0(TIMER0START);

    //Set RB0 as interuppt
    INTCONbits.INT0IE = 1;
    INTCONbits.INT0IF = 0;
    INTCON2bits.INTEDG0 = 1;

    while(1){
        /*if(brakeFlag){ //Brake the motors if flag is set
            BRAKEPIN = 1;
        }else*/
        if(i>500){ //If no new signal has been recieved for X cycles, stop motors
            setMotorsVector(180,512);
            //SetDCPWM1(0);
            //SetDCPWM2(0);
            //Delay10KTCYx(50);
        }

        getAngle();
        //ngetIR();
        
        

        if(uartFlag == 1){
            i = 0; //feed watchdog
            getAngle();
            uartFlag = 0;

            //Set motor speed and direction
            setMotorsVector(rxAngle,rxSpeed);
        }
        i++;
        //PIC Serial loopback test
        /*if(PORTAbits.RA4 ==0){
            printf("%i",5);
            printf("%c",'A');
            printf("%i",10);
            printf("%c",'S');
        }*/
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

void setMotorsVector(int Ang,int set_speed){
    float L_Coeff = 0;
    float R_Coeff = 0;
    float temp;
    //int currAng=0;
    int dAng=0;
	
	
	//PID control
	//set_speed comes in as rpm
	int error = set_speed - current_speed;
	
	
	int output = PIDContrl(error);
	
	//replace with real transfer funtion
	int set_speed_pwm = set_speed;
	int output_pwm = output; 
	
	//use this when using PID
	//Mag = Mag + output_pwm;
	//use this for no PID but arduino sending down RPM values
	Mag = set_speed_pwm;
	//use this with arduino sending down pwm values
	Mag = set_speed;
	
    //Center possible analog values around 0 [(0,1024) to (-512, 512)]
    Mag = Mag - JHALF;
    //Spread Mag to from (-512, 512) to (-1024, 1024) (For PWM duty cycle)
    Mag = 2*Mag;


    if(Ang >270){
        Ang = 270;
    }else if(Ang < 90){
        Ang = 90;
    }

    dAng = currAng-Ang; //Find amount of degrees currently off from target angle

 //Determine wheel porportions based upon current/desired angles
    //Caster must turn to the right:
    if ((dAng)>(ANGBUFF)){
        //L_Coeff = 0.5;
        L_Coeff = .1;
        //R_Coeff = 1-ANGCOEF*(dAng/ANGMAX);  //As the angle is closer to being correct, it TURNS slower
        R_Coeff = 1;
        /*if(R_Coeff<0){
            R_Coeff = 0;
        }*/
    //Caster must turn to the left:
    }else if((dAng)<(-1*ANGBUFF)){
        //L_Coeff = 1+ANGCOEF*(dAng/ANGMAX); //Add because dAng will be negative
        L_Coeff = 1;
        //R_Coeff = 0.5;
        R_Coeff = .1;
        /*if(L_Coeff<0){
            L_Coeff = 0;
        }*/
    //Default to straight
    }else{
        L_Coeff = 1;
        R_Coeff = 1;
    }

//Determine motor directions and calculate speed values
    //If input Mag > 512+buffer, go forwards
      if(Mag > JYBUFFER) {
        // direction to forward
        ML_Dir = 0;
        MR_Dir = 0;
        //find pwm values
        ML_PWM = Mag * L_Coeff;
        MR_PWM = Mag * R_Coeff;

    //If input Mag < 512-buffer, go backwards
      } else if(Mag < (-1*JYBUFFER)){
        // direction to backward
        ML_Dir = 1;
        MR_Dir = 1;
        //find pwm values
        temp = L_Coeff;
        L_Coeff = R_Coeff;
        R_Coeff = temp;

        ML_PWM = -1*Mag * L_Coeff;
        MR_PWM = -1*Mag * R_Coeff;

    //If not otherwise moving...
      }else{
         //rotate caster appropriately:
         if((dAng)>(ANGBUFF)){
             ML_Dir = 0;
             MR_Dir = 1;
             ML_PWM = STOPTURNSPEED;
             MR_PWM = STOPTURNSPEED;
         }else if((dAng)<(-1*ANGBUFF)){
             ML_Dir = 1;
             MR_Dir = 0;
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

    return;
}

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

    encVal = (rotEncoder[0]+rotEncoder[1]*2+rotEncoder[2]*4+rotEncoder[3]*8+rotEncoder[4]*16+rotEncoder[5]*32+rotEncoder[6]*64+rotEncoder[7]*128);
    /*
    //PORTD = 0xFF;
    Delay10KTCYx(50);

    // binary to integer conversion
    for(i=0; i<8;i++){
        encVal += (int) ceil(rotEncoder[i]*pow(2,i));

        PORTD = encVal;
        Delay10KTCYx(20);
    }*/

    PORTD = encVal;

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
    angle = (int) pos*(2.8345); //  2.8345 = 360/127
    currAng = angle;

    return angle;
}

/*****************************************************************
* Function:        getIR(void)
* Input:
* Output:
* Overview: Reads the IR sensor and increments the count
******************************************************************/
void getIR(void)
{
    
}	

/*****************************************************************
* Function:      int PIDControl(int error)
* Input:	int error , difference between input speed and actual speed in rpm
* Output:	int output, speed to set the motors at
* Overview: Reads the IR sensor and increments the count
******************************************************************/
int PIDControl(int error)
{
	//PID variables
	int kd = 1;
	int ki = 1;
	int kp = 1;
	
	PIDcount = PIDcount + 1;
	
	if(PIDcount > PIDTIMER){//update the integral and derivative terms
	int a_error = a_error + error;
	int d_error = a_error - p_error;
	if(a_error > A_ERROR_LIMIT){
		a_error = A_ERROR_LIMIT;
		}	
	PIDcount = 0;
	}
	
	//get values
	int prop = KP*error;
	int integ = KI*a_error;
	int deriv = KD*d_error;
	
	int output = prop + integ + deriv;
	
	
	
	p_error = error;
	return output;	
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
        /*if((char)(newByte)=='B'){
            brakeFlag = lastVal;
            lastVal = 0;
        }
        else */
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
    if(INTCONbits.TMR0IF)// .1 seconds have past(78 ticks)
    {
	  INTCONbits.TMR0IF = 0; 
	  WriteTimer0(TIMER0ZERO);
	  current_speed = 0; //Timer overflow occured, meaing 8 seconds
                             //elapsed without a speed read
                  //(count/SEGMENTS)/(.00166);//gives rpm .00166 is .1sec in min
	  
    }
    if(INTCONbits.INT0IF) {
        INTCONbits.INT0IF = 0;
        int result = ReadTimer0();
        WriteTimer0(TIMER0ZERO);
        if (result < 30000 && previous_count < 30000 ) { //Ok to avrage 
            int total_count = result+previous_count;
            int avg_count = total_count/2;
            current_speed = (float) avg_count * .000128;
        } else { //Case of moving very slow, no overflow
            current_speed = (float) result * .000128;
        }
        previous_count = result;
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


