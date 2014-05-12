/*********************************************************************
* FileName:        LCD Module.c
* Processor:       PIC18F4520
* Compiler:        MPLAB C18 v.3.06 
*
* This file contains the information necessary to use the LCD Module
*
*	Wiring:
*   (Important for your project LCD, obviouly this is already done on PICDEM 2)
*		LCD side	- PIC side
*
*		1: Vss		- Ground
*		3: Vo		- 1.2ish volts (middle leg of pot)
*		5: RW pin	- PORTD pin RD5 (could just ground it if writing only)
*		7: DB0		- not used in 4 bit mode
*		9: DB2		- not used in 4 bit mode
*		11: DB4		- PORTD pin RD0
*		13: DB6		- PORTD pin RD2
*
*		2: Vdd		- Power (5 volts)
*		4: RS pin	- PORTD pin RD4
*		6: EN pin	- PORTD pin RD6
*		8: DB1		- not used in 4 bit mode
*		10: DB3		- not used in 4 bit mode
*		12: DB5		- PORTD pin RD1
*		14: DB7		- PORTD pin RD3
*
*   Explaining the PIC side lines with a few more words:
*
*	  PORTD RD7:RD4 are control/power lines to LCD module
*		PORTD.6 = LCD EN    : LCD enable signal
*		PORTD.5 = RD/WR		: LCD Read/Write operation, (Read = 1; Write = 0)
*		PORTD.4 = RS        : LCD Register Select, (Instruction Reg = 0; Data Reg = 1)
*
*	  PORTD RD3:RD0 are data lines for LCD Module -- LCD Module used in 4-bit mode
*		PORTD.3 = LCD D7
*		PORTD.2 = LCD D6
*		PORTD.1 = LCD D5
*		PORTD.0 = LCD D4
*
* Author               Date    Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Naveen Raj        6/9/03      Original MPAM (Microchip Application Maestro) code
* David Fisher		9/24/08		Modified for ME430 use on PICDEM 2 boards
********************************************************************/

#include "LCD Module.h"
char _vXLCDreg =0;          //Used as a flag to check if from XLCDInit()

// Prototypes added by DSF 5/18/08 as well as functions at the end of .c file
void XLCDDelay15ms (void);
void XLCDDelay4ms (void);
void XLCD_Delay500ns(void);
void XLCDDelay(void);

/*********************************************************************
 * Function         : void XLCDInit(void)
 * PreCondition     : None
 * Input            : None
 * Output           : None
 * Side Effects     : None
 * Overview         : LCD is intialized
 * Note             : This function will work with all Hitachi HD447780
 *                    LCD controller.
 ********************************************************************/
void XLCDInit(void)
{
	// Add by DSF 9/26/08
	//  You need these three lines for the PICDEM 2 Board only
	//  If you are using an external LCD wire you can get back RD7 for IO
	//   by deleting these three lines of code	
	LCD_PWR = 1;
	TRISD = 0x00;
	LCD_PWR=1;   // to power up the LCD
	
	
/*This par of the code is initialization by instruction*/
_vXLCDreg=1; 

//PORT initialization  
    XLCD_DATAPORT_TRIS  &= 0xf0;
    XLCD_DATAPORT &= 0xf0;

//control port initialization
XLCD_RSPIN_TRIS =0;                         //make control ports output
XLCD_ENPIN_TRIS =0;
#ifndef XLCD_RW_GROUND
XLCD_RWPIN_TRIS =0;                         //if RW pin grounded
#endif

XLCD_RSPIN  =0;                             //clear control ports
XLCD_ENPIN  =0;
#ifndef XLCD_RW_GROUND
XLCD_RWPIN=0;                               //if RW pin grounded
#endif

//initialization by instruction
XLCDDelay15ms(); 
    XLCD_DATAPORT       &= 0xf0;    // Clear lower port
    XLCD_DATAPORT   |= 0b00000011;  // Function set cmd(4-bit interface)
    XLCD_ENPIN = 1;                 // Clock the cmd in
    XLCD_Delay500ns();   
    XLCD_ENPIN = 0;
    XLCDDelay4ms();
    XLCD_DATAPORT       &= 0xf0;    // Clear lower port
    XLCD_DATAPORT   |= 0b00000011;  // Function set cmd(4-bit interface)
    XLCD_ENPIN = 1;                 // Clock the cmd in
    XLCD_Delay500ns();   
    XLCD_ENPIN = 0;
    XLCDDelay4ms(); 
    XLCD_DATAPORT       &= 0xf0;    // Clear lower port
    XLCD_DATAPORT   |= 0b00000011;  // Function set cmd(4-bit interface)
    XLCD_ENPIN = 1;                 // Clock the cmd in
    XLCD_Delay500ns();   
    XLCD_ENPIN = 0;
    
    XLCDDelay4ms();    
    #ifdef XLCD_UPPER               // Upper nibble interface
    XLCD_DATAPORT   &= 0x0f;        // Clear upper port
    XLCD_DATAPORT   |= 0b00100000;
    #else                           // Lower nibble interface
    XLCD_DATAPORT       &= 0xf0;    // Clear lower port
    XLCD_DATAPORT   |= 0b00000010;  // Function set cmd(4-bit interface)
    #endif

    XLCD_ENPIN = 1;                 // Clock the cmd in
    XLCD_Delay500ns();   
    XLCD_ENPIN = 0;

//-----------------------------------------------------------------------
//function set command "0 0 1 DL N F X X"
//-----------------------------------------------------------------------
        #ifdef XLCD_FONT5x8
        XLCDCommand(0b00101000);    //if 2Line 5x8
        #else
        XLCDCommand(0b00101100);    //if 2Line 5x10
        #endif
    XLCDCommand(0b00001000);        //display off
    XLCDCommand(0b00000001);        //display clear
/////////////////////////////////////////////////////////////////////////////////
//Entry mode setting
////////////////////////////////////////////////////////////////////////////////
//Entry mode command " 0 0 0 0 0 1 ID S "
//ID =0 no cursor increment during read and write
//ID =1 cursor increment during read and write
//S =0 no display during read and write
//S =1 display shift 
       
#ifdef XLCD_CURSOR_INCREMENT
    #ifdef XLCD_DISPLAY_SHIFT
    XLCDCommand(0b00000111);    //if cursor inc and display shift
    #endif
    #ifdef XLCD_DISPLAY_NOSHIFT
    XLCDCommand(0b00000110);    //if cursor inc and no display shift
    #endif
#endif

#ifdef  XLCD_CURSOR_NOINCREMENT
    #ifdef XLCD_DISPLAY_SHIFT
    XLCDCommand(0b00000101);    //if no cursor increment, but with display shift
    #endif
    #ifdef XLCD_DISPLAY_NOSHIFT
    XLCDCommand(0b00000100);    //if no cursor increment, and no display shift
    #endif
#endif 
///////////////////////////////////////////////////////////////////////////////////
//Display on off ,Blink ,cursor command set 
// ///////////////////////////////////////////////////////////////////////////////
//"0 0 0 0 1 D C B "
//D=1 dislay on, C=1 cursor on, B=1 blink on

#ifdef  XLCD_DISPLAYON
    #ifdef XLCD_CURSORON
        #ifdef XLCD_BLINKON
        XLCDCommand(0b00001111);    //display on cursor on blink on
        #else
        XLCDCommand(0b00001110);    //display on cursor on blink off
        #endif
    #endif    
   
    #ifdef XLCD_CURSOROFF
        #ifdef XLCD_BLINKON
        //XLCDCommand(0b00001001);    //display on cursor off blink on
        #else
        XLCDCommand(0b00001100);    // display on cursor off blink off
        #endif
    #endif
#endif


#ifdef  XLCD_DISPLAYOFF
    XLCDCommand(0b00001000);        //display off
#endif            
_vXLCDreg=0;
// end of initialization
      return;
}

/*********************************************************************
 * Function         : void XLCDCommand(unsigned char cmd)
 * PreCondition     : None
 * Input            : cmd - Command to be set to LCD.
 * Output           : None
 * Side Effects     : None
 * Overview         : None
 * Note             : None
 ********************************************************************/
void XLCDCommand(unsigned char cmd)
{
if(_vXLCDreg==1)                            //if  called from XLCDinit routine is always Blocking
{
    #ifdef  XLCD_DELAYMODE
    XLCDDelay();
    #endif
    #ifdef  XLCD_READBFMODE
    XLCDIsBusy();
    #endif
}


if(_vXLCDreg==0)                            //if not called from XLCDinit routine
{                                           //if NON Block the user need to call XLCDIsBusy 
#ifdef  XLCD_BLOCK                          //and check the w reg status to check if the
    #ifdef  XLCD_DELAYMODE                  //module is free
    XLCDDelay();
    #endif
    #ifdef  XLCD_READBFMODE
    XLCDIsBusy();
    #endif
#endif
}

XLCD_RSPIN=0;
XLCD_ENPIN=0;
#ifndef XLCD_RW_GROUND
XLCD_RWPIN=0;
#endif


        XLCD_DATAPORT &=0xF0;               //clear port
        XLCD_DATAPORT |=((cmd>>4)&0x0f);
        XLCD_ENPIN = 1;                     // Clock the cmd in
        XLCD_Delay500ns();
        XLCD_ENPIN = 0;
        
        XLCD_DATAPORT &= 0xF0;              //clear port
        XLCD_DATAPORT |= cmd&0x0f ;	        //shift left 4 times
        XLCD_ENPIN = 1;
        XLCD_Delay500ns();
        XLCD_ENPIN = 0;
	
    return;
}
/*********************************************************************
 * Function         :XLCDPut()
 * PreCondition     :None
 * Input            :cmd - Command to be set to LCD.
 * Output           :None
 * Side Effects     :None
 * Overview         :None
 * Note             :None
 ********************************************************************/
void XLCDPut(char data)
{
#ifdef  XLCD_BLOCK
    #ifdef  XLCD_DELAYMODE
    XLCDDelay();
    #endif
    #ifdef  XLCD_READBFMODE
    XLCDIsBusy();
    #endif
#endif


#ifndef XLCD_RW_GROUND
XLCD_RWPIN=0;
#endif
XLCD_RSPIN=1;
XLCD_ENPIN=0;
#ifdef XLCD_8BIT
    XLCD_DATAPORT=data;
    XLCD_ENPIN = 1;
    XLCD_Delay500ns();
    XLCD_ENPIN = 0;
#endif
#ifdef XLCD_4BIT
    #ifdef XLCD_UPPER
        XLCD_DATAPORT &=0x0f;               //clear port
        XLCD_DATAPORT |= data&0xf0;         //write upper nibble to port
        XLCD_ENPIN = 1;                     // Clock the cmd in
        XLCD_Delay500ns();
        XLCD_ENPIN = 0;
        
        XLCD_DATAPORT &= 0x0f;              //clear port
        XLCD_DATAPORT |= (data<<4)&0xf0;	//shift left 4 times
        XLCD_ENPIN = 1;
        XLCD_Delay500ns();
        XLCD_ENPIN = 0;
    #endif
    #ifdef XLCD_LOWER
        XLCD_DATAPORT &=0xF0;               //clear port
        XLCD_DATAPORT |=((data>>4)&0x0f);
        XLCD_ENPIN = 1;                     // Clock the cmd in
        XLCD_Delay500ns();
        XLCD_ENPIN = 0;
        
        XLCD_DATAPORT &= 0xF0;              //clear port
        XLCD_DATAPORT |= data&0x0f ;	    //shift left 4 times
        XLCD_ENPIN = 1;
        XLCD_Delay500ns();
        XLCD_ENPIN = 0;
    #endif  



#endif

   return;
}



#ifndef XLCD_RW_GROUND    //need not compile any read command if RWpin grounded

/*********************************************************************
 * Function         :char XLCDIsBusy(void)
 * PreCondition     :None
 * Input            :None
 * Output           :non-zero if LCD controller is ready to accept new
 *                   data or commandzero otherwise.
 * Side Effects     :None
 * Overview         :None
 * Note             :None
 ********************************************************************/
char XLCDIsBusy(void)
{
XLCD_RSPIN=0;
XLCD_RWPIN=1;
XLCD_ENPIN=0;

#ifdef XLCD_8BIT
    XLCD_DATAPORT_TRIS=0xFF;                //make port input    
    XLCD_DATAPORT=0;      
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    
    if(_vXLCDreg==1)                        //will execute only if  called from XLCDInit
    { 
    while(XLCD_DATAPORT&0x80);
    XLCD_ENPIN=0;
    XLCD_DATAPORT_TRIS=0x00;                //make port output
    return;
    }  
      
    #ifdef  XLCD_BLOCK  
    if(_vXLCDreg==0)                        // will execute only if not called from XLCDInit
    {    
    while(XLCD_DATAPORT&0x80);
    XLCD_ENPIN=0;
    XLCD_DATAPORT_TRIS=0x00;                //make port input
    return;
    }
    #endif
    
    
    #ifdef  XLCD_NONBLOCK
    if(_vXLCDreg==0)                        //will execute only if not called from XLCDInit
    {
        if(XLCD_DATAPORT&0x80)              //Read bit 7 (busy bit)
        {   
        XLCD_ENPIN=0;                   
        XLCD_DATAPORT_TRIS=0x00;            //make port op
        return 1;                           //Return TRUE
        }       
        else                                
        {
        XLCD_ENPIN=0;
        XLCD_DATAPORT_TRIS=0x00;            //make port op
        return 0;                           //Return FALSE
        }      
    }
    #endif           
#endif           


#ifdef XLCD_4BIT 
    
    #ifdef  XLCD_UPPER   
    XLCD_DATAPORT_TRIS|=0xF0;               //make upper port input
    XLCD_DATAPORT   &=0x0F;
    XLCD_ENPIN=1;
    XLCD_Delay500ns();    
    if(_vXLCDreg==1)                        // will execute only if  called from XLCDInit
    { 
        while(XLCD_DATAPORT&0x80);
        XLCD_ENPIN=0;
        XLCD_Delay500ns();
        XLCD_ENPIN=1;
        XLCD_Delay500ns();
        XLCD_ENPIN=0;
        XLCD_DATAPORT_TRIS&=0x0F;           //make upper port output
        return;
    } 
    
    #ifdef  XLCD_BLOCK    
    if(_vXLCDreg==0)                        // When not called from the XLCDInit
    { 
        while(XLCD_DATAPORT&0x80);
        XLCD_ENPIN=0;
        XLCD_Delay500ns();
        XLCD_ENPIN=1;
        XLCD_Delay500ns();
        XLCD_ENPIN=0;
        XLCD_DATAPORT_TRIS&=0x0F;           //make upper port output
        return;
    }    
    #endif

    #ifdef  XLCD_NONBLOCK    
    if(_vXLCDreg==0)                        // will execute only if not called from XLCDInit
    {
    if(XLCD_DATAPORT&0x80)                  
        {  
        XLCD_ENPIN=0;                                 
        XLCD_Delay500ns();
        XLCD_ENPIN=1;
        XLCD_Delay500ns();
        XLCD_ENPIN=0;
        XLCD_DATAPORT_TRIS&=0x0F;           //make port output
        return 1;                           //Return TRUE
        }       
    else                                
        {
        XLCD_ENPIN=0; 
        XLCD_Delay500ns();                  // If high
        XLCD_ENPIN=1;
        XLCD_Delay500ns();
        XLCD_ENPIN=0;
        XLCD_DATAPORT_TRIS&=0x0F;           //make port input
        return 0;                           // Return FALSE
        }      

    }
    #endif     
    #endif      
    
    
    #ifdef  XLCD_LOWER
    XLCD_DATAPORT_TRIS|=0x0F;               //make lower port input
    XLCD_DATAPORT   &=0xF0; 
    XLCD_ENPIN=1;
    XLCD_Delay500ns();    
    if(_vXLCDreg==1)                        // will execute only if  called from XLCDInit
        { 
        while(XLCD_DATAPORT&0x08);
        XLCD_ENPIN=0;
        XLCD_Delay500ns();
        XLCD_ENPIN=1;
        XLCD_Delay500ns();
        XLCD_ENPIN=0;
        XLCD_DATAPORT_TRIS&=0xF0;           //make port output
        return;
        }      
    #ifdef XLCD_BLOCK 
    if(_vXLCDreg==0)                        //will execute only if  called from XLCDInit
        { 
        while(XLCD_DATAPORT&0x08);
        XLCD_ENPIN=0;
        XLCD_Delay500ns();
        XLCD_ENPIN=1;
        XLCD_Delay500ns();
        XLCD_ENPIN=0;
        XLCD_DATAPORT_TRIS&=0xF0;           //make port output
        return;
        } 
    #endif
  
    #ifdef XLCD_NONBLOCK 
    if(_vXLCDreg==0)                        // will execute only if  called from XLCDInit
        { 
            if(XLCD_DATAPORT&0x08)                  
            {  
            XLCD_ENPIN=0;                               
            XLCD_Delay500ns();
            XLCD_ENPIN=1;
            XLCD_Delay500ns();
            XLCD_ENPIN=0;
            XLCD_DATAPORT_TRIS=0x00;        //make port input
            return 1;                       // Return TRUE
            }        
        else                                
            {
            XLCD_ENPIN=0; 
            XLCD_Delay500ns();              
            XLCD_ENPIN=1;
            XLCD_Delay500ns();
            XLCD_ENPIN=0;
            XLCD_DATAPORT_TRIS=0x00;        //make port input
            return 0;                       // Return FALSE
            }      

        } 
    #endif
    #endif 
#endif 

}

/*********************************************************************
 * Function         :unsigned char XLCDGetAddr(void)
 * PreCondition     :None
 * Input            :None
 * Output           :Current address byte from LCD
 * Side Effects     :None
 * Overview         :None
 * Note             :The address is read from the character generator
 *                   RAM or display RAM depending on current setup.
 ********************************************************************/

unsigned char XLCDGetAddr(void)

{
char addr =0;
#ifdef  XLCD_BLOCK
    #ifdef  XLCD_DELAYMODE
    XLCDDelay();
    #endif
    #ifdef  XLCD_READBFMODE
    XLCDIsBusy();
    #endif
#endif

XLCD_RSPIN=0;
XLCD_RWPIN=1;
XLCD_ENPIN=0;

#ifdef XLCD_8BIT
    XLCD_DATAPORT_TRIS=0xFF;                //make port input        
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    addr=XLCD_DATAPORT;  
    XLCD_ENPIN=0;
    XLCD_DATAPORT_TRIS=0x00;                //make port input
    return(addr&0x7F);     
#endif        
#ifdef XLCD_4BIT    
    #ifdef  XLCD_UPPER 
    XLCD_DATAPORT_TRIS|=0xF0;               //make upper port input      
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    addr=XLCD_DATAPORT&0xF0;
    XLCD_ENPIN=0;;
    XLCD_Delay500ns();   
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    addr|=(XLCD_DATAPORT>>4)&0x0F;
    XLCD_ENPIN=0;    
    XLCD_DATAPORT_TRIS&=0x0F;               //make upper port output
    return(addr&0x7F);
    #endif  
          
    #ifdef  XLCD_LOWER
    XLCD_DATAPORT_TRIS|=0x0F;               //make lower port input  
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    addr=(XLCD_DATAPORT<<4)&0xF0;
    XLCD_ENPIN=0;    
    XLCD_Delay500ns();
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    addr|=XLCD_DATAPORT&0x0F;
    XLCD_ENPIN=0;    
    XLCD_DATAPORT_TRIS&=0xF0;               //make port output
    return(addr&0x7F);
    #endif  
 
#endif  

}


/*********************************************************************
 * Function         :char XLCDGet(void)
 * PreCondition     :None    
 * Input            :None
 * Output           :Current data byte from LCD
 * Side Effects     :None
 * Overview         :None
 * Note             :The data is read from the character generator
 *                   RAM or display RAM depending on current setup.
 ********************************************************************/
char XLCDGet(void)
{
    char data=0;
#ifdef  XLCD_BLOCK
    #ifdef  XLCD_DELAYMODE
    XLCDDelay();
    #endif
    #ifdef  XLCD_READBFMODE
    XLCDIsBusy();
    #endif
#endif
    XLCD_RSPIN=1;
    XLCD_RWPIN=1;
    XLCD_ENPIN=0;

#ifdef  XLCD_8BIT
    XLCD_DATAPORT_TRIS=0xFF;
    XLCD_ENPIN=1;
    XLCD_Delay500ns(); 
    data=XLCD_DATAPORT;
    XLCD_ENPIN=0;
    XLCD_DATAPORT_TRIS=0x00;
    return(data);
#endif
#ifdef  XLCD_4BIT
    #ifdef  XLCD_UPPER
    XLCD_DATAPORT_TRIS |=0xf0;              //make upper input     
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    data = XLCD_DATAPORT&0xf0;              //Read the upper nibble of data
    XLCD_ENPIN=0;
    XLCD_Delay500ns();
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    data |= ((XLCD_DATAPORT>>4)&0x0f);      //Read the upper nibble of data
    XLCD_ENPIN=0;
    XLCD_DATAPORT_TRIS &=0x0f;              //make output
    return(data);
    #endif
    
    #ifdef  XLCD_LOWER
    XLCD_DATAPORT_TRIS |=0x0F;              //make  input     
    XLCD_ENPIN=1;
    XLCD_Delay500ns();
    data = (XLCD_DATAPORT<<4)&0xf0;         //Read the upper nibble of data
    XLCD_ENPIN=0;
    XLCD_Delay500ns();
    XLCD_ENPIN=1;
    XLCD_Delay500ns();;
    data |= XLCD_DATAPORT&0x0f;             //Read the upper nibble of data
    XLCD_ENPIN=0;
    XLCD_DATAPORT_TRIS &=0xf0;              //make output
    return(data);
    #endif
#endif    
    

}
	
#endif		//end of #ifndef XLCD_RW_GROUND(all read commands)



/*********************************************************************
 * Function         :XLCDPutRomString(rom char *string)
 * PreCondition     :None    
 * Input            :None
 * Output           :Displays string in Program memory
 * Side Effects     :None
 * Overview         :None
 * Note             :is lways blocking till the string is written fully
 ********************************************************************/

void XLCDPutRomString(rom char *string)
{
     while(*string)                         // Write data to LCD up to null
    {    
        #ifdef  XLCD_NONBLOCK
        while(XLCDIsBusy());
        #endif 
        XLCDPut(*string);                   // Write character to LCD
        string++;                           // Increment buffer
    }
    return;
}
/*********************************************************************
 * Function         :XLCDPutRomString(rom char *string)
 * PreCondition     :None    
 * Input            :None
 * Output           :Displays string in Program memory
 * Side Effects     :None
 * Overview         :None
 * Note             :is lways blocking till the string is written fully
 ********************************************************************/
void XLCDPutRamString(char *string)
{
    while(*string)                          // Write data to LCD up to null
    {       
        #ifdef  XLCD_NONBLOCK
        while(XLCDIsBusy());
        #endif
               
        XLCDPut(*string);                   // Write character to LCD
        string++;                           // Increment buffer
    }
    return;
}

// Timing Functions
//   Note: If you ever want to use a frequency that is NOT 4 MHz you will need to go into the LCD Module.c file (this file)
//   and modify these functions (found below).  They are hard coded to cause specific time delays and expect a 4 MHz
//   clock signal.  They are easy to change but if you aren't at 4 MHz they won't do want they say.

/*The user  is require to write this 15 milli second delay in his routine,this delay */
/*is required as it is used in XLCDInit() which is used to initialize the LCD module*/
void XLCDDelay15ms (void)
{
    // Want 15ms using 4 MHz
    // 15 000 instructions
    Delay1KTCYx(15);
}

/*The user  is require to write this 4 milli second delay in his routine,this  delay */
/*is required as it is used in XLCDInit() which is used to initialize the LCD module*/
void XLCDDelay4ms (void)
{
    // Want 4ms using 4 MHz
    // 4 000 instructions
    Delay1KTCYx(4);
}

/*The user  is require to write this 500 nano second in his routine  this  delay */
/*is required as it is used in all read and write commaands in the XLCD routines*/
void XLCD_Delay500ns(void)
{
    Nop();
    Nop();
    Nop();
}

/*The user  is require to write this XLCDDelay() in his routine,this is required to write if */
/*the mode selected is by delay , it is used in all XLCD read and write commands of this routine */
void XLCDDelay(void)
{
	// We are using the blocking delay mode (it's the easiest so we need this)
    // Want at least 2 ms using 4 MHz (DSF picked 2 ms, longer is safer)
    // 2 000 instructions
    Delay1KTCYx(2);
}


