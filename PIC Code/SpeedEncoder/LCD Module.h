/*********************************************************************
* FileName:        LCD Module.h
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

#ifndef __LCD_MODULE_H
#define __LCD_MODULE_H
#include <delays.h>

// Primary initialization functions
void XLCDInit(void);                					// Initialise the LCD, must be done before using any other commands
#define XLCDClear()     			XLCDCommand(0x01)	// Clear LCD
#define XLCDL1home()    			XLCDCommand(0x80)	// Return to beginning of line 1
#define XLCDL2home()    			XLCDCommand(0xC0)	// Return to beginning of line 2

// Primary functions for writing to the LCD
void XLCDPut(char data);                            	// to display an ASCII character from an individual char variable
void XLCDPutRamString(char *string);                	// to display a data string from an array of chars

// Additional cursor related functions
#define XLCDCursorOnBlinkOn()        XLCDCommand(0x0F)	// the user may refer to the LCD data sheet
#define XLCDCursorOnBlinkOff()       XLCDCommand(0x0E)	// and generate more commands like these
#define XLCDDisplayOnCursorOff()     XLCDCommand(0x0C)  // if you want to go digging for more
#define XLCDDisplayOff()             XLCDCommand(0x08)
#define XLCDCursorMoveLeft()         XLCDCommand(0x10)
#define XLCDCursorMoveRight()        XLCDCommand(0x14)
#define XLCDDisplayMoveLeft()        XLCDCommand(0x18)
#define XLCDDisplayMoveRight()       XLCDCommand(0x1C)

// Additional internal commands that you don't need to use - You can stop reading this header file
void XLCDPutRomString(rom char *string);            	// to display a const data string (constant are directly typed text stored in ROM)
char XLCDIsBusy(void);                              //to check if the LCD Busy flag (used internally)
void XLCDCommand(unsigned char cmd);                //to send commands to LCD (used internally)          
unsigned char XLCDGetAddr(void);
char XLCDGet(void);
#define XLCDReturnHome() 			XLCDCommand(0x02)

// Timing Functions
//   Note: If you ever want to use a frequency that is NOT 4 MHz you will need to go into the LCD Module.c file
//   and modify these functions.  They are hard coded to cause specific time delays and expect a 4 MHz
//   clock signal.  They are easy to change but if you aren't at 4 MHz they won't do want they say.
void XLCDDelay15ms(void);
void XLCDDelay4ms(void);
void XLCDDelay100us(void);
void XLCD_Delay500ns(void);
void XLCDDelay(void);

// Setup the pin locations for PICDEM board - DSF 5/18/08 additions
#define XLCD_DATAPORT       PORTD
#define XLCD_DATAPORT_TRIS  TRISD
#define XLCD_RWPIN   		PORTDbits.RD5
#define XLCD_RWPIN_TRIS  	TRISDbits.TRISD5
#define XLCD_RSPIN   		PORTDbits.RD4
#define XLCD_RSPIN_TRIS  	TRISDbits.TRISD4
#define XLCD_ENPIN   		PORTDbits.RD6
#define XLCD_ENPIN_TRIS  	TRISDbits.TRISD6
#define LCD_PWR				PORTDbits.RD7				// Necessary for PICDEM 2 board

/* Setup mode for LCD - added by DSF 5/18/08 */
#define XLCD_4BIT 
#define    CLOCK_FREQ    .4
#define    XLCD_2LINE 
#define    XLCD_FONT5x8    
#define    XLCD_LOWER                      
#define    XLCD_BLOCK         
#define    XLCD_DELAYMODE         
#define    XLCD_DISPLAYON
#define    XLCD_CURSORON
#define    XLCD_BLINKON
#define    XLCD_CURSOR_INCREMENT
#define    XLCD_DISPLAY_NOSHIFT

#endif
