/*
 * File:   newmain.c
 * Author:  Markel Biain & Nathan Ewnetu 
 *              21723801 & 21483214
 * Description: Type your initial to save your run, high score run will be saved on to EEprom. 
 *              Aim of the game is to avoid cactus which is moving across the screen. Your character
 *              can switch from top to bottom by a press of a button. Game difficulty can be increased
 *              and decreased at the start of every run via the potentiometer. 
 * Created on 01 December 2023, 13:51
 */

#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <stdbool.h>
#include <xc.h>
#include <stdio.h>      // Include Standard I/O header file
#include <stdlib.h>     // Include rand() function
#include "ee302lcd.h"	// Include LCD header file. This file must be in same
#include "I2C_EE302.h"  // Include I2C header file

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 4000000
#endif 
#define CLOSED 0		// Define switch action "Closed" as 0
#define Button RB0		// Assign Label Button to PortB bit 0 (RB0) 
#define POT RA0         // Define the potentiometer pin


// globals _____________________________________________________
unsigned char gOutString[16];	// character array for easier print
unsigned int gCactusX[2] = {17, 25};//gCactusX[0] for cactus0 and gCactusX[1] for cactus1
unsigned int gCactusY[2] = {1, 2};  //The y position of the cacti
unsigned int gVolts;                //The voltage values of potentiometer
unsigned int gHighscore = 0;        //High score of game 
unsigned int gScore = 0;            //Score obtained, score given if a your char passes a cacti
unsigned short gSpeed = 100000;     //speed of game
char sNameInitail;                  //char to hold initial of player
bool charTop = true;                //characters position, true if on top line
bool gPlaying = false;              //if game has started set true

    
// Prototypes_____________________________________________________________
void setup(void);                   // Declare setup function
void checkButton(void);             // Declare data to LCD function
void lcdTitle(void);                // Declare title to LCD function
void clear_outString(void);         // Declare outString clear function
void endGame(void);                 // Declare end of game function
void initCustomCharacters(void);    // Declare function to initialize the custom characters
void resetCactus(int);              // Declare function to reset cactus position, int to define which cactus (0 or 1)
void menu(void);                    // Declare function to print game over tittle when die in game
void startScreen(void);             // Declare function to display game tittle
void writeStringEeprom(void);       // Declare function to write to EEPROM
char readStringEeprom(void);        // Declare function to read to EEPROM
void rec(void);                     // Declare function to read letter from PC via UART
void pRead(void);                   // Declare function to read voltage from potentiometer
void option(void);                  // Declare function to choose game difficulty


//Main program
void main(void)
{
	setup();                //Call initialisation
    startScreen();          //Game tittle fro 2s
    __delay_ms(2000);
    Lcd8_Clear();
    option();               // Choose game difficulty
	for(;;){                // Superloop
        if(gPlaying){       // Check if in game currently
            checkButton();  // Constantly checking for button input
        }else{
            menu();         // Waiting screen, displays highest score and gives option to change difficulty
        }
	}
}

void menu(){
    if(gScore>gHighscore){      //Check if highs core has been beaten
        gHighscore = gScore;    //Change high score
        rec();                  //Receive players initial
        writeStringEeprom();    //Save players initial
        
    }
    endGame();                  //Display endgame tittle for 3s
    __delay_ms(3000);
    Lcd8_Clear;
    option();                   //Choose difficulty
    while (Button != CLOSED){}  //Wait for input
    gPlaying = true;            //Restart game score and timer
    gScore = 0;
    TMR1 = 50000;               //Reset game to interrupt timer to 2 Hz
}

void setup(void)
	{
    Lcd8_Init();    //Initialize LCD
    i2c_init();     //Initialize I2C to communicate with EEPPROM
    TRISA = 0x01;
    TRISC = 0xD8;   //RC6 and RC7 must be set to inputs for USART. RC3 and RC4 for SCL and SDA as inputs
    TXSTA = 0x24;   //Set TXEN bit to enable transmit.
                    //Set BRGH bit for Baud Rate table selection.
    RCSTA = 0x90;   //Set CREN bit for continuous read.
                    //Set SPEN bit to enable serial port.
    SPBRG = 0x19;   //Set Baud Rate to 9600
    TRISD = 0x00;   // Port D bits 0 and 1 as outputs
    PORTD = 0xff;   // Initialize PORTD to 0
	TRISB=0x01;		// Set PORTB bit 0, 1 and 2 as inputs
    T1CON = 0x21;   // Timer1 settings: 16-bit, internal clock, prescaler 1:4
    TMR1 = gSpeed;  // Load Timer1 to generate a 1Hz interrupt (for 4MHz Fosc)
    
    ADCON0 = 0b01000001; // Channel AN0 selected, ADC module is enabled
    ADCON1 = 0b01000010;
    
    GIE = 1;   // Enable global interrupts
    PEIE = 1;  // Enable peripheral interrupts
    TMR1IE = 1;// Enable Timer1 interrupt
}

void option(void){
    while (Button != CLOSED){   //Loop until button pressed 
        pRead();                //Read ADC input to choose difficulty
        Lcd8_Set_Cursor(1,0);   //Print current option
        Lcd8_Write_String("Difficulty:");
        Lcd8_Set_Cursor(2,0);
        if(gVolts<100){         //Print difficulty option according to ADC input
            Lcd8_Write_String("Hard");
            gSpeed = 65000;
        }else if(gVolts<200){
            Lcd8_Write_String("Medium ");
            gSpeed = 50000;
        }else{
            Lcd8_Write_String("Easy");
            gSpeed = 40000;
        }
        __delay_ms(100);
        Lcd8_Clear();
    }
    gPlaying = true;            //Set game to play mode
    Lcd8_Clear();               //Print player character
    Lcd8_Set_Cursor(2, 0);
    Lcd8_Write_Char(0);
}

void checkButton(){
    if (Button == CLOSED){  //Constantly check if button pressed and if so toggle character position
        __delay_ms(50);
        if(charTop){    //Toggle character position
            Lcd8_Set_Cursor(1, 0);
            charTop = false;
        }
        else{
            Lcd8_Set_Cursor(2, 0);
            charTop = true;
        }
        Lcd8_Write_Char(0);  // Display characterDino1
        while (Button == CLOSED){}  //Do nothing while button is pressed
        __delay_ms(50);
    }
}

void resetCactus(int cactus){ //true if cactus1 false if cactus 2
    int x  = rand()%16+16;  //set random position
    int y = rand()%2+1;
    gCactusX[cactus] =  x;  //Move cactus to the random position
    gCactusY[cactus] = y;
}

void clear_outString(void)  // clear out string
{
    for (int i=0; i<16; i++)
    {
        gOutString[i] = 0x00;   //clear character 
    }
}

void endGame(void){
    Lcd8_Clear();                                               //clear LCD display
    clear_outString();                                          //clear outString array
    sprintf(gOutString,"Game Over");                            //define string as "Button press"
    Lcd8_Write_String(gOutString);                              //print string to LCD
    __delay_ms(1000);
    Lcd8_Clear();
    sNameInitail = readStringEeprom();
    Lcd8_Set_Cursor(1,1);                                       //select line 2 of LCD
    sprintf(gOutString,"HS: %c %d",sNameInitail,gHighscore);	//define count as a char in outString
    Lcd8_Write_String(gOutString);                              //print string to LCD
    
    Lcd8_Set_Cursor(2,1);                                       //select line 2 of LCD
    sprintf(gOutString,"Your Score: %d",gScore);                //define count as a char in outString
    Lcd8_Write_String(gOutString);                              //print string to LCD
 
}

// charcter standing upright
const char characterManNormal[8] = {
  0b00100,
  0b01110,
  0b01110,
  0b00100,
  0b11111,
  0b00100,
  0b01010,
  0b10001
};
// cactus character
const char characterCactus[8]={
  0b01110,
  0b01111,
  0b11111,
  0b11110,
  0b01111,
  0b11111,
  0b11110,
  0b01110,
};

void initCustomCharacters(void){    //Initialize the 2 custom characters
    Lcd8_Cmd(0x40);
    for(int i = 0; i<8; i++){
        Lcd8_Write_Char(characterManNormal[i]);
    }
    for(int i = 0; i<8; i++){
        Lcd8_Write_Char(characterCactus[i]);
    }
}

void lcdTitle(void){
    Lcd8_Set_Cursor(1,6);
	Lcd8_Write_String("Game");		// print "Game" on line 1 of LCD
	Lcd8_Set_Cursor(2,6);				// select line 2
	Lcd8_Write_String("EE302");		// print "EE302" on line 2 of LCD
}

void startScreen(void){
    initCustomCharacters(); // Call initialization of custom characters
	lcdTitle();             // Call LCDTitle

}

void __interrupt() isr(void) {  //move cactus to the left every interrupt
    if (TMR1IF) {
        TMR1IF = 0;             //Reset timer flag
        TMR1 = gSpeed;          //Declare speed of game depending on what player chose
        if (gPlaying) {         //If game being played move cacti one position to left
            Lcd8_Clear();
            for (int i = 0; i < 2; i++) {
                Lcd8_Set_Cursor(gCactusY[i], gCactusX[i]--);
                Lcd8_Write_Char(2);
                if (gCactusX[i] == 0) { //Check if cactus reached end 
                    gScore++;           //Add to score
                    if (gCactusY[i] - 1 == (int)charTop) {//If the cactus reaches the end check if on same space as player
                        gPlaying = false;                   //Set playing to true
                    }
                    resetCactus(i);                         //Reset cactus position
                }
            }
            if (charTop) {              //Print character on location it was on
                Lcd8_Set_Cursor(2, 0);
            } else {
                Lcd8_Set_Cursor(1, 0);
            }
            Lcd8_Write_Char(0);
        } else {
            TMR1 = 49945;               //Reset timer and do nothing if game not being played
        }
    }
}

void writeStringEeprom(void) //writes highscore name to eeprom
{
   
    i2c_start();         //start,
    i2c_write(0xA0);     //write control byte (A2, A1, A0 - LOW, R/W = 0)
    i2c_write(0x00); // write address of high byte 
    i2c_write(0xC0); // write address of low byte

    i2c_write(sNameInitail);    //write current character and increase pointer by 1 for next char
    
    i2c_stop();          //send stop condition
    __delay_ms(5);       //delay for write to propagatei2c_start();        
       
}
 
char readStringEeprom(void)
{
    i2c_start();				//Send Start Condition
    i2c_write(0xa0);			//Write Control Byte (A2,A1,A0 all low, R/W = 0)(Write)
    i2c_write(0x00);		//Write high byte of address 
    i2c_write(0xC0);		//Write low byte of address 
    i2c_repStart();				//Send reStart Condition
    i2c_write(0xa1);			//Write Control Byte (A2,A1,A0 all low, R/W = 1)(Read)

    sNameInitail =  i2c_read(0); //for last data byte, read data followed by NACK 
    
    i2c_stop();   
    
    return sNameInitail;
}

void rec(void){ //function to receive letter
    if(RCIF){   //check if something is in serial port
        sNameInitail = RCREG;   //reads from serial port
    }
}

void pRead(void)    //read value from ADC
{
    __delay_us(50);
    GO_nDONE=1;
    while(GO_nDONE)continue;
    gVolts = ADRESH;    //Save input from ADC
}

