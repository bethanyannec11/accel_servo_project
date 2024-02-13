
/****** ASEN 4/5067 Final Project ******************************************************
 * Author: Bethany-Anne Calvert
 * Date  : Dec. 19, 2023
 *
 * Updated for XC8
 * 
 * Description
 * On power up execute the following sequence:
 *      Initialization Routine:
 *          Ports
 *          EEPROM
 *          MSSP1
 *          Timers/Interrupts
 *      
 * The following then occurs forever:
 *      ISR's will update the following:
 *          Pitch and Velocity variable information from SPI accelerometer data
 *          Read EEPROM data to pull servo PWM settings
 *       Convert data to PWM values and actuate servos
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *   PWM();
 *
 * HiPriISR (included just to show structure)
 *   TMR7handler
 *
 * LoPriISR
 *   TMR3handler
 ******************************************************************************/

#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <pic18f87k22.h>


#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
unsigned char TMR3X = 0;            // Upper timing register
unsigned char TMR7X = 0;            // Upper timing register
unsigned char on = 0;               // LED toggle flag
int vel = 0;                        // rounding velocity integer
char vel_r = 0x000C;                // Velocity checking variable
float v2 = 0;                       // final velocity
float acc = 0 ;                     // acceleration from acc
char pitch = 0;                     // pitch angle holder from EEPROM data
char pitch_x_p = 0;                 // tells if pitch is positive
char pitch_x_m = 0;                 // tells if pitch is negative
float t = 1/12.5;                   // sampling time
float v1 = 0;                       // initial velocity
char CNTL1_on = 0b10001001;         // PC1 on; Low-Power; DRE off; 4g; TDTE off; res; TPE on
char CNTL1_off = 0b00001001;        // PC1 off; Low-Power; DRE off; 4g; TDTE off; res; TPE on
char CNTL1_w = 0x1B;                // CNTL1 write address
char trash = 0;                     // trash register for buffer read
char XOUT_L = 0x8A;                 // Address of Low byte of accelerometer x-accel
char XOUT_H = 0x89;                 // Address of High byte of accelerometer x-accel
char TSCP = 0x94;                   // Accelerometer angle orientation info
int X_L = 0;                        // Low byte of accelerometer x-accel
int X_H = 0;                        // High byte of accelerometer x-accel
int velocity_array[35];             // Array holding velocity data
int angle1_array[35];               // Array holding pitch 1 angle data
int angle2_array[35];               // Array holding pitch 2 angle data
int angle3_array[35];               // Array holding pitch 3 angle data
int angle4_array[35];               // Array holding pitch 4 angle data
int angle5_array[35];               // Array holding pitch 5 angle data
int angle6_array[35];               // Array holding pitch 1 angle data
char angle = 0;                     // Angle for demonstration
char pwm[2];                        // PWM vector
float time = 0;                     // PWM time before conversion
float p_time = .001;                // initial high cycle PWM time in s
float h_time = .019;                // initial low cycle PWM time in s
long int pwm_time = 0;              // integer/hex rep of high cycle PWM time
long int high_time = 0;             // integer/hex rep of low cycle PWM time
int count = 0;                    // counter for demonstration

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR3handler(void);     // Interrupt handler for TMR3 accel. update
void TMR7handler(void);     // Interrupt handler for TMR5 servo update
void SPI(void);             // Routine to extract SPI data
void Data_Load(void);       // Load data values into arrays
void EEPROM_setup(void);    // Storing data in EEPROM
void EEPROM_write(void);    // Writing to EEPROM
void EEPROM_read(void);     // Reading from EEPROM
void PWM(void);             // Change the PWM of the servos

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
      while(1) {
          PWM();                // Udpate PWM vals
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * 
 ******************************************************************************/
void Initial() {
    // Configure the IO ports
    TRISE = 0;                      // CS/RE2????
    LATE = 0;                       
    TRISC = 0b10010000;             // SDO/RC5 out; SDI/RC4 in; SCK/RC3 out, CS/RC2 out
    LATC = 0;                       // RC1-0 Servos out
    
    
    // Configure MSSP1
    SSP1STAT = 0b11000000;          // master mode, trans from idle to active, clr SSP1BUF
    SSP1CON1 = 0b00100000;          // Enable serial port, idle clock is low, Master Fosc/4
    
    // Configure EEPROM
    Data_Load();
    EEPROM_setup();            // Load EEPROM with data
    
    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    
    // Initializing TMR3 & interrupts
    T3CON = 0b00000110;             // 16-bit, Fosc / 4, no pre/post scale timer
    TMR3 = 64405;                   // Initialize TMR3 registers
    PIR2bits.TMR3IF = 0;            // Clear TMR1 interrupts
    PIE2bits.TMR3IE = 1;            // Enable TMR1 interrupts
    IPR2bits.TMR3IP = 0;            // Assign high priority
    
    
    // Initializing TMR7
    p_time = p_time/(250*pow(10,-9));       // high duty cycle, 1ms
    h_time = h_time/(250*pow(10,-9));       // low duty cycle 19ms
    pwm_time = 65536-(int)p_time;           // convert for timer
    high_time = 65536-((int)h_time-65536);  //convert for timer
    T7CON = 0b00000110;             // 16-bit, Fosc / 4, no pre/post scale timer
    TMR7 = pwm_time;                // Initialize TMR7 registers
    PIR5bits.TMR7IF = 0;            // Clear TMR7 interrupts
    PIE5bits.TMR7IE = 1;            // Enable TMR7 interrupts
    IPR5bits.TMR7IP = 1;            // Assign low priority


//   // Turn off/initialize accelerometer CNTL register
    LATEbits.LATE2 = 0;             // Drive CS low for output
    SSP1BUF = CNTL1_w;              // write CNTL1 write address
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    trash = SSP1BUF;                // Read buffer before writing again
    __delay_us(10);
    SSP1BUF = CNTL1_off;             // write settings to buffer COULD POSSIBLY NEED TO JUST SET FIRST BIT FIRST
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    trash = SSP1BUF;                // Read buffer before writing again
    LATEbits.LATE2 = 1;         
    __delay_us(10);
    
    // Turn on accelerometer
    LATEbits.LATE2 = 0;             // Drive CS low for output
    SSP1BUF = CNTL1_w;              // write CNTL1 write address
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    trash = SSP1BUF;                // Read buffer before writing again
    __delay_us(10);
    SSP1BUF = CNTL1_on;             // write settings to buffer COULD POSSIBLY NEED TO JUST SET FIRST BIT FIRST
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    trash = SSP1BUF;                // Read buffer before writing again
    LATEbits.LATE2 = 1;             // Drive CS low for output 

    
    // Turn on timers and LEDs
    PORTEbits.RE5 = 1;             // Turning on RC0
    PORTEbits.RE6 = 1;             // Turning on RC1
    T3CONbits.TMR3ON = 1;           // Turning on TMR3
    T7CONbits.TMR7ON = 1;           // Turning on TMR7
    on = 1;
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    while(1) {
        if( PIR5bits.TMR7IF ) {
            TMR7handler();
            continue;
        }
        
        // Save temp copies of WREG, STATUS and BSR if needed.
        break;      // Supports RETFIE automatically
    }
}	// Supports retfie FAST automatically



/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until TMR0IF is clear.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) 
{
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
        if( PIR2bits.TMR3IF ) {
            TMR3handler();
            continue;
        }
        
        // Save temp copies of WREG, STATUS and BSR if needed.
        break;      // Supports RETFIE automatically
    }
}



/******************************************************************************
 * PWM subroutine.
 *
 * Loads data from EEPROM to actuate servos
 ******************************************************************************/
void PWM() {
    unsigned char j;
    unsigned char k;
    if(count < 100){
        angle = -5;                  // assign randomly
        vel_r = 18;                   // assign randomly
    }
    if(count > 99){
        angle = 9;                  // assign randomly
        vel_r = 2;                   // assign randomly
    }
    
    
        //check angle 1 and vel
        EEADR = 0x22;
        EEADRH = 0x0;
        EEPROM_read();
        pitch = EEDATA; 
        
        if(angle == pitch){
            for(j=1;j<35;j=j+2){
                EEADR = (char)0x00+j;
                EEADRH = 0x0;
                EEPROM_read();
                vel = EEDATA; 
                
                if(vel_r == vel){
                    for(k=0;k<2;k++){
                        EEADR = (char)0x22+j+k;
                        EEADRH = 0x0;
                        EEPROM_read();
                        pwm[k] = EEDATA;  
                    }
                    break;
                }
            }    
        }
        
        //check angle 2 and vel
        EEADR = 0x45;
        EEADRH = 0x0;
        EEPROM_read();
        pitch = EEDATA;
        
        if(angle == pitch){
            for(j=1;j<35;j=j+2){
                EEADR = (char)0x00+j;
                EEADRH = 0x0;
                EEPROM_read();
                vel = EEDATA; 
                
                if(vel_r == vel){
                    for(k=0;k<2;k++){
                        EEADR = (char)0x45+j+k;
                        EEADRH = 0x0;
                        EEPROM_read();
                        pwm[k] = EEDATA; 
                    }
                    break;
                }
            }    
        }
        
        //check angle 3 and vel
        EEADR = 0x68;
        EEADRH = 0x0;
        EEPROM_read();
        pitch = EEDATA;
        
        if(angle == pitch){
            for(j=1;j<35;j=j+2){
                EEADR = (char)0x00+j;
                EEADRH = 0x0;
                EEPROM_read();
                vel = EEDATA; 
                
                if(vel_r == vel){
                    for(k=0;k<2;k++){
                        EEADR = (char)0x68+j+k;
                        EEADRH = 0x0;
                        EEPROM_read();
                        pwm[k] = EEDATA; 
                    }
                    break;
                }
            }    
        }
        
        //check angle 4 and vel
        EEADR = 0x8B;
        EEADRH = 0x0;
        EEPROM_read();
        pitch = EEDATA;
        
        if(angle == pitch){
            for(j=1;j<35;j=j+2){
                EEADR = (char)0x00+j;
                EEADRH = 0x0;
                EEPROM_read();
                vel = EEDATA; 
                
                if(vel_r == vel){
                    for(k=0;k<2;k++){
                        EEADR = (char)0x8B+j+k;
                        EEADRH = 0x0;
                        EEPROM_read();
                        pwm[k] = EEDATA;  
                    }
                    break;
                }
            }    
        }
        
        //check angle 5 and vel
        EEADR = 0x00;
        EEADRH = 0x1;
        EEPROM_read();
        pitch = EEDATA;
        
        if(angle == pitch){
            for(j=1;j<35;j=j+2){
                EEADR = (char)0x00+j;
                EEADRH = 0x0;
                EEPROM_read();
                vel = EEDATA; 
                
                if(vel_r == vel){
                    for(k=0;k<2;k++){
                        EEADR = (char)0x00+j+k;
                        EEADRH = 0x1;
                        EEPROM_read();
                        pwm[k] = EEDATA;    
                    }
                     break;
                }
            }    
        }
        
        //check angle 2 and vel
        EEADR = 0x22;
        EEADRH = 0x1;
        EEPROM_read();
        pitch = EEDATA;
        
        if(angle == pitch){
            for(j=1;j<35;j=j+2){
                EEADR = (char)0x00+j;
                EEADRH = 0x0;
                EEPROM_read();
                vel = EEDATA; 
                
                if(vel_r == vel){
                    for(k=0;k<2;k++){
                        EEADR = (char)0x22+j+k;
                        EEADRH = 0x1;
                        EEPROM_read();
                        pwm[k] = EEDATA;    
                    }
                     break;
                }
            }    
        }
    
        
    // Reset pwm timing
    float pwm_int = (float)((pwm[0] << 8) | pwm[1]);       // combine upper and lower bytes
    time = pwm_int*pow(10,-7);
    p_time = time/(250*pow(10,-9));
    h_time = (.02f-time)/(250*pow(10,-9));
    pwm_time = 65536-(int)p_time;
    high_time = 65536-((int)h_time-(int)65536);
    // Set PWM for servos
    
}
/******************************************************************************
 * TMR7handler interrupt service routine.
 *
 * Updates TMRX register, clears interrupt flag, and activates SPI data
 ******************************************************************************/
void TMR7handler() {
    if(on==1) {                  // Enter if on
        LATEbits.LATE5 = 0;      // Toggle RC1
        LATEbits.LATE6 = 0;      // Toggle RC0
        TMR7X++;                // Increment higher time register
        if(TMR7X==2){
        TMR7 = high_time;       // Set timer for low duty cycle
        TMR7X = 0;              // Reset TMR7X
        on = 0;                 // clear on
        }
        }
        
    else {                                 // RC1/RC2 off, so start on time
        TMR7 = pwm_time;                   // Set timer for high duty cycle
        LATEbits.LATE5 = 1;                // Toggle RC1
        LATEbits.LATE6 = 1;                // Toggle RC0
        on = 1;                            // Set on
        if(count < 100){
            count++;
        }
        if(count > 99){
            count++;
        }
        if(count == 200){
            count = 0;
        }
            
        
    }
    
    PIR5bits.TMR7IF = 0;                   // Clear flag 

    
}  

/******************************************************************************
 * TMR3handler interrupt service routine.
 *
 * Updates TMRX register, clears interrupt flag, and activates SPI data
 ******************************************************************************/
void TMR3handler() {
    PIR2bits.TMR3IF = 0;
    TMR3X++;
    if(TMR3X==7){
       TMR3X = 0;       
       TMR3 = 0xFB95;
       SPI();
    }
    
}
/******************************************************************************
 * SPI subroutine
 *
 * Saves the forward velocity and pitch angle to a variable at a rate of 10Hz
 ******************************************************************************/
void SPI() {
    // Read upper byte of accelerometer data
    LATEbits.LATE2 = 0;             // Drive CS low for output
    SSP1BUF = XOUT_H;               // write CNTL1 write address
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    trash = SSP1BUF;                // Read buffer before writing again
    __delay_us(1);
    SSP1BUF = 0x88;                 // write something to buffer so you can read
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    X_H = SSP1BUF;                  // Read high byte of accel data
    LATEbits.LATE2 = 1;             // Drive CS low for output
    
    // Read lower byte of accelerometer data
    LATEbits.LATE2 = 0;             // Drive CS low for output
    SSP1BUF = XOUT_L;               // write CNTL1 write address
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    trash = SSP1BUF;                // Read buffer before writing again
    __delay_us(1);
    SSP1BUF = 0x88;                 // write something to buffer so you can read
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    X_L = SSP1BUF;                  // Read high byte of accel data
    LATEbits.LATE2 = 1;             // Drive CS low for output
    
    
    // Read orientation change
    LATEbits.LATE2 = 0;             // Drive CS low for output
    SSP1BUF = TSCP;                 // write CNTL1 write address
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    trash = SSP1BUF;                // Read buffer before writing again
    __delay_us(1);
    SSP1BUF = 0x88;                 // write something to buffer so you can read
    while(SSP1STATbits.BF==0){};    // Wait for buffer flag to set
    pitch = SSP1BUF;                // Read high byte of accel data
    LATEbits.LATE2 = 1;             // Drive CS low for output

    // Convert to binary and store variables
    int acc_int = (X_H << 8) | X_L;            // combine upper and lower bytes
    acc = (float)acc_int;                      // recast to float
    v2 = acc*t+v1;                             // get velocity
    v1 = v2;                                   // store velocity for next meas.
    vel = (int)v2;
    
    pitch_x_m = (pitch & (1 << 3));            // pitching down if 1
    pitch_x_p = (pitch & (1 << 2));            // pitching up if 1
    
    
}
/******************************************************************************
 * Data_Load subroutine.
 *
 * Loads EEPROM with predetermined servo positioning data
 ******************************************************************************/
void Data_Load() {
    
    // velocity array
    velocity_array[0] = 0;
    velocity_array[1] = 2;
    velocity_array[2] = 0;
    velocity_array[3] = 3;
    velocity_array[4] = 0;
    velocity_array[5] = 4;
    velocity_array[6] = 0;
    velocity_array[7] = 5;
    velocity_array[8] = 0;
    velocity_array[9] = 6;
    velocity_array[10] = 0;
    velocity_array[11] = 7;
    velocity_array[12] = 0;
    velocity_array[13] = 8;
    velocity_array[14] = 0;
    velocity_array[15] = 9;
    velocity_array[16] = 0;
    velocity_array[17] = 0x0A;
    velocity_array[18] = 0x00;
    velocity_array[19] = 0x0B;        //11430
    velocity_array[20] = 0;
    velocity_array[21] = 0x0C;        //11287
    velocity_array[22] = 0x00;
    velocity_array[23] = 13;        //11144
    velocity_array[24] = 0x00;
    velocity_array[25] = 14;        //11001
    velocity_array[26] = 0x00;
    velocity_array[27] = 0xF;        //10858
    velocity_array[28] = 0x00;
    velocity_array[29] = 16;        //10715
    velocity_array[30] = 0x00;
    velocity_array[31] = 17;        //10572
    velocity_array[32] = 0x00;
    velocity_array[33] = 18;        //10286
    velocity_array[34] = 0x00;
    
    // all of the following pwm times are *10^-7s
    // first angle data
    angle1_array[0] = -5;
    angle1_array[1] = 0x32;         //12860
    angle1_array[2] = 0x3C;
    angle1_array[3] = 0x31;         //12717
    angle1_array[4] = 0xAD;
    angle1_array[5] = 0x31;         //12574
    angle1_array[6] = 0x1E;
    angle1_array[7] = 0x30;         //12431
    angle1_array[8] = 0x8F;
    angle1_array[9] = 0x2F;         //12145
    angle1_array[10] = 0x71;
    angle1_array[11] = 0x2E;        //11859
    angle1_array[12] = 0x53;
    angle1_array[13] = 0x2E;        //12002
    angle1_array[14] = 0xE2;
    angle1_array[15] = 0x2D;        //11716 
    angle1_array[16] = 0xC4;
    angle1_array[17] = 0x2D;        //11573
    angle1_array[18] = 0x35;
    angle1_array[19] = 0x2C;        //11430
    angle1_array[20] = 0xA6;
    angle1_array[21] = 0x2C;        //11287
    angle1_array[22] = 0x17;
    angle1_array[23] = 0x2B;        //11144
    angle1_array[24] = 0x88;
    angle1_array[25] = 0x2A;        //11001
    angle1_array[26] = 0xF9;
    angle1_array[27] = 0x2A;        //10858
    angle1_array[28] = 0x6A;
    angle1_array[29] = 0x29;        //10715
    angle1_array[30] = 0xDB;
    angle1_array[31] = 0x29;        //10572
    angle1_array[32] = 0x4C;
    angle1_array[33] = 0x28;        //10286
    angle1_array[34] = 0x2E;
    

    // second angle data
    angle2_array[0] = 0;
    angle2_array[1] = 0x38;         //14433
    angle2_array[2] = 0x61;
    angle2_array[3] = 0x38;         //14576
    angle2_array[4] = 0xF0;
    angle2_array[5] = 0x39;         //14719
    angle2_array[6] = 0x7F;
    angle2_array[7] = 0x39;         //14719
    angle2_array[8] = 0x7F;
    angle2_array[9] = 0x38;         //14576
    angle2_array[10] = 0xF0;
    angle2_array[11] = 0x38;        //14433
    angle2_array[12] = 0x61;
    angle2_array[13] = 0x37;        //14290
    angle2_array[14] = 0xD2;
    angle2_array[15] = 0x36;        //14004
    angle2_array[16] = 0xB4;
    angle2_array[17] = 0x35;        //13718
    angle2_array[18] = 0x96;
    angle2_array[19] = 0x35;        //13575
    angle2_array[20] = 0x07;
    angle2_array[21] = 0x34;        //13432
    angle2_array[22] = 0x78;
    angle2_array[23] = 0x33;        //13289
    angle2_array[24] = 0xE9;
    angle2_array[25] = 0x33;        //13146
    angle2_array[26] = 0x5A;
    angle2_array[27] = 0x32;        //13003
    angle2_array[28] = 0xCB;
    angle2_array[29] = 0x33;        //13146
    angle2_array[30] = 0x5A;
    angle2_array[31] = 0x32;        //13003
    angle2_array[32] = 0xCB;
    angle2_array[33] = 0x32;        //12860
    angle2_array[34] = 0x3C;
    
    
    // third angle data
    angle3_array[0] = 3;
    angle3_array[1] = 0x3A;         //14862
    angle3_array[2] = 0x0E;
    angle3_array[3] = 0x3A;         //15005
    angle3_array[4] = 0x9D;
    angle3_array[5] = 0x3B;         //15148
    angle3_array[6] = 0x2C;
    angle3_array[7] = 0x3B;         //15291
    angle3_array[8] = 0xBB;
    angle3_array[9] = 0x3C;         //15434
    angle3_array[10] = 0x4A;
    angle3_array[11] = 0x3C;        //15577
    angle3_array[12] = 0xD9;
    angle3_array[13] = 0x3D;        //15720
    angle3_array[14] = 0x68;
    angle3_array[15] = 0x3D;        //15863
    angle3_array[16] = 0xF7;
    angle3_array[17] = 0x3E;        //16006
    angle3_array[18] = 0x86;
    angle3_array[19] = 0x3F;        //16149
    angle3_array[20] = 0x15;
    angle3_array[21] = 0x3E;        //16006
    angle3_array[22] = 0x86;
    angle3_array[23] = 0x3D;        //15863
    angle3_array[24] = 0xF7;
    angle3_array[25] = 0x3F;        //16149
    angle3_array[26] = 0x15;
    angle3_array[27] = 0x3E;        //16006
    angle3_array[28] = 0x86;
    angle3_array[29] = 0x3F;        //16292
    angle3_array[30] = 0xA4;
    angle3_array[31] = 0x40;        //16435
    angle3_array[32] = 0x33;
    angle3_array[33] = 0x40;        //16578
    angle3_array[34] = 0xC2;
    
    // fourth angle data
    angle4_array[0] = 5;        
    angle4_array[1] = 0x41;         //16864
    angle4_array[2] = 0xE0;
    angle4_array[3] = 0x42;         //17007
    angle4_array[4] = 0xFE;
    angle4_array[5] = 0x42;         //17150
    angle4_array[6] = 0xFE;
    angle4_array[7] = 0x43;         //17293
    angle4_array[8] = 0x8D;
    angle4_array[9] = 0x44;         //17436
    angle4_array[10] = 0x1C;
    angle4_array[11] = 0x44;        //17579
    angle4_array[12] = 0xAB;
    angle4_array[13] = 0x45;        //17722
    angle4_array[14] = 0x3A;
    angle4_array[15] = 0x45;        //17865
    angle4_array[16] = 0xC9;
    angle4_array[17] = 0x45;        //17722
    angle4_array[18] = 0x3A;
    angle4_array[19] = 0x44;        //17436
    angle4_array[20] = 0x1C;
    angle4_array[21] = 0x43;        //17293
    angle4_array[22] = 0x8D;
    angle4_array[23] = 0x42;        //17150
    angle4_array[24] = 0xFE;
    angle4_array[25] = 0x42;        //17150
    angle4_array[26] = 0xFE;
    angle4_array[27] = 0x42;        //17007
    angle4_array[28] = 0x6F;
    angle4_array[29] = 0x43;        //17293
    angle4_array[30] = 0x8D;
    angle4_array[31] = 0x44;        //17436
    angle4_array[32] = 0x1C;
    angle4_array[33] = 0x45;        //17722
    angle4_array[34] = 0x3A;
    
    // fifth angle data
    angle5_array[0] = 9;
    angle5_array[1] = 0x45;         //17722
    angle5_array[2] = 0x3A;
    angle5_array[3] = 0x45;         //17865
    angle5_array[4] = 0xC9;
    angle5_array[5] = 0x46;         //18008
    angle5_array[6] = 0x58;
    angle5_array[7] = 0x46;         //18151
    angle5_array[8] = 0xE7;
    angle5_array[9] = 0x47;         //18294
    angle5_array[10] = 0x76;
    angle5_array[11] = 0x46;        //18151
    angle5_array[12] = 0xE7;
    angle5_array[13] = 0x46;        //18008
    angle5_array[14] = 0x58;
    angle5_array[15] = 0x46;        //18008
    angle5_array[16] = 0x58;
    angle5_array[17] = 0x45;        //17865
    angle5_array[18] = 0xC9;
    angle5_array[19] = 0x45;        //17722
    angle5_array[20] = 0x3A;
    angle5_array[21] = 0x45;        //17865
    angle5_array[22] = 0xC9;
    angle5_array[23] = 0x45;        //17865
    angle5_array[24] = 0xC9;
    angle5_array[25] = 0x46;        //18008
    angle5_array[26] = 0x58;
    angle5_array[27] = 0x46;        //18151
    angle5_array[28] = 0xE7;
    angle5_array[29] = 0x46;        //18151
    angle5_array[30] = 0xE7;
    angle5_array[31] = 0x47;        //18294
    angle5_array[32] = 0x76;
    angle5_array[33] = 0x47;        //18294
    angle5_array[34] = 0x76;
    
    // sixth angle data
    angle6_array[0] = 12;
    angle6_array[1] = 0x4D;         //19724
    angle6_array[2] = 0x0C;
    angle6_array[3] = 0x4D;         //19867
    angle6_array[4] = 0xA4; 
    angle6_array[5] = 0x4E;         //20000
    angle6_array[6] = 0x20;
    angle6_array[7] = 0x4C;         //19581
    angle6_array[8] = 0x7D;
    angle6_array[9] = 0x4B;         //19295
    angle6_array[10] = 0x5F;
    angle6_array[11] = 0x4C;        //19581
    angle6_array[12] = 0x7D;
    angle6_array[13] = 0x4B;        //19295
    angle6_array[14] = 0x5F;
    angle6_array[15] = 0x4B;        //19438
    angle6_array[16] = 0xEE;
    angle6_array[17] = 0x4B;        //19295
    angle6_array[18] = 0x5F;
    angle6_array[19] = 0x4B;        //19438
    angle6_array[20] = 0xEE;
    angle6_array[21] = 0x4B;        //19438
    angle6_array[22] = 0xEE;
    angle6_array[23] = 0x4B;        //19438
    angle6_array[24] = 0xEE;
    angle6_array[25] = 0x4C;        //19581
    angle6_array[26] = 0x7D;
    angle6_array[27] = 0x4C;        //19581
    angle6_array[28] = 0x7D;
    angle6_array[29] = 0x4D;        //19724
    angle6_array[30] = 0x0C;
    angle6_array[31] = 0x4D;        //19867
    angle6_array[32] = 0xA4;
    angle6_array[33] = 0x4D;        //19724
    angle6_array[34] = 0x0C;
}

/******************************************************************************
 * EEPROM_setup subroutine.
 *
 * Loads EEPROM with predetermined servo positioning data
 ******************************************************************************/
void EEPROM_setup() {
    char i;
    // velocity data
    
    for(i=0;i<35;i++){
        EEADR = 0+i;
        EEADRH = 0x00;
        EEDATA = (char)velocity_array[i];
        EEPROM_write();
    }
    
    // first angle data
    for(i=0;i<35;i++){
        EEADR = 34+i;
        EEADRH = 0x0;
        EEDATA = (char)angle1_array[i];
        EEPROM_write();
    }
    
    // second angle data
    for(i=0;i<35;i++){
        EEADR = (char)0x45+i;
        EEADRH = 0x0;
        EEDATA = (char)angle2_array[i];
        EEPROM_write();
    }
    
    // third angle data
    for(i=0;i<35;i++){
        EEADR = (char)0x68+i;
        EEADRH = 0x0;
        EEDATA = (char)angle3_array[i];
        EEPROM_write();
    }
    
    // fourth angle data
    for(i=0;i<35;i++){
        EEADR = (char)0x8B+i;
        EEADRH = 0x0;
        EEDATA = (char)angle4_array[i];
        EEPROM_write();
    }
    
    // fifth angle data
    for(i=0;i<35;i++){
        EEADR = (char)0x00+i;
        EEADRH = 0x1;
        EEDATA = (char)angle5_array[i];
        EEPROM_write();
    }
    
    // sixth angle data
    for(i=0;i<35;i++){
        EEADR = (char)0x22+i;
        EEADRH = 0x1;
        EEDATA = (char)angle6_array[i];
        EEPROM_write();
    }
}
/******************************************************************************
 * EEPROM_write subroutine.
 *
 * Enables write of EEPROM data
 ******************************************************************************/
void EEPROM_write(){
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    while(EECON1bits.WR){};
    INTCONbits.GIE = 1;
    EECON1bits.WREN = 0;
}
/******************************************************************************
 * EEPROM_read subroutine.
 *
 * Enables read of EEPROM data
 ******************************************************************************/
void EEPROM_read(){
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;
    __delay_us(10);
    
}