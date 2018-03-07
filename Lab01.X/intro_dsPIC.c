/*
  This simple program makes writes the time to the screen (Secure CRT), makes 
  LEDs flash, and reads from a variable resistor 
  It uses timer1 and interrupts
*/

#include <p30f4011.h>

#include <libpic30.h>
#include <delay.h>
#include <adc10.h>
#include <pwm.h>
#include <uart.h>
#include <string.h>
#include <stdio.h>
#include <qei.h>
#include <timer.h>
#include <ports.h>

#define PERIOD 14739 // for 1000 Hz pwm frequency
// #define PERIOD 4605 // for 50 Hz frequency with 64 prescalar
// #define PERIOD 18424 // for 50 Hz frequency with 16 prescalar
#define MAX_DUTY 2*PERIOD
#define MAX_COUNT 1

// Configuration Bits
#pragma config FPR = FRC_PLL16   // 117.92 MHz
#pragma config FOS = PRI
#pragma config FCKSMEN = CSW_FSCM_OFF
#pragma config WDT = WDT_OFF
#pragma config FPWRT = PWRT_16
#pragma config BODENV = BORV27
#pragma config BOREN = PBOR_OFF
#pragma config MCLRE = MCLR_EN
#pragma config GWRP = GWRP_OFF

// define some variables

unsigned int AD_value;
unsigned int GO;

/***************************************************************/

// external input interrupt handler

void __attribute__((interrupt,no_auto_psv)) _INT1Interrupt( void )
{
  unsigned int dutycyclereg, dutycycle;
  char updatedisable;

  // turn off the pwm signal

  dutycycle = (unsigned int)0;
  dutycyclereg = 1;
  updatedisable = 0;
  SetDCMCPWM( dutycyclereg, dutycycle, updatedisable );  // duty cycle set to low
  dutycyclereg = 2;
  updatedisable = 0;
  SetDCMCPWM( dutycyclereg, dutycycle, updatedisable );  // duty cycle set to low
  
  // turn off the LED to indicate power if off

  LATFbits.LATF6 = 0;  // signal the power is off
  
  // Disable the interrupt

  DisableINT1;

  // now just wait

  while(1);
}

/************************************************************/

// Initialize external interrupt 1

void Init_INT1(void)
{
  unsigned int config;

  config = FALLING_EDGE_INT & // interrupt on a falling edge
           EXT_INT_ENABLE &        // enable the interrupts
           //EXT_INT_PRI_0 ;
           GLOBAL_INT_ENABLE;
            
  ConfigINT1( config );


   // turn on the LED to show interrupt is set

  TRISFbits.TRISF6 = 0; 
  LATFbits.LATF6 = 1;  // signal the interrupt is set

  // prepare for an input on RD0

  TRISDbits.TRISD0 = 1;

  // enable the interrupt

  DisableINT1;
  EnableINT1;

  return;
}

/**********************************************************/

// timer 1 interrupt handler

void __attribute__((interrupt,auto_psv)) _T1Interrupt( void )
{ 
  unsigned int ReadQEI( void );
  extern unsigned int AD_value;
  extern unsigned int GO;

 // read from the A/D channel

  ADCON1bits.SAMP = 1; // start the sampling
  while(!ADCON1bits.DONE);
  AD_value = ReadADC10(0);

  // reset Timer 1 interrupt flag 

  IFS0bits.T1IF = 0;

  // if GO is 1 we are not done before the next interrupt!
 /* 
  if(GO == 1)
   LATEbits.LATE1 = 1;
  */
  GO = 1;
}

/***********************************************************/ 

// Initialize timer 1

void Init_Timer1( unsigned int period )
{
  unsigned int config;

  config = T1_INT_PRIOR_4 & // set interrupt priority to 2
           T1_INT_ON;       // enable the interrupts
            
  ConfigIntTimer1( config );

  config =  T1_ON &  // turn on the timer
            T1_IDLE_CON & // operate during sleep
            T1_GATE_OFF & // timer gate accumulation is disabled
            T1_PS_1_256 &   // timer prescale is 256
            T1_SYNC_EXT_OFF & // don't synch with external clock
            T1_SOURCE_INT; // use the internal clock

  OpenTimer1( config, period );
  
  TRISEbits.TRISE1 = 0;  // prepare for the overrun LED indicator
  LATEbits.LATE1 = 0; // the LED should be off

  return;
}

/********************************************************/

// setup ADC10

  void adc_init(void){

  unsigned int config1, config2, config3, configport, configscan;

  ADCON1bits.ADON = 0 ; // turn off ADC

  SetChanADC10(
       ADC_CH0_NEG_SAMPLEA_VREFN & // negative reference for channel 0 is VREF negative
       ADC_CH0_POS_SAMPLEA_AN2    // 
      // ADC_CHX_NEG_SAMPLEA_VREFN &  // negative reference for channel 1 is VREF negative
      // ADC_CHX_POS_SAMPLEA_AN3AN4AN5
                 );

  ConfigIntADC10( ADC_INT_DISABLE ); // disable the interrupts

  config1 = 
       ADC_MODULE_ON &  //turn on ADC module
       ADC_IDLE_CONTINUE & // let it idle if not in use
       ADC_FORMAT_INTG &   // unsigned integer format
       ADC_CLK_AUTO  &  // manual trigger source
       ADC_AUTO_SAMPLING_OFF  & // do not continue sampling
       ADC_SAMPLE_SIMULTANEOUS & // sample both channels at the same time
       ADC_SAMP_ON; // enable sampling

  config2 = 
       ADC_VREF_AVDD_AVSS & // voltage reference
       ADC_SCAN_OFF & // don't scan
       ADC_CONVERT_CH0 & // convert channel 0
       ADC_SAMPLES_PER_INT_1 & // 1 samples per interrupt
       ADC_ALT_BUF_OFF & // don't use the alternate buffer
       ADC_ALT_INPUT_OFF; // don't use an alternate input

  config3 = 
       ADC_SAMPLE_TIME_2 & // auto sample time bits
       ADC_CONV_CLK_SYSTEM & // use the system clock
       ADC_CONV_CLK_13Tcy;  // conversion clock speed (coeff of TCY is ADCS)
       
  configport = 
       ENABLE_AN2_ANA;   // parameters to be configured in the ADPCFG 
          
  configscan = 
       SCAN_NONE; // scan select parameter for the ADCSSL register
  
  OpenADC10( config1, config2, config3, configport, configscan );
}

/*********************************************************/

// setup pwm

  void pwm_init(void){

  unsigned int config1, config2, config3;
  unsigned int sptime;

  config1 = PWM_INT_DIS &     // disable the interrupt
            PWM_FLTA_DIS_INT; // disable the interrupt on fault

  ConfigIntMCPWM( config1 );

  config1 = PWM_EN & //  enable the PWM module
            PWM_IPCLK_SCALE1 & // input prescaler set to 1
            PWM_OP_SCALE1 & // post scalar set to 1
            PWM_MOD_UPDN; // free running mode

  config2 = PWM_MOD1_IND & // pwm modules run independently
            PWM_MOD2_IND & 
            PWM_MOD3_IND & 
            PWM_PDIS1H & // disable 1 high
            PWM_PDIS2H & // disable 2 high
            PWM_PDIS3H & // enable 3 high
            PWM_PDIS1L & // disable 1 low
            PWM_PDIS2L & // disable 2 low
            PWM_PDIS3L ;  // disable 3 low

  config3 = PWM_UEN; // enable updates

  sptime = 0x0;

  OpenMCPWM( PERIOD, sptime, config1, config2, config3 );

}

/********************************************************/

// setup the UART

 void uart1_init(void) {
 
unsigned int config1, config2, ubrg;

config1 = UART_EN & // enable the UART
          UART_IDLE_CON & // set idle mode
          UART_DIS_WAKE & // disable wake-up on start
          UART_DIS_LOOPBACK & // disable loopback
          UART_DIS_ABAUD & // disable autobaud rate detect
          UART_NO_PAR_8BIT & // no parity, 8 bits
          UART_1STOPBIT;     // one stop bit

config2 = UART_INT_TX_BUF_EMPTY & // interrupt anytime a buffer is empty
          UART_TX_PIN_NORMAL & // set transmit break pin
          UART_TX_ENABLE & // enable UART transmission
          UART_INT_RX_CHAR & // receive interrupt mode selected
          UART_ADR_DETECT_DIS & // disable address detect
          UART_RX_OVERRUN_CLEAR; // overrun bit clear

ubrg = 15; // 115200 baud

OpenUART1( config1, config2, ubrg); 
}

/*****************************************************************/

int main ( void )
{
  extern unsigned int AD_value;
  extern unsigned int GO;
  unsigned int dutycyclereg, dutycycle, period;
  char updatedisable;
  double scale, time, dt;
  int i, on, count, int_time;
  double x, y, z;

 // set up the external interrupt
  
   Init_INT1();
  
  //  set up the A/D parameters

  adc_init();
 
  // set up the pwm

  pwm_init();

  // set up the uart

  uart1_init();

  // disable updates

  updatedisable = 0;
 
  // get some scaling out of the way

  scale = ((double) MAX_DUTY)/1023.0;

  // initialize timer1
  // dt can be no larger than 0.25 seconds

  dt = 0.1;  // the sampling interval in seconds

  // dt = N*256/29,480,000;  assuming a 256 prescaler.
  // so N = dt* 115156

  period = (unsigned int) (dt*115156.0);
 
  if (period > 32768 )
  {
     period = 32768;
  }
  printf("....period is %6u (should be < 32768) \n ", period);

  Init_Timer1( period );

  AD_value = 0;
  time = 0;
  on = 0;

 TRISEbits.TRISE1 = 0;  // output
 TRISEbits.TRISE3 = 0;
  
  count = MAX_COUNT;
  GO = 0;
  
/********************* MAIN LOOP **********************/

  while(1){

    while(!GO );
    time = time + dt;
  /*    
    for(i=1; i<2000; i++)
    {
        x = 10.0;
        y = 20.0;
        z = x*y;
    }
 */        
 
    if (on == 0)
    {
       LATEbits.LATE1 = 1;
       LATEbits.LATE3 = 1;
       on = 1;
    }
    else
    {
       LATEbits.LATE1 = 0;
       LATEbits.LATE3 = 0;
       on = 0;
    }
  /*
   dutycycle = (unsigned int) ((double) AD_value)*scale;
   dutycyclereg = 1;
   SetDCMCPWM( dutycyclereg, dutycycle, updatedisable);
   dutycycle = (unsigned int) ((double)(1023- AD_value))*scale;
   dutycyclereg = 2;
   SetDCMCPWM( dutycyclereg, dutycycle, updatedisable); 
  */
   if (--count == 0)
   {
       int_time = (int)(100.0*time);  // convert for printout
       printf("%8d %u \n", int_time, AD_value);
       count = MAX_COUNT ;
   }
   GO = 0;  // all done
  } 
 
}
