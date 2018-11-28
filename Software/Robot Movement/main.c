// main.c
// Runs on TM4C
// Robot car that is controlled via bluetooth.
// WASD commands control movement
// Dedicated buttons to control speed
// Mark Munoz
// November 20, 2018

// 1. Pre-processor Directives Section
// include header files that contain 
// function prototypes and constant definitions
#include "PLL.h"
#include "UART.h"
#include "PWM.h"
#include "String.h"
#include "tm4c123gh6pm.h"


// 2. Misc. Function Prototypes
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void PortF_Init(void);
	
	
// 3. Main loop
int main(void){
	const unsigned long period = 2000;  // (40MHz PLL clk / 2000) = 20KHz PWM freq
	unsigned long dutyA, dutyB, dutyMax;
	char mode [6];            // controls action based on bluetooth input
	PLL_Init();               // PLL w/ 40Mhz bus clk
	UART1_Init();             // HC05 UART 57600 baud, 8O1 
	PortF_Init();             // debug LEDs
	Direction_Init();         // Init PD0 and PD1 as motor direction signals, PD2 as output enable
    GPIO_PORTD_DATA_R |= 0x04;// enables motors output
	PWM0A_Init(period, 6);    // initialize PB4 as motor 1 PWM(initially off)
	PWM0B_Init(period, 6);    // initialize PB5 as motor 2 PWM(initially off)
	dutyMax = period/2;       // initially 50% speed

	//**************Configure bluetooth Commands*************
	//UART1_OutString("AT+NAME=3D-ROMAP\r\n");     // set custom name	
    //UART1_OutString("AT+PSWD=0159\r\n");         // set custom password
	//UART1_OutString("AT+UART=57600,0,1\r\n");    // 57600, 8O1

	while(1){
		UART1_InString(mode);         // receive updated mode from bluetooth
		
		if(strncmp(mode, "w", 1) == 0){  
			GPIO_PORTF_DATA_R |=  0x02; // RED LED
			GPIO_PORTD_DATA_R &= ~0x03; // ensure motors going forward
			dutyA = dutyB = dutyMax;    // give motors max speed
		}
		
		else if(strncmp(mode, "a", 1) == 0){  
			GPIO_PORTF_DATA_R |= 0x04;  // BLUE LED
			dutyA = period-dutyMax;     // keep one motor turning
			dutyB = dutyMax;            // reverse motor PWM
			GPIO_PORTD_DATA_R |= 0x02;  // motor A going in reverse
			GPIO_PORTD_DATA_R &=~0x01;  // ensure motor B going forward
		}
		
		else if(strncmp(mode, "s", 1) == 0){    
			GPIO_PORTF_DATA_R |= 0x08;  // GEEN LED
			dutyA = period - dutyMax;   // reverse motors PWM
			dutyB = period - dutyMax;   // reverse motors PWM
			GPIO_PORTD_DATA_R |= 0x03;  // reverse motors direction
		}
		
		else if(strncmp(mode, "d", 1) == 0){  // ASCII 'd' is received
			GPIO_PORTF_DATA_R |= 0x0E;  // WHITE LED
			dutyB = period-dutyMax;     // give motorB max speed
			dutyA = dutyMax;            // stop other motor
			GPIO_PORTD_DATA_R |= 0x01;  // motor B going reverse
			GPIO_PORTD_DATA_R &=~0x02;  // motor A going forward
		}
		
		else if(strncmp(mode, "50%", 1) == 0){
			GPIO_PORTF_DATA_R |= 0x0A;  // debug LED
			dutyMax = .5*period;        // 50% max speed
		}
		
		else if(strncmp(mode, "75%", 1) == 0){
			GPIO_PORTF_DATA_R |= 0x0C;  // YELLOW LED
			dutyMax = .75*period;       // 75% max speed
		}
		
		else if(strncmp(mode, "100%", 1) == 0){
			GPIO_PORTF_DATA_R |= 0x02;  // RED LED
			dutyMax = period - 2;       // 100% max speed
		}
		else if(strncmp(mode, "q", 1) == 0){ 
			GPIO_PORTD_DATA_R &= ~0x03; // set both motors forward & disable output
			dutyA = dutyB = 6;          // set pwm to 0
			GPIO_PORTF_DATA_R &= ~0x0E; // clear debug LEDs
		}
		
		PWM0A_Duty(dutyA*.666666667);    // update motor PWMs
		PWM0B_Duty(dutyB*.666666667);	// run at max 12v (2/3 of 18v)
	}
}

// 4. Subroutines Section
// DEBUG LEDs
void PortF_Init(void){ 
  // GPIO assignments
  volatile unsigned long delay;     // dummy instruction variable
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // dummy instruction to give setup enough time   
  GPIO_PORTF_LOCK_R  |=  0x4C4F434B;// 2) unlock PortF PF0  
  GPIO_PORTF_CR_R    |=  0x0E;      // allow changes to PF4-PF1     
  GPIO_PORTF_AMSEL_R &= ~0x0E;      // 3) disable analog function
  GPIO_PORTF_PCTL_R  &= ~0x0000FFF0;// 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R   |=  0x0E;      //    PF3-PF1 is output
  GPIO_PORTF_AFSEL_R &= ~0x0E;      // 6) no alternate function on PF4-PF1
  GPIO_PORTF_PUR_R   &= ~0x0E;      //    disable pullup resistors for LED
  GPIO_PORTF_DEN_R   |=  0x0E;      // 7) enable digital pins PF4-PF1 
  }
