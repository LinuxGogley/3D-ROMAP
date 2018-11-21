// PWM.c
// Initializes hardware PWM to control motors
//
#include "PWM.h"
#include "tm4c123gh6pm.h"

	// Output on PB4/M0PWM0
void PWM0A_Init(unsigned long period, unsigned long duty){
	// Initializing PWM on PB4/M0PWM2
	SYSCTL_RCGCPWM_R |= 0x01;		// 1) Activate PWM0
	SYSCTL_RCGCGPIO_R |= 0x02;	// 2) Activate Port B
	while((SYSCTL_PRGPIO_R&0x02) == 0){};
	GPIO_PORTB_AFSEL_R |= 0x10;    // Enable Alt Funct on PB4 
	GPIO_PORTB_PCTL_R &= ~0x000F0000; // Configure PB4 as PWM0
	GPIO_PORTB_PCTL_R |=  0x00040000;
	GPIO_PORTB_AMSEL_R &= ~0x10;  // Disable analog function on PB4
	GPIO_PORTB_DEN_R |= 0x10;     // Enable digital I/O on PB4
	SYSCTL_RCC_R = 0x00100000 |   // 3) use PWM divider
			(SYSCTL_RCC_R & (~0x000E0000));   // configure for /2 divider
	PWM0_1_CTL_R = 0;             // 4) Reload down-count mode
	PWM0_1_GENA_R = 0xC8;					// Low on LOAD, high on CMPA down
	PWM0_1_LOAD_R = period - 1;   // 5) Cycles needed to count down to 0
	PWM0_1_CMPA_R = duty - 1;			// 6) Count value when output rises
	PWM0_1_CTL_R  |= 0x00000001;
	PWM0_ENABLE_R |= 0x00000004;		 // Enable PB4 - PWM2EN (Pg.1247)
}

// Output on PB5/M0PWM3
void PWM0B_Init(unsigned long period, unsigned long duty){
	volatile unsigned long delay;
	// Initializing PWM on PB5/M0PWM2
	SYSCTL_RCGCPWM_R |= 0x01;		// 1) Activate PWM0
	SYSCTL_RCGCGPIO_R |= 0x02;	// 2) Activate Port B
	delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating
	GPIO_PORTB_AFSEL_R |= 0x20;    // Enable Alt Funct on PB5 (0010 0000)
	GPIO_PORTB_PCTL_R &= ~0x00F00000; // Configure PB5 as PWM0
	GPIO_PORTB_PCTL_R |=  0x00400000;	// Pin Mux(4)	
	GPIO_PORTB_AMSEL_R &= ~0x20;  // Disable analog function on PB5
	GPIO_PORTB_DEN_R |= 0x20;     // Enable digital I/O on PB5
	SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
	SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
	SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
	PWM0_1_CTL_R = 0;             // 4) Reload down-count mode
	PWM0_1_GENB_R = (PWM_0_GENB_ACTCMPBD_ONE|PWM_0_GENB_ACTLOAD_ZERO);
	PWM0_1_LOAD_R = period - 1;   // 5) Cycles needed to count down to 0
	PWM0_1_CMPB_R = duty - 1;			// 6) Count value when output rises		
	PWM0_1_CTL_R  |= 0x00000001;   //7) start PWM0
	PWM0_ENABLE_R |= 0x00000008;   // enable PB5/M0PWM0
}

// change duty cycle of PB4
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0A_Duty(unsigned long duty){
	PWM0_1_CMPA_R = duty - 1;             // 6) count value when output rises
}


// change duty cycle of PB5
// duty is number of PWM clock cycles output is high 
void PWM0B_Duty(unsigned long duty){
	PWM0_1_CMPB_R = duty - 1;             // 6) count value when output rises
}

// Initialize PD0 and PD1 as motor direction signals	
// and PD2 as motor output enable
void Direction_Init(void) {
	// Port D Initialization
	SYSCTL_RCGC2_R     |=  0x08;       // Enable port D clock
  GPIO_PORTD_CR_R 	 |=  0x07;       // Enable changes to PD2-PD0
  GPIO_PORTD_AMSEL_R &= ~0x07;       // Disable analog PD2-PD0 
  GPIO_PORTD_PCTL_R  &= ~0x00000FFF; // Configure PD2-PD0 as GPIO 
  GPIO_PORTD_DIR_R 	 |=  0x07;       // Set PD2-PD0 as output
  GPIO_PORTD_AFSEL_R &= ~0x07;       // Disable alternate functionality  
  GPIO_PORTD_DEN_R 	 |=  0x07;       // Digital enable PD2-PD0
}

