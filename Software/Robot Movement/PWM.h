// PWM.h
// Runs on TM4C123
// Hardware PWM on 

// period is 16-bit number of PWM clock cycles in one period (3<=period)
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0A_Init(unsigned long period, unsigned long duty);
void PWM0B_Init(unsigned long period, unsigned long duty);

// change duty cycle of motors
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0A_Duty(unsigned long duty);
void PWM0B_Duty(unsigned long duty);

// allow for motor change direction
void Direction_Init(void);

// LED initialization
void PortE_Init(void);
