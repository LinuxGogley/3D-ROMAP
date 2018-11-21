// PLL.h
// Runs on LM4F120/TM4C123
// A software function to change the bus frequency using the PLL.

// The #define statement SYSDIV2 initializes
// the PLL to the desired frequency.
#define SYSDIV2 9 // bus frequency is 400MHz/(SYSDIV2+1) = 400MHz/(9+1) = 40 MHz

// configure the system to get its clock from the PLL
void PLL_Init(void);
