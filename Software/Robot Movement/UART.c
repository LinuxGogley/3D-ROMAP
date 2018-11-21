// UART.c
// Initialize UART0 w/ 9600 8N1
// Initialize UART1 w/ 57600 8O1
#include "UART.h"
#include "tm4c123gh6pm.h"

//------------UART_Init------------
// Initialize the UART for 9600 baud rate (assuming 40 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART0_Init(void){
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
	
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART while initializing 
  UART0_IBRD_R = 260;                   // IBRD = int(40,000,000 / (16 * 9,600)) = int(260.4166)
  UART0_FBRD_R = 27;                    // FBRD = int(0.41667 * 64 + 0.05) = 27
                                        // 8 bit word length (no parity, one stop bit, FIFOs enabled)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= UART_CTL_UARTEN;       // enable UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1-0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1-0
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA
}

void UART1_Init(void){
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1; // activate UART1

  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB; // activate port B
  UART1_CTL_R &= ~UART_CTL_UARTEN;      // disable UART while initializing 
  UART1_IBRD_R = 43;                    // IBRD = int(40,000,000 / (16 * 57600)) = int(43.4027777778) = 43
  UART1_FBRD_R = 25;                    // FBRD = int(0.4027777778 * 64 + 0.05) = 25
                                        // 8 bit word length (one stop bit, FIFOs enabled, ODD pairity)
  UART1_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN+UART_LCRH_PEN);
  UART1_CTL_R |= UART_CTL_UARTEN;       // enable UART
	GPIO_PORTB_AFSEL_R |= 0x03;           // enable alt funct on PB1-0
  GPIO_PORTB_DEN_R |= 0x03;             // enable digital I/O on PB1-0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTB_AMSEL_R &= ~0x03;          // disable analog functionality on PB
}

//------------UARTx_InChar------------
// Wait for new serial port input 
// Input: none
// Output: ASCII code for data received
unsigned char UART0_InChar(void){
  if((UART0_FR_R&UART_FR_RXFE) == 0){
		return((unsigned char)(UART0_DR_R&0xFF));
	}
	else{
		return 0;
	}
}

unsigned char UART0_InChar_B(void){
  while((UART0_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART0_DR_R&0xFF));
}

unsigned char UART1_InChar(void){
  if((UART1_FR_R&UART_FR_RXFE) == 0){
		return((unsigned char)(UART1_DR_R&0xFF));
	}
	else{
		return 0;
	}
}

unsigned char UART1_InChar_B(void){
  while((UART1_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART1_DR_R&0xFF));
}
//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART0_OutChar(unsigned char data){
  while((UART0_FR_R&UART_FR_TXFF) != 0);
  UART0_DR_R = data;
}
void UART1_OutChar(unsigned char data){
  while((UART1_FR_R&UART_FR_TXFF) != 0);
  UART1_DR_R = data;
}


//------------UART_OutString------------
// Outputs string to serial port
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART0_OutString(char str[]){
	unsigned int i = 0;
  while(str[i] != 0x0){
    UART0_OutChar(str[i]);
    i++;
  }
}

void UART1_OutString(char str[]){
	unsigned int i = 0;
  while(str[i] != 0x0){
    UART1_OutChar(str[i]);
    UART0_OutChar(str[i]);
    i++;
  }
}

//------------UARTx_InString------------
// Outputs string to serial port
// Input: received string
// Output: none
void UART1_InString(char *bufPt) {
	unsigned short max = 6;      // maximum length of input string
	int length=0;
	char character;
  character = UART1_InChar();
  while(character != 0x00){
		if(length < max){
      *bufPt = character;
      bufPt++;
      length++;
      //UART0_OutChar(character);// echo on serial terminal 
    }
    character = UART1_InChar();
  }
  *bufPt = 0;
}
