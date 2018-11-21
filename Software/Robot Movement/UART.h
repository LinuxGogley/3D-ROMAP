// UART.h
// Runs on TM4C123

//------------UARTx_Init------------
// Initialize the UART1 for 9600 baud rate (assuming 80 MHz clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART0_Init(void);
void UART1_Init(void);

//------------UARTx_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
unsigned char UART0_InChar(void);
unsigned char UART0_InChar_B(void);
unsigned char UART1_InChar(void);
unsigned char UART1_InChar_B(void);

//------------UARTx_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART0_OutChar(unsigned char data);
void UART1_OutChar(unsigned char data);

//------------UARTx_OutDec------------
// Output 32-bit decimal to UART output
// Input: data is a 32-bit decimal value to be transferred
// Output: none
void UART0_OutDec(unsigned long data);
void UART1_OutDec(unsigned long data);

//------------UARTx_OutString------------
// Output String (NULL termination)
// Input: pointer to a NULL-terminated string to be transferred
// Output: none
void UART0_OutString(char str[]);
void UART1_OutString(char str[]);

//------------UARTx_InString------------
// Takes ASCII characters from input and assigns them to a char array
// Input: pointer to empty buffer, size of buffer
// Output: Null terminated string
void UART1_InString(char *bufPt);
void UART0_InString(char *bufPt);


