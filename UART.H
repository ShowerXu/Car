


unsigned int txData;                        // UART internal variable for TX
volatile unsigned char bSendCharEnd;


void UART_TX_init(void);
void UART_TX_char(unsigned char byte);
void UART_TX_print(unsigned char *string);
void Uart_TX_Int(void);
void int2str(unsigned char *s,int temp_data);
void int2StrB(unsigned char *str,int nb);
