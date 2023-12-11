#include "UART.h"
#include "GPIO.h"
#include "lcd.h"
vunit16 data ;
void clock_init()
{
	RCC_GPIOA_CLK_EN() ;
	RCC_GPIOB_CLK_EN() ;
	RCC_AFIO_CLK_EN() ;
}
void receive_handler(void)
{
	MCAL_UART_ReceiveData(USART3, &data , disable) ;
	LCD_WRITE_CHAR(data) ;
}

int main(void)
{

	clock_init() ;
	LCD_INIT() ;
	UART_Config_t config ;
	config.BaudRte = UART_BaudRate_9600 ;
	config.HWFlowCTL = UART_FlowCTL_NONE ;
	config.IRQ_Enable = UART_IRQ_Enable_RXNEIE ;
	config.P_IRQ_CallBack = receive_handler ;
	config.LoadLength = UART_LoadLength_8B ;
	config.StopBits = UART_StopBits_1 ;
	config.USART_mode = UART_Mode_TX_RX ;
	config.parity = UART_Parity_NONE ;
	MCAL_UART_Init(USART3, &config) ;
	MCAL_UART_GPIO_Set_Pins(USART3) ;
	while(1)
	{

	}

}
