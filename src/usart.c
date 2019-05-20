#include "usart.h"

void init_usart1()
{

	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable peripheral clocks. */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART1 Rx pin as floating input. */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Tx as alternate function push-pull. */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART2 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable transmit and receive interrupts for the USART2. */
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* Enable the USART1 IRQ in the NVIC module (so that the USART1 interrupt
	 * handler is enabled). */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the RS232 port. */
	USART_Cmd(USART1, ENABLE);

	queue = createQueue(10);
}
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//enter interrupt when STM32 receice data.
	{
		USART_Temp_Data = (uint16_t) USART_ReceiveData(USART1); //receive a char
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

	    enqueue(queue, USART_Temp_Data);
	}
}
void send_byte(uint8_t b)
{
  /* Send one byte */
  USART_SendData(USART1, b);

  /* Loop until USART2 DR register is empty */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}
void retarget_init()
{
  // Initialize UART
}

int _write (char *ptr, int len)
{
  /* Write "len" of char from "ptr" to file id "fd"
   * Return number of char written.
   * Need implementing with UART here. */

  for ( int i = 0; i<len ;i++)
  {
    send_byte(*ptr);
    ptr++;
  }
  return len;
}

void _ttywrch(int ch) {
  /* Write one char "ch" to the default console
   * Need implementing with UART here. */
  /* Send one byte */
  USART_SendData(USART1, ch);

  /* Loop until USART2 DR register is empty */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

int data_available() {
	return !isEmpty(queue);
}

uint8_t get_data() {
	return dequeue(queue);
}

void broadcast_reading(uint16_t tadc) {
	uint8_t msb, lsb, chk;

	lsb = tadc & 0x3F;
	msb = tadc >> 6;

	chk = msb ^ lsb;

	msb |= 0x00; // msb type
	lsb |= 0x40; // lsb type
	chk |= 0x80; // check type;

	send_byte(msb);

	int i =0;

	for(i = 0; i < 100; i++) {
		asm volatile("nop");
	}

	send_byte(lsb);

	for(i = 0; i < 100; i++) {
		asm volatile("nop");
	}

	send_byte(chk);
//	send_byte(chk);

//	for(i = 0; i < 100; i++) {
//		asm volatile("nop");
//	}
}
