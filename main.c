/* 
    main.c 

    neo.xiong@freescale.com

    IOUART Driver example code
*/

#include "board.h"
#include "fsl_iouart_driver.h"


static iouart_state_t iouartStatePtr;

const uint8_t WelcomString[] = "\n\r++++++++++++++++ Welcome to IOUART get started Example +++++++++++++++++\n\r";
uint8_t aaa;
uint8_t bbb;

void OnDataReceived(uint8_t d)
{
    d = d;
}

int main(void)
{
    hardware_init();

    const iopuart_user_config_t iouartConfig = {
        .txPinName       = GPIO_MAKE_PIN(PORTB_IDX, 1),
        .rxPinName       = GPIO_MAKE_PIN(PORTB_IDX, 2),
        .baudRate        = BOARD_DEBUG_UART_BAUD
    };

    IOUART_DRV_Init(&iouartStatePtr, &iouartConfig, &OnDataReceived);

    IOUART_DRV_SendData(WelcomString, sizeof(WelcomString));
    
    while (1);
}
/* test line */
