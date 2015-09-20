/* 
    fsl_iouart_driver.h 

    neo.xiong@freescale.com

    IOUART driver API and Data structure declartion file. 
*/

#ifndef __FSL_IOUART_DRIVER_H__
#define __FSL_IOUART_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "fsl_gpio_driver.h" /* make use of GPIO_MAKE_PIN, GPIO_EXTRACT_PORT, GPIO_EXTRACT_PIN */


typedef void (*iouart_rx_callback_t)(uint8_t d);

typedef struct _iopuart_user_config {
  uint32_t txPinName;
  uint32_t rxPinName;
  uint32_t baudRate;
  
  /* reserved for parity, stop bit and frame length */
} iopuart_user_config_t;

typedef enum _lpuart_status
{
    kStatus_IOUART_Success                  = 0x00U,
    kStatus_IOUART_Fail                     = 0x01U,
    kStatus_IOUART_RxStandbyModeError       = 0x03U,
    kStatus_IOUART_ClearStatusFlagError     = 0x04U,
    kStatus_IOUART_TxNotDisabled            = 0x05U,
    kStatus_IOUART_RxNotDisabled            = 0x06U,
    kStatus_IOUART_TxBusy                   = 0x07U,
    kStatus_IOUART_RxBusy                   = 0x08U,
    kStatus_IOUART_NoTransmitInProgress     = 0x09U,
    kStatus_IOUART_NoReceiveInProgress      = 0x0AU,
    kStatus_IOUART_Timeout                  = 0x0BU,
    kStatus_IOUART_Initialized              = 0x0CU,
    kStatus_IOUART_NoDataToDeal             = 0x0DU,
    kStatus_IOUART_RxOverRun                = 0x0EU
} iouart_status_t;

typedef struct _iouartState {
    const uint8_t *txBuff;          /*!< The buffer of data being sent.*/
    volatile size_t txSize;          /*!< The remaining number of bytes to be transmitted. */
    volatile bool isTxBusy;          /*!< True if there is an active transmit.*/
    iouart_rx_callback_t cb;
  
    uint8_t txStateMachine;
    uint8_t rxStateMachine;
    uint8_t rxBuff;
    
    PORT_Type *portBaseTx;
    PORT_Type *portBaseRx;
    GPIO_Type *gpioBaseTx;
    GPIO_Type *gpioBaseRx;
    uint32_t   pinNumberTx;
    uint32_t   pinNumberRx;
} iouart_state_t;

#if defined(__cplusplus)
extern "C" {
#endif

iouart_status_t IOUART_DRV_Init(iouart_state_t *iouartStatePtr, const iopuart_user_config_t *iouartUserConfig, iouart_rx_callback_t cb);
iouart_status_t IOUART_DRV_SendData(const uint8_t *txBuff, uint32_t txSize);


void IOUART_DRV_EdgeDetectIRQHandler(void);
void IOUART_DRV_TxIRQHandler(void);
void IOUART_DRV_RxIRQHandler(void);

#if defined(__cplusplus)
}
#endif

#endif /* __FSL_IOUART_DRIVER_H__ */
