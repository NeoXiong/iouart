#ifndef __FSL_IOUART_DRIVER_H__
#define __FSL_IOUART_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_os_abstraction.h"

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
    const uint8_t * txBuff;          /*!< The buffer of data being sent.*/
	uint8_t * rxBuff;                /*!< The buffer of received data.*/
    volatile size_t txSize;          /*!< The remaining number of bytes to be transmitted. */
    volatile size_t rxSize;          /*!< The remaining number of bytes to be received. */
    volatile bool isTxBusy;          /*!< True if there is an active transmit.*/
    volatile bool isRxBusy;          /*!< True if there is an active receive.*/
    volatile bool isTxBlocking;      /*!< True if transmit is blocking transaction. */
    volatile bool isRxBlocking;      /*!< True if receive is blocking transaction. */
    semaphore_t txIrqSync;           /*!< Used to wait for ISR to complete its Tx business.*/
    semaphore_t rxIrqSync;           /*!< Used to wait for ISR to complete its Rx business.*/
    //lpuart_rx_callback_t rxCallback; /*!< Callback to invoke after receiving byte.*/
    void * rxCallbackParam;          /*!< Receive callback parameter pointer.*/
    //lpuart_tx_callback_t txCallback; /*!< Callback to invoke after transmitting byte.*/
    void * txCallbackParam;          /*!< Transmit callback parameter pointer.*/
} iouart_state_t;

typedef enum _iouart_baudrate{
    kIoUartBaudRate4800,
    kIoUartBaudRate9600,
    kIoUartBaudRate19200,
    kIoUartBaudRate38400
} iouart_baudrate_t;

#if defined(__cplusplus)
extern "C" {
#endif

iouart_status_t IOUART_DRV_Init(uint32_t instance, iouart_state_t *iouartStatePtr, iouart_baudrate_t baudRate);
iouart_status_t IOUART_DRV_SendData(uint32_t instance, const uint8_t *txBuff, uint32_t txSize);

#if defined(__cplusplus)
}
#endif

#endif /* __FSL_IOUART_DRIVER_H__ */
