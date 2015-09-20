#include <assert.h>
#include <string.h>

#include "fsl_interrupt_manager.h"
#include "fsl_clock_manager.h"

#include "fsl_port_hal.h"
#include "fsl_gpio_hal.h"
#include "fsl_tpm_driver.h"

#include "fsl_iouart_driver.h"

static iouart_state_t *g_iouartStatePtr = NULL;


static void _IOUART_DRV_StartTmr(TPM_Type *tpm)
{
    TPM_HAL_SetClockMode(tpm, kTpmClockSourceModuleClk);
}

static void _IOUART_DRV_StopTmr(TPM_Type *tpm)
{
    TPM_HAL_SetClockMode(tpm, kTpmClockSourceNoneClk);
    TPM_HAL_ClearCounter(tpm);
}


iouart_status_t IOUART_DRV_Init(iouart_state_t *iouartStatePtr, const iopuart_user_config_t *iouartUserConfig, iouart_rx_callback_t cb)
{
    assert(iouartStatePtr && iouartUserConfig && cb);

    uint32_t portNumberTx, portNumberRx;
    PORT_Type *portBaseTx, *portBaseRx;
    
    /* Exit if current instance is already initialized. */
    if (g_iouartStatePtr != NULL)
    {
        return kStatus_IOUART_Initialized;
    }

    /* Clear the state struct for this instance. */
    memset(iouartStatePtr, 0, sizeof(iouart_state_t));

    /* Save runtime structure pointer.*/
    g_iouartStatePtr = iouartStatePtr;
    
    portNumberTx = GPIO_EXTRACT_PORT(iouartUserConfig->txPinName);
    portNumberRx = GPIO_EXTRACT_PORT(iouartUserConfig->rxPinName);
    portBaseTx = g_portBase[portNumberTx];
    portBaseRx = g_portBase[portNumberRx];
    
    iouartStatePtr->gpioBaseTx = g_gpioBase[portNumberTx];
    iouartStatePtr->gpioBaseRx = g_gpioBase[portNumberRx];
    iouartStatePtr->pinNumberTx = GPIO_EXTRACT_PIN(iouartUserConfig->txPinName);
    iouartStatePtr->pinNumberRx = GPIO_EXTRACT_PIN(iouartUserConfig->rxPinName);
    
    CLOCK_SYS_EnablePortClock(portNumberTx);
    CLOCK_SYS_EnablePortClock(portNumberRx);
    
    PORT_HAL_SetMuxMode(portBaseTx, iouartStatePtr->pinNumberTx, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(portBaseRx, iouartStatePtr->pinNumberRx, kPortMuxAsGpio);
    
    /* config rx falling edge interrupt */
    PORT_HAL_ClearPinIntFlag(portBaseRx, iouartStatePtr->pinNumberRx);
    PORT_HAL_SetPinIntMode(portBaseRx, iouartStatePtr->pinNumberRx, kPortIntFallingEdge);

    GPIO_HAL_SetPinDir(iouartStatePtr->gpioBaseTx, iouartStatePtr->pinNumberTx, kGpioDigitalOutput);
    GPIO_HAL_SetPinDir(iouartStatePtr->gpioBaseRx, iouartStatePtr->pinNumberRx, kGpioDigitalInput);
    GPIO_HAL_SetPinOutput(iouartStatePtr->gpioBaseTx, iouartStatePtr->pinNumberTx);
    
    INT_SYS_EnableIRQ(g_portIrqId[portNumberRx]);
    
    uint32_t targetFreq = iouartUserConfig->baudRate * 2;
    
    CLOCK_SYS_EnableTpmClock(TPM0_IDX);
    CLOCK_SYS_EnableTpmClock(TPM1_IDX);
    
    CLOCK_SYS_SetTpmSrc(TPM0_IDX, kClockTpmSrcIrc48M);
    CLOCK_SYS_SetTpmSrc(TPM1_IDX, kClockTpmSrcIrc48M);
    
    TPM_HAL_Reset(TPM0, TPM0_IDX);
    TPM_HAL_Reset(TPM1, TPM1_IDX);
    
    TPM_HAL_SetTriggerMode(TPM0, false);
    TPM_HAL_SetTriggerMode(TPM1, false);
    TPM_HAL_SetStopOnOverflowMode(TPM0, false);
    TPM_HAL_SetStopOnOverflowMode(TPM1, false);
    
    TPM_HAL_ClearTimerOverflowFlag(TPM0);
    TPM_HAL_ClearTimerOverflowFlag(TPM1);
    TPM_HAL_EnableTimerOverflowInt(TPM0);
    TPM_HAL_EnableTimerOverflowInt(TPM1);
    
    TPM_HAL_SetClockMode(TPM0, kTpmClockSourceNoneClk);
    TPM_HAL_SetClockMode(TPM1, kTpmClockSourceNoneClk);
    
    TPM_HAL_SetClockDiv(TPM0, kTpmDividedBy1);
    TPM_HAL_SetClockDiv(TPM1, kTpmDividedBy1);
                        
    TPM_HAL_SetMod(TPM0, CLOCK_SYS_GetTpmFreq(TPM0_IDX) / targetFreq - 1);
    TPM_HAL_SetMod(TPM1, CLOCK_SYS_GetTpmFreq(TPM1_IDX) / targetFreq - 1);
    
    //TPM_HAL_SetClockMode(TPM0, kTpmClockSourceModuleClk);
   // TPM_HAL_SetClockMode(TPM1, kTpmClockSourceModuleClk);

    NVIC_ClearPendingIRQ(g_tpmIrqId[TPM0_IDX]);
    INT_SYS_EnableIRQ(g_tpmIrqId[TPM0_IDX]);
    NVIC_ClearPendingIRQ(g_tpmIrqId[TPM1_IDX]);
    INT_SYS_EnableIRQ(g_tpmIrqId[TPM1_IDX]);

    return kStatus_IOUART_Success;
}

iouart_status_t IOUART_DRV_SendData(const uint8_t *txBuff, uint32_t txSize)
{
    assert(txBuff);

    /* Make sure only the previous data are sent out before start another send */
    if (!g_iouartStatePtr->isTxBusy)
    {
       g_iouartStatePtr->isTxBusy = true;
    }
    else
    {
        return kStatus_IOUART_TxBusy;
    }
    
    if (txSize == 0)
    {
        return kStatus_IOUART_NoDataToDeal;
    }

    /* initialize the module driver state structure */
    g_iouartStatePtr->txBuff = txBuff;
    g_iouartStatePtr->txSize = txSize;

    /* Pull low to send Start bit */
    GPIO_HAL_ClearPinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx);
    
    /* start Tx timer to send all bits */
    _IOUART_DRV_StartTmr(TPM0);
    
    return kStatus_IOUART_Success;
}

void IOUART_DRV_EdgeDetectIRQHandler(void)
{
    
    
    _IOUART_DRV_StartTmr(TPM1);
}


void IOUART_DRV_TxIRQHandler(void)
{
    uint32_t bit;
    
    if (TPM_HAL_GetTimerOverflowStatus(TPM0))
    {
        TPM_HAL_ClearTimerOverflowFlag(TPM0);
        
        switch (g_iouartStatePtr->txStateMachine)
        {
        case 0:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 1: // b0
            bit = ((*(g_iouartStatePtr->txBuff)) >> 0) & 0x01;
            GPIO_HAL_WritePinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx, bit);
            
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 2:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 3: // b1
            bit = ((*(g_iouartStatePtr->txBuff)) >> 1) & 0x01;
            GPIO_HAL_WritePinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx, bit);
            
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 4:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 5: // b2
            bit = ((*(g_iouartStatePtr->txBuff)) >> 2) & 0x01;
            GPIO_HAL_WritePinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx, bit);
            
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 6:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 7: // b3
            bit = ((*(g_iouartStatePtr->txBuff)) >> 3) & 0x01;
            GPIO_HAL_WritePinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx, bit);
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 8:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 9: // b4
            bit = ((*(g_iouartStatePtr->txBuff)) >> 3) & 0x01;
            GPIO_HAL_WritePinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx, bit);
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 10:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 11: // b5
            bit = ((*(g_iouartStatePtr->txBuff)) >> 5) & 0x01;
            GPIO_HAL_WritePinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx, bit);
            
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 12:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 13: // b6
            bit = ((*(g_iouartStatePtr->txBuff)) >> 6) & 0x01;
            GPIO_HAL_WritePinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx, bit);
            
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 14:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 15: // b7
            bit = ((*(g_iouartStatePtr->txBuff)) >> 7) & 0x01;
            GPIO_HAL_WritePinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx, bit);
            
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 16:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 17:
            GPIO_HAL_SetPinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 18:
            g_iouartStatePtr->txStateMachine++;
            break;
            
        case 19:
            ++g_iouartStatePtr->txBuff;
            if (--g_iouartStatePtr->txSize > 0)
            {
                GPIO_HAL_ClearPinOutput(g_iouartStatePtr->gpioBaseTx, g_iouartStatePtr->pinNumberTx);
                g_iouartStatePtr->txStateMachine = 0;
            }
            else /* txSize == 0 */
            {
                g_iouartStatePtr->txStateMachine = 0;
                g_iouartStatePtr->isTxBusy = false;
                _IOUART_DRV_StopTmr(TPM0);
            }
            break;
            
        default:
            break;
        }
    }
}

void IOUART_DRV_RxIRQHandler(void)
{
    uint32_t level;
    
    if (TPM_HAL_GetTimerOverflowStatus(TPM1))
    {
        TPM_HAL_ClearTimerOverflowFlag(TPM1);
        
        switch (g_iouartStatePtr->rxStateMachine)
        {
        case 0:
            g_iouartStatePtr->rxBuff = 0;
            // check start bit
            break;
            
        case 1: 
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 2: // b0
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->rxBuff |= level << 0;
            break;
            
        case 3:
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 4: // b1
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->rxBuff |= level << 1;
            break;
            
        case 5: 
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 6: // b2
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->rxBuff |= level << 2;
            break;
            
        case 7: 
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 8: // b3
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->rxBuff |= level << 3;
            break;
            
        case 9: 
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 10: // b4
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->rxBuff |= level << 4;
            break;
            
        case 11: 
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 12: // b5
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->rxBuff |= level << 5;
            break;
            
        case 13: 
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 14: // b6
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->rxBuff |= level << 6;
            break;
            
        case 15: 
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 16: // b7
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            g_iouartStatePtr->rxBuff |= level << 7;
            break;
            
        case 17:
            g_iouartStatePtr->rxStateMachine++;
            break;
            
        case 18:
            level = GPIO_HAL_ReadPinInput(g_iouartStatePtr->gpioBaseRx, g_iouartStatePtr->pinNumberTx);
            if (level != 1)
            {
                // frame error
            }
            
            // as quick as possible, other wise will affect the following receive progress
            (*g_iouartStatePtr->cb)(g_iouartStatePtr->rxBuff);
            
            g_iouartStatePtr->rxStateMachine = 0;
            _IOUART_DRV_StopTmr(TPM1);
            break;

        default:
            break;
        }
    }
}


