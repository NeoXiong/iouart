#include "fsl_iouart_driver.h"

#if (PORT_INSTANCE_COUNT > 0)

void PORTA_IRQHandler(void)
{
    IOUART_DRV_EdgeDetectIRQHandler();
}

#endif

#if (PORT_INSTANCE_COUNT > 1)

void PORTB_IRQHandler(void)
{
    IOUART_DRV_EdgeDetectIRQHandler();
}

#endif

#if (PORT_INSTANCE_COUNT > 2)

void PORTC_IRQHandler(void)
{
    IOUART_DRV_EdgeDetectIRQHandler();
}

#endif

#if (PORT_INSTANCE_COUNT > 3)

void PORTD_IRQHandler(void)
{
    IOUART_DRV_EdgeDetectIRQHandler();
}

#endif

#if (PORT_INSTANCE_COUNT > 4)

void PORTE_IRQHandler(void)
{
    IOUART_DRV_EdgeDetectIRQHandler();
}

#endif

void TPM0_IRQHandler(void)
{
    IOUART_DRV_TxIRQHandler();
}

void TPM1_IRQHandler(void)
{
    IOUART_DRV_RxIRQHandler();
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

