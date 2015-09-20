/* 
    fsl_iouart_irq.c 

    neo.xiong@freescale.com

    This file override weak entry label defined in .s startup code file, 
    Generally, there're not part of IOUART_Driver because other code may also
    use the shared interrupt entry, e.g. other gpio interrupts on the same
    port. But for TPMx interrupt, it's less likely they can be used for other 
    function as we defined specific period and controlled when it run and stop. 
*/

extern void IOUART_DRV_EdgeDetectIRQHandler(void);
extern void IOUART_DRV_TxIRQHandler(void);
extern void IOUART_DRV_RxIRQHandler(void);

void PORTB_IRQHandler(void)
{
    /* add your other GPIO interrupt code here */
    
    IOUART_DRV_EdgeDetectIRQHandler();
}

void TPM0_IRQHandler(void)
{
    IOUART_DRV_TxIRQHandler();
}

void TPM1_IRQHandler(void)
{
    IOUART_DRV_RxIRQHandler();
}
