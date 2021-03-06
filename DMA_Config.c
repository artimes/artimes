#include "Global.h"

/**
  * @brief  DMA channel1 configuration
  * @param  None
  * @retval None
  */

#ifdef DMA_ADC
__IO uint16_t RegularConvData_Tab[4];
#endif
#ifdef DMA_ADC
static void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure ;
    /* DMA1 clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
    
    /* DMA1 Channel1 Config */
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)ADC1_DR_Address ;
    DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)RegularConvData_Tab ;
    DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC ;
    DMA_InitStructure.DMA_BufferSize=4 ;
    DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable ;
    DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable ;
    DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord ;
    DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord ;
    DMA_InitStructure.DMA_Mode=DMA_Mode_Circular ;
    DMA_InitStructure.DMA_Priority=DMA_Priority_High ;
    DMA_InitStructure.DMA_M2M=DMA_M2M_Disable ;
    DMA_Init(DMA1_Channel1,&DMA_InitStructure);
    /* DMA1 Channel1 enable */
    DMA_Cmd(DMA1_Channel1,ENABLE);
}
#endif
