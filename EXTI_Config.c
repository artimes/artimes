#include "Global.h"

void EXTI_Config(void)
{
    /********************�����ʼ���õĽṹ�����***********************************/
    EXTI_InitTypeDef EXTI_InitStructure ;
    NVIC_InitTypeDef NVIC_InitStructure ;
    /*******************************************************************************/
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);//ʹ������ʱ��
    
    /**********************����HU�ⲿ�ж�***************************************/
    SYSCFG_EXTILineConfig(EXTI_PortSource_HU,EXTI_PinSource_HU);
    
    EXTI_InitStructure.EXTI_Line=EXTI_Line_HU ;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt ;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling ;
    //EXTI_Trigger_Rising_Falling
    EXTI_InitStructure.EXTI_LineCmd=ENABLE ;
    EXTI_Init(&EXTI_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSource_HV,EXTI_PinSource_HV);
    
    EXTI_InitStructure.EXTI_Line=EXTI_Line_HV ;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt ;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling ;
    //EXTI_Trigger_Rising_Falling
    EXTI_InitStructure.EXTI_LineCmd=ENABLE ;
    EXTI_Init(&EXTI_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSource_HW,EXTI_PinSource_HW);
    
    EXTI_InitStructure.EXTI_Line=EXTI_Line_HW ;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt ;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling ;
    //EXTI_Trigger_Rising_Falling
    EXTI_InitStructure.EXTI_LineCmd=ENABLE ;
    EXTI_Init(&EXTI_InitStructure);
    
    SYSCFG_EXTILineConfig(EXTI_PortSource_LS,EXTI_PinSource_LS);
    
    EXTI_InitStructure.EXTI_Line=EXTI_Line_LS ;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt ;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling ;
    //EXTI_Trigger_Rising_Falling
    EXTI_InitStructure.EXTI_LineCmd=ENABLE ;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel=EXTI_IRQn_HU ;
    NVIC_InitStructure.NVIC_IRQChannelPriority=0x01 ;
    //�ж����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel=EXTI_IRQn_HW ;
    NVIC_InitStructure.NVIC_IRQChannelPriority=0x01 ;
    //�ж����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel=EXTI_IRQn_LS ;
    NVIC_InitStructure.NVIC_IRQChannelPriority=0x01 ;
    //�ж����ȼ�Ϊ1
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
    NVIC_Init(&NVIC_InitStructure);
    /***************************************************************************/
    
    
    
    
    /**********************���ù��������ж�*************************************/
    /*
    	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);
    	
      EXTI_InitStructure.EXTI_Line = EXTI_Line3;
      EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
      EXTI_InitStructure.EXTI_LineCmd = ENABLE;
      EXTI_Init(&EXTI_InitStructure);
    
      NVIC_InitStructure.NVIC_IRQChannel = EXTI2_3_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
    	*/
    /***************************************************************************/
}
