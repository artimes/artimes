#include "Global.h"


void UART1_Config(void)
{
    /***********************定义使用的变量************************************/
    GPIO_InitTypeDef GPIO_InitStructure ;
    USART_InitTypeDef USART_InitStruct ;
    /*************************************************************************/
    
    USART_DeInit(USART1);//UART复位
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能UART时钟
    
    /**************************引脚功能初始化****************************************/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);//引脚时钟使能
    
    //RXD引脚初始化	
    GPIO_InitStructure.GPIO_Pin=RXD_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_OD ;//PP
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//UP
    GPIO_Init(RXD_PIN_GROUP,&GPIO_InitStructure);
    
    //TXD引脚初始化	
    GPIO_InitStructure.GPIO_Pin=TXD_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;//PP
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//UP
    GPIO_Init(TXD_PIN_GROUP,&GPIO_InitStructure);
    
    GPIO_PinAFConfig(RXD_PIN_GROUP,RXD_PIN_SOURCE,GPIO_AF_0);
    GPIO_PinAFConfig(TXD_PIN_GROUP,TXD_PIN_SOURCE,GPIO_AF_0);
    /*******************************************************************/
    
    
    USART_StructInit(&USART_InitStruct);
    
    USART_InitStruct.USART_BaudRate=UART1_BAUDRATE ;
    USART_InitStruct.USART_WordLength=USART_WordLength_8b ;
    USART_InitStruct.USART_StopBits=USART_StopBits_1 ;
    USART_InitStruct.USART_Parity=USART_Parity_No ;
    USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx ;//使能发送和接收
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None ;
    
    USART_Init(USART1,&USART_InitStruct);
    USART_Cmd(USART1,ENABLE);
    
    
    USART1->ICR|=USART_ICR_TCCF ;
    USART_ITConfig(USART1,USART_IT_TC,DISABLE);//禁止串口发送中断
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//使能串口接收中断
    
    NVIC_InitTypeDef NVIC_InitStructure ;
    NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPriority=3 ;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
    NVIC_Init(&NVIC_InitStructure);
}



void UART2_Config(void)
{
    /***********************定义使用的变量************************************/
    GPIO_InitTypeDef GPIO_InitStructure ;
    USART_InitTypeDef USART_InitStruct ;
    /*************************************************************************/
    
    //UART复位
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能UART时钟
    
    /**************************引脚功能初始化****************************************/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);//引脚时钟使能
    
    USART_DeInit(USART2);
    
    GPIO_InitStructure.GPIO_Pin=RXD2_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_OD ;//PP
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//UP
    GPIO_Init(RXD2_PIN_GROUP,&GPIO_InitStructure);
    
    //TXD引脚初始化	
    GPIO_InitStructure.GPIO_Pin=TXD2_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;//PP
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//UP
    GPIO_Init(TXD2_PIN_GROUP,&GPIO_InitStructure);
    
    GPIO_PinAFConfig(RXD2_PIN_GROUP,RXD2_PIN_SOURCE,GPIO_AF_1);
    GPIO_PinAFConfig(TXD2_PIN_GROUP,TXD2_PIN_SOURCE,GPIO_AF_1);
    
    /*******************************************************************/
    
    
    USART_StructInit(&USART_InitStruct);
    
    USART_InitStruct.USART_BaudRate=UART2_BAUDRATE ;
    USART_InitStruct.USART_WordLength=USART_WordLength_8b ;
    USART_InitStruct.USART_StopBits=USART_StopBits_1 ;
    USART_InitStruct.USART_Parity=USART_Parity_No ;
    USART_InitStruct.USART_Mode=USART_Mode_Rx|USART_Mode_Tx ;//使能发送和接收
    USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None ;
    
    USART_Init(USART2,&USART_InitStruct);
    USART_Cmd(USART2,ENABLE);
    
    
    USART2->ICR|=USART_ICR_TCCF ;
    USART_ITConfig(USART2,USART_IT_TC,DISABLE);//禁止串口发送中断
    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//使能串口接收中断
    
    NVIC_InitTypeDef NVIC_InitStructure ;
    NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPriority=3 ;
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
    NVIC_Init(&NVIC_InitStructure);
}




