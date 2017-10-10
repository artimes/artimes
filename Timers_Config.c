#include "Global.h"

/************************Timer1配置，触发ADC采样*************************/
void Timers_Config(void)
{
    //定义用到的变量
    uint16_t TimerPeriod=0 ;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
    TIM_OCInitTypeDef TIM_OCInitStructure ;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure ;
    GPIO_InitTypeDef GPIO_InitStructure ;
    
    
    /* 使能用到的外部引脚时钟 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB,ENABLE);
    
    /* 使能TIM1模块时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
    
    /*************************引脚初始化**************************************/
    //UP引脚初始化
    GPIO_InitStructure.GPIO_Pin=UP_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(UP_PIN_GROUP,&GPIO_InitStructure);
    
    //UN引脚初始化
    GPIO_InitStructure.GPIO_Pin=UN_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(UN_PIN_GROUP,&GPIO_InitStructure);
    
    //VP引脚初始化
    GPIO_InitStructure.GPIO_Pin=VP_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(VP_PIN_GROUP,&GPIO_InitStructure);
    
    //VN引脚初始化
    GPIO_InitStructure.GPIO_Pin=VN_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(VN_PIN_GROUP,&GPIO_InitStructure);
    
    //WP引脚初始化
    GPIO_InitStructure.GPIO_Pin=WP_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(WP_PIN_GROUP,&GPIO_InitStructure);
    
    //WN引脚初始化
    GPIO_InitStructure.GPIO_Pin=WN_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF ;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(WN_PIN_GROUP,&GPIO_InitStructure);
    
    //PWM模块引脚设为外设功能
    GPIO_PinAFConfig(UP_PIN_GROUP,UP_PIN_SOURCE,GPIO_AF_2);
    GPIO_PinAFConfig(UN_PIN_GROUP,UN_PIN_SOURCE,GPIO_AF_2);
    GPIO_PinAFConfig(VP_PIN_GROUP,VP_PIN_SOURCE,GPIO_AF_2);
    GPIO_PinAFConfig(VN_PIN_GROUP,VN_PIN_SOURCE,GPIO_AF_2);
    GPIO_PinAFConfig(WP_PIN_GROUP,WP_PIN_SOURCE,GPIO_AF_2);
    GPIO_PinAFConfig(WN_PIN_GROUP,WN_PIN_SOURCE,GPIO_AF_2);
    /*****************************************************************************/
    
    
    /*TIM1复原  */
    TIM_DeInit(TIM1);
    
    /**************************TIM1时基初始化****************************************/
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    
    TimerPeriod=(SystemCoreClock/(TIM1_FREQ*2))-1 ;
    // because TIM_CounterMode_CenterAligned1 it has to use twice value
    
    PWM_double=(TimerPeriod<<1);
    PWM_half=TimerPeriod>>1 ;
    PWM_max=TimerPeriod ;
    
    /* Time Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler=0 ;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_CenterAligned1 ;
    ////LB replaced: TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period=TimerPeriod ;
    TIM_TimeBaseStructure.TIM_ClockDivision=0 ;
    if(TIM1_PWM_MODE==2)
    //TIM1->RCR = 1; 																										//更新事件分频
    TIM_TimeBaseStructure.TIM_RepetitionCounter=1 ;
    //LB replaced 0 with 1; see rm pp.218
    
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
    /*********************************************************************************/
    
    /**************************TIM比较输出单元初始化**********************************************/
    TIM_OCStructInit(&TIM_OCInitStructure);
    
    if(TIM1_PWM_MODE==1)
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1 ;
    //上管开通与比较值成正比。
    
    if(TIM1_PWM_MODE==2)
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2 ;
    //下管开通与比较值成正比
    
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable ;
    TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable ;
    TIM_OCInitStructure.TIM_OCPolarity=OCP ;
    // TIM_OCPolarity_Low;    
    TIM_OCInitStructure.TIM_OCNPolarity=OCN ;
    TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set ;
    TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset ;
    
    //计算各通道占空比
    //Channel1Pulse = (uint16_t) (((uint32_t) 0 * (TimerPeriod - 1)) / 100);
    //Channel2Pulse = (uint16_t) (((uint32_t) 50 * (TimerPeriod - 1)) / 100);
    //Channel3Pulse = (uint16_t) (((uint32_t) 80 * (TimerPeriod - 1)) / 100);
    
    TIM_OCInitStructure.TIM_Pulse=PWM_half ;
    TIM_OC1Init(TIM1,&TIM_OCInitStructure);
    
    TIM_OCInitStructure.TIM_Pulse=PWM_half ;
    TIM_OC2Init(TIM1,&TIM_OCInitStructure);
    
    TIM_OCInitStructure.TIM_Pulse=PWM_half ;
    TIM_OC3Init(TIM1,&TIM_OCInitStructure);
    
    //使能预加载寄存器
    TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
    
    
    /**************************死区单元初始化************************************************/
    TIM_BDTRStructInit(&TIM_BDTRInitStructure);
    TIM_BDTRInitStructure.TIM_OSSRState=TIM_OSSRState_Enable ;
    TIM_BDTRInitStructure.TIM_OSSIState=TIM_OSSIState_Enable ;
    TIM_BDTRInitStructure.TIM_LOCKLevel=TIM_LOCKLevel_1 ;
    TIM_BDTRInitStructure.TIM_DeadTime=80 ;
    TIM_BDTRInitStructure.TIM_Break=TIM_Break_Disable ;
    // LB replaced TIM_Break_Enable;
    TIM_BDTRInitStructure.TIM_BreakPolarity=TIM_BreakPolarity_High ;
    TIM_BDTRInitStructure.TIM_AutomaticOutput=TIM_AutomaticOutput_Disable ;
    // LB TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);
    /****************************************************************************************/
    
    if(CURRENT_T1==1)
    TIM_SelectOutputTrigger(TIM1,TIM_TRGOSource_Update);//选择更新事件为触发源触发ADC转换
    TIM_Cmd(TIM1,ENABLE);//使能TIM1
    if(TIM1_PWM_MODE==1)
    TIM1->RCR=1 ;//选择PWM1，使用上计中断
    
    /**************************TIM3初始化****************************************************/
    TIM_DeInit(TIM3);//TIM3复位
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);//时基结构体复位
    TIM_OCStructInit(&TIM_OCInitStructure);
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//使能TIM3时钟源
    
    //Time base configuration /
    TIM_TimeBaseStructure.TIM_Period=65535 ;//设置TIM3周期
    TIM_TimeBaseStructure.TIM_Prescaler=48000000/TIM3_FREQ ;//设置TIM3计数频率
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up ;
    
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//TIM3时基初始化
    TIM_Cmd(TIM3,ENABLE);//使能TIM3
}

/************************Timer1中断配置*************************/
void setupTIM1_Interrupt(void)
{
    NVIC_InitTypeDef NVIC_InitStructure ;
    
    /* Enable the TIM1_BRK_UP_TRG_COM_IRQn gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_UP_TRG_COM_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPriority=0 ;
    //T1中断优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE ;
    NVIC_Init(&NVIC_InitStructure);
    
    /* TIM Interrupts enable */
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
}


/************************Timer1主中断*************************/

