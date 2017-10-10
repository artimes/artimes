#include "Global.h"


void GPIO_Config(void)
{
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC|RCC_AHBPeriph_GPIOF,ENABLE);
    
    //电源输出引脚初始化，高有效，弱拉低	
    GPIO_InitTypeDef GPIO_InitStructure ;
    /*GPIO_InitStructure.GPIO_Pin = POWER_PIN;     //电源开关
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //输出
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//低速输出
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//高有效，弱拉低
      GPIO_Init(POWER_PIN_GROUP, &GPIO_InitStructure);*/
    
    //大灯输出引脚初始化，高有效，弱拉低	
    GPIO_InitStructure.GPIO_Pin=LIGHT_PIN ;//大灯
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT ;//输出
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;//推挽输出
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;//低速输出
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN ;//高有效，弱拉低
    GPIO_Init(LIGHT_PIN_GROUP,&GPIO_InitStructure);
    
    //大灯短路输入引脚初始化，高有效，弱拉低	
    GPIO_InitStructure.GPIO_Pin=LightShort_PIN ;//大灯
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//输入
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//不拉高或拉低
    GPIO_Init(LightShort_PIN_GROUP,&GPIO_InitStructure);
    
    //刹车输入信号引脚初始化，低有效，弱拉高	
    GPIO_InitStructure.GPIO_Pin=BRAKE_PIN ;//刹车
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//输入
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP ;
    GPIO_Init(BRAKE_PIN_GROUP,&GPIO_InitStructure);
    
    //U相霍尔输入信号引脚初始化
    GPIO_InitStructure.GPIO_Pin=HU_PIN ;//HU信号
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//输入
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//不拉高或拉低
    GPIO_Init(HU_PIN_GROUP,&GPIO_InitStructure);
    
    //V相霍尔输入信号引脚初始化
    GPIO_InitStructure.GPIO_Pin=HV_PIN ;//HV信号
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//输入
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//不拉高或拉低
    GPIO_Init(HV_PIN_GROUP,&GPIO_InitStructure);
    
    //W相霍尔输入信号引脚初始化
    GPIO_InitStructure.GPIO_Pin=HW_PIN ;//HW信号
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//输入
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//不拉高或拉低
    GPIO_Init(HW_PIN_GROUP,&GPIO_InitStructure);
    
    //角位信号引脚初始化
    GPIO_InitStructure.GPIO_Pin=ANGLE_PLUSE_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//输入
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP ;//不拉高或拉低
    GPIO_Init(ANGLE_PLUSE_PIN_GROUP,&GPIO_InitStructure);
    
    //过流信号引脚初始化，高有效，弱拉低
    GPIO_InitStructure.GPIO_Pin=OVER_CURRENT_PIN ;//过流信号
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//输入
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN ;//高有效，弱拉低
    GPIO_Init(OVER_CURRENT_PIN_GROUP,&GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9 ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11 ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
}
