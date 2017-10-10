#include "Global.h"


void GPIO_Config(void)
{
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC|RCC_AHBPeriph_GPIOF,ENABLE);
    
    //��Դ������ų�ʼ��������Ч��������	
    GPIO_InitTypeDef GPIO_InitStructure ;
    /*GPIO_InitStructure.GPIO_Pin = POWER_PIN;     //��Դ����
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //���
      GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//�������
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//����Ч��������
      GPIO_Init(POWER_PIN_GROUP, &GPIO_InitStructure);*/
    
    //���������ų�ʼ��������Ч��������	
    GPIO_InitStructure.GPIO_Pin=LIGHT_PIN ;//���
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT ;//���
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP ;//�������
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz ;//�������
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN ;//����Ч��������
    GPIO_Init(LIGHT_PIN_GROUP,&GPIO_InitStructure);
    
    //��ƶ�·�������ų�ʼ��������Ч��������	
    GPIO_InitStructure.GPIO_Pin=LightShort_PIN ;//���
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//����
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//�����߻�����
    GPIO_Init(LightShort_PIN_GROUP,&GPIO_InitStructure);
    
    //ɲ�������ź����ų�ʼ��������Ч��������	
    GPIO_InitStructure.GPIO_Pin=BRAKE_PIN ;//ɲ��
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//����
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP ;
    GPIO_Init(BRAKE_PIN_GROUP,&GPIO_InitStructure);
    
    //U����������ź����ų�ʼ��
    GPIO_InitStructure.GPIO_Pin=HU_PIN ;//HU�ź�
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//����
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//�����߻�����
    GPIO_Init(HU_PIN_GROUP,&GPIO_InitStructure);
    
    //V����������ź����ų�ʼ��
    GPIO_InitStructure.GPIO_Pin=HV_PIN ;//HV�ź�
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//����
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//�����߻�����
    GPIO_Init(HV_PIN_GROUP,&GPIO_InitStructure);
    
    //W����������ź����ų�ʼ��
    GPIO_InitStructure.GPIO_Pin=HW_PIN ;//HW�ź�
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//����
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;//�����߻�����
    GPIO_Init(HW_PIN_GROUP,&GPIO_InitStructure);
    
    //��λ�ź����ų�ʼ��
    GPIO_InitStructure.GPIO_Pin=ANGLE_PLUSE_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//����
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP ;//�����߻�����
    GPIO_Init(ANGLE_PLUSE_PIN_GROUP,&GPIO_InitStructure);
    
    //�����ź����ų�ʼ��������Ч��������
    GPIO_InitStructure.GPIO_Pin=OVER_CURRENT_PIN ;//�����ź�
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN ;//����
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN ;//����Ч��������
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
