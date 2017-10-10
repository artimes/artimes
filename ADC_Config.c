#include "Global.h"
#define ADC1_DR_Address                0x40012440         
//ADCת��ֵ�Ĵ�����ַ

/*************ADC��DMAһ���ʼ��*****************************************/
//ADCУ�������ڸ�λ״̬�½��У���ADCû��ʹ��ʱ
//���� ADC��λ  ʹ��ADCʱ��  ADCУ��  ʹ��DMAʱ��  ���ò�ʹ��DMA  ����ADC
//ʹ��ȫ�ֱ���ADC_Values���洢ADת��ֵ������
void ADCWithDMA_Config(void)
{
    /****************************��������*************************************************/
    ADC_InitTypeDef ADC_InitStructure ;
    DMA_InitTypeDef DMA_InitStructure ;
    GPIO_InitTypeDef GPIO_InitStructure ;
    /*************************************************************************************/
    
    /***********************ADC��λ��ʹ��ADCʱ�ӣAADCУ��*********************************/
    ADC_DeInit(ADC1);//ADC��λ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//ʹ��ADCʱ��
    ADC_GetCalibrationFactor(ADC1);//ADCУ��
    /*************************************************************************************/
    
    /**********************************DMA����********************************************/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//ʹ��DMAʱ��
    
    /* ����DMAͨ��1 */
    DMA_DeInit(DMA1_Channel1);//DMAͨ��1��λ
    DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)ADC1_DR_Address ;//�趨ת�����������ַ
    DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)ADC_Values ;//�趨ת�����ݵĴ洢��ַ
    DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC ;//�趨ת������Ϊ�����赽�洢
    DMA_InitStructure.DMA_BufferSize=7 ;//�趨���泤��
    DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable ;//��������ģʽ��ֹ
    DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable ;//�洢����ģʽʹ��
    DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord ;//�������ݳ��Ȱ���ֳ�
    DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord ;//�洢���ݳ��Ȱ���ֳ�
    DMA_InitStructure.DMA_Mode=DMA_Mode_Circular ;//DMAģʽΪѭ��
    DMA_InitStructure.DMA_Priority=DMA_Priority_High ;//DMA���ȼ�Ϊ��
    DMA_InitStructure.DMA_M2M=DMA_M2M_Disable ;//�洢�Դ洢��ֹ
    DMA_Init(DMA1_Channel1,&DMA_InitStructure);//��ʼ��DMAͨ��
    
    DMA_Cmd(DMA1_Channel1,ENABLE);//ʹ��DMAͨ��
    
    ADC_DMARequestModeConfig(ADC1,ADC_DMAMode_Circular);//ADC��DMA����һֱ����
    
    ADC_DMACmd(ADC1,ENABLE);//ʹ��ADC��DMA����
    /******************************************************************************************/
    
    
    /******************ADC�������ų�ʼ��******************************/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
    
    //���ư��¶Ȳ�����ʼ��
    GPIO_InitStructure.GPIO_Pin=TEMP_CONTROLLER_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(TEMP_CONTROLLER_PIN_GROUP,&GPIO_InitStructure);
    
    //����¶Ȳ�����ʼ��
    GPIO_InitStructure.GPIO_Pin=TEMP_MOTOR_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(TEMP_MOTOR_PIN_GROUP,&GPIO_InitStructure);
    
    //�����źŲ�����ʼ��
    GPIO_InitStructure.GPIO_Pin=TORQUE_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(TORQUE_PIN_GROUP,&GPIO_InitStructure);
    
    //��ѹ�źŲ�����ʼ��
    GPIO_InitStructure.GPIO_Pin=VOLTAGE_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(VOLTAGE_PIN_GROUP,&GPIO_InitStructure);
    
    //��ѹ�źŲ�����ʼ��
    GPIO_InitStructure.GPIO_Pin=VOLTAGE_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(VOLTAGE_PIN_GROUP,&GPIO_InitStructure);
    
    //W������źŲ�����ʼ��
    GPIO_InitStructure.GPIO_Pin=CURRENT_W_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(CURRENT_W_PIN_GROUP,&GPIO_InitStructure);
    
    //V������źŲ�����ʼ��
    GPIO_InitStructure.GPIO_Pin=CURRENT_V_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(CURRENT_V_PIN_GROUP,&GPIO_InitStructure);
    
    //U������źŲ�����ʼ��
    GPIO_InitStructure.GPIO_Pin=CURRENT_U_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(CURRENT_U_PIN_GROUP,&GPIO_InitStructure);
    /**********************************************************/
    
    
    /************************************ADC����**************************************************/
    ADC_StructInit(&ADC_InitStructure);//ADC�ṹ�帴λ
    ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b ;//ADC������12λ
    ADC_InitStructure.ADC_ContinuousConvMode=DISABLE ;//��ֹ����ת��
    if(CURRENT_T1==1)
    {
        ADC_InitStructure.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_Rising ;//�ⲿ�����ش���ת��
        ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_TRGO ;//T1����ADCת��
    }
    
    ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right ;//ADCת���������Ҷ���
    ADC_InitStructure.ADC_ScanDirection=ADC_ScanDirection_Backward ;//����ɨ��
    ADC_Init(ADC1,&ADC_InitStructure);//�ýṹ���ʼ��ADC
    
    //����ADCͨ��
    ADC_ChannelConfig(ADC1,TEMP_CONTROLLER_PIN_CHANNEL|TEMP_MOTOR_PIN_CHANNEL|TORQUE_PIN_CHANNEL|
    VOLTAGE_PIN_CHANNEL|CURRENT_W_PIN_CHANNEL|CURRENT_V_PIN_CHANNEL|CURRENT_U_PIN_CHANNEL,ADC_SampleTime_7_5Cycles);
    
    ADC_Cmd(ADC1,ENABLE);//ʹ��ADCģ��
    if(CURRENT_T1==1)
    {
        while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY));
        ADC_StartOfConversion(ADC1);//��ת�����أ�ʵ��ת����ʼ��T1������
    }
}



