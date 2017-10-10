#include "Global.h"
#define ADC1_DR_Address                0x40012440         
//ADC转换值寄存器地址

/*************ADC和DMA一起初始化*****************************************/
//ADC校正必须在复位状态下进行，即ADC没有使能时
//流程 ADC复位  使能ADC时钟  ADC校正  使能DMA时钟  配置并使能DMA  配置ADC
//使用全局变量ADC_Values，存储AD转换值的数列
void ADCWithDMA_Config(void)
{
    /****************************变量定义*************************************************/
    ADC_InitTypeDef ADC_InitStructure ;
    DMA_InitTypeDef DMA_InitStructure ;
    GPIO_InitTypeDef GPIO_InitStructure ;
    /*************************************************************************************/
    
    /***********************ADC复位，使能ADC时钟ADC校正*********************************/
    ADC_DeInit(ADC1);//ADC复位
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//使能ADC时钟
    ADC_GetCalibrationFactor(ADC1);//ADC校正
    /*************************************************************************************/
    
    /**********************************DMA配置********************************************/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//使能DMA时钟
    
    /* 配置DMA通道1 */
    DMA_DeInit(DMA1_Channel1);//DMA通道1复位
    DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)ADC1_DR_Address ;//设定转移数据外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr=(uint32_t)ADC_Values ;//设定转移数据的存储地址
    DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralSRC ;//设定转移数据为从外设到存储
    DMA_InitStructure.DMA_BufferSize=7 ;//设定缓存长度
    DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable ;//外设增长模式禁止
    DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable ;//存储增长模式使能
    DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_HalfWord ;//外设数据长度半个字长
    DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord ;//存储数据长度半个字长
    DMA_InitStructure.DMA_Mode=DMA_Mode_Circular ;//DMA模式为循环
    DMA_InitStructure.DMA_Priority=DMA_Priority_High ;//DMA优先级为高
    DMA_InitStructure.DMA_M2M=DMA_M2M_Disable ;//存储对存储禁止
    DMA_Init(DMA1_Channel1,&DMA_InitStructure);//初始化DMA通道
    
    DMA_Cmd(DMA1_Channel1,ENABLE);//使能DMA通道
    
    ADC_DMARequestModeConfig(ADC1,ADC_DMAMode_Circular);//ADC向DMA请求一直存在
    
    ADC_DMACmd(ADC1,ENABLE);//使能ADC对DMA请求
    /******************************************************************************************/
    
    
    /******************ADC输入引脚初始化******************************/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
    
    //控制板温度采样初始化
    GPIO_InitStructure.GPIO_Pin=TEMP_CONTROLLER_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(TEMP_CONTROLLER_PIN_GROUP,&GPIO_InitStructure);
    
    //电机温度采样初始化
    GPIO_InitStructure.GPIO_Pin=TEMP_MOTOR_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(TEMP_MOTOR_PIN_GROUP,&GPIO_InitStructure);
    
    //力矩信号采样初始化
    GPIO_InitStructure.GPIO_Pin=TORQUE_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(TORQUE_PIN_GROUP,&GPIO_InitStructure);
    
    //电压信号采样初始化
    GPIO_InitStructure.GPIO_Pin=VOLTAGE_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(VOLTAGE_PIN_GROUP,&GPIO_InitStructure);
    
    //电压信号采样初始化
    GPIO_InitStructure.GPIO_Pin=VOLTAGE_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(VOLTAGE_PIN_GROUP,&GPIO_InitStructure);
    
    //W相电流信号采样初始化
    GPIO_InitStructure.GPIO_Pin=CURRENT_W_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(CURRENT_W_PIN_GROUP,&GPIO_InitStructure);
    
    //V相电流信号采样初始化
    GPIO_InitStructure.GPIO_Pin=CURRENT_V_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(CURRENT_V_PIN_GROUP,&GPIO_InitStructure);
    
    //U相电流信号采样初始化
    GPIO_InitStructure.GPIO_Pin=CURRENT_U_PIN ;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN ;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_Init(CURRENT_U_PIN_GROUP,&GPIO_InitStructure);
    /**********************************************************/
    
    
    /************************************ADC配置**************************************************/
    ADC_StructInit(&ADC_InitStructure);//ADC结构体复位
    ADC_InitStructure.ADC_Resolution=ADC_Resolution_12b ;//ADC解析度12位
    ADC_InitStructure.ADC_ContinuousConvMode=DISABLE ;//禁止连续转换
    if(CURRENT_T1==1)
    {
        ADC_InitStructure.ADC_ExternalTrigConvEdge=ADC_ExternalTrigConvEdge_Rising ;//外部上升沿触发转换
        ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T1_TRGO ;//T1触发ADC转换
    }
    
    ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right ;//ADC转换的数据右对齐
    ADC_InitStructure.ADC_ScanDirection=ADC_ScanDirection_Backward ;//反向扫描
    ADC_Init(ADC1,&ADC_InitStructure);//用结构体初始化ADC
    
    //配置ADC通道
    ADC_ChannelConfig(ADC1,TEMP_CONTROLLER_PIN_CHANNEL|TEMP_MOTOR_PIN_CHANNEL|TORQUE_PIN_CHANNEL|
    VOLTAGE_PIN_CHANNEL|CURRENT_W_PIN_CHANNEL|CURRENT_V_PIN_CHANNEL|CURRENT_U_PIN_CHANNEL,ADC_SampleTime_7_5Cycles);
    
    ADC_Cmd(ADC1,ENABLE);//使能ADC模块
    if(CURRENT_T1==1)
    {
        while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_ADRDY));
        ADC_StartOfConversion(ADC1);//打开转换开关（实际转换开始由T1触发）
    }
}



