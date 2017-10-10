/**********************定义用到的全局变量**********************************/
#include "Global.h" 


/************电流相关的全局变量**********************/

volatile int16_t ADC_Values[7]=
{
    0,0,0,0,0,0,0 
}
;
volatile uint8_t Flag_offset_finished ;

volatile uint32_t flag_all_low300=0 ;

volatile _iq Isa=0,Isb=0,Isc=0,I_Alpha=0,I_Beta=0,Id=0,Iq=0,Iq_Set=0,I_phase_sum=0 ;
volatile _iq Isa_sh=0,Isb_sh=0,Isc_sh=0,I_phase_sum_sh=0 ;
volatile _iq Isa_old = 0,Isb_old = 0,Isc_old= 0, Isa_d = 0,Isb_d = 0,Isc_d= 0;//Hallless

volatile int16_t Iq_ave=0 ;//电流平均值
int16_t Iq_ave_ave=0 ;
int16_t Current_lpf10=0 ;
volatile int32_t Iq_ref=0 ;//参考电流值
Channels Current_Offset=
{
    0,0,0 
}
;



/************力矩相关的全局变量**********************/
uint8_t FlagA=1,Flag_EN_ANG ;
uint8_t Flag_0toput=0 ;
uint8_t ANG_PIN;
uint8_t Flag_tor_sin_OK=0 ;//0启动传感器 超过2V  标志位
uint8_t Flag_tor_start=0 ;
int16_t Torque_AD_lpf=0 ;
int16_t Torque_AD_lpf_protect=0 ;
int16_t Torque_AD_lpf_comp=0 ;//用做比较的传感器滤波值
int16_t Torque_lpf10=0 ;
int32_t Torque_Set=0 ;
volatile _iq TORQUE_K_3=_IQ(1);
volatile _iq TORQUE_K_2=_IQ(0.75);
volatile _iq TORQUE_K_1=_IQ(0.5);

/************电压相关的全局变量**********************/
uint16_t Vdc_lpf10=0 ;
uint8_t Battery_level=5 ;//电池电量等级
uint16_t Power_value=_IQ(0.21);//功率限制值的变量
uint16_t U_delta=_IQ(0.15);//附加电压变量
uint8_t Flag_OV_start_ok=0 ;//开机过压标志位
uint8_t Flag_UV_start_ok=0 ;//开机欠压标志位


uint16_t PWM_double=0 ;
uint16_t PWM_max=0 ;
uint16_t PWM_half=0 ;

uint8_t Flag_over_E=0 ;

/************通信相关的全局变量**********************/
volatile uint32_t cnt_no_receive_data_panel=0,cnt_no_receive_data_bat=0 ;//在没有接收到数据的情况下递增
uint8_t Flag_test=0 ;
uint8_t Flagbat=0 ;

/*************保护有关全局变量***********************/
volatile uint8_t Flag_Pha_OC=0 ;//相电流过流标志
uint8_t Flag_warning=0 ;
uint32_t cnt_time_out=0 ;//未操作时计时
uint8_t Flag_run_state=0 ;//指示运行状态
/*************转速有关全局变量***********************/

uint8_t Dirspd=1 ;//电机转向标志位
uint16_t RPM_lpf=0 ;//电机转速，减速后
uint16_t RPM_lpf_10=0 ;
uint16_t speed_ave=0 ;//电机转速，减速前
volatile uint16_t bike_speed=0 ;//以公里为单位的车速
volatile uint16_t bike_speed_10=0 ;
volatile uint16_t Fre_hall=0 ;//霍尔频率
volatile uint16_t Fre_hall_10=0 ;//10倍霍尔频率
volatile uint16_t Fre_hall_ave=0 ;//平均霍尔频率
volatile uint16_t Fre_hall_10_ave=0 ;//10倍霍尔频率


uint8_t speed_low_flag=1 ;//低速标志位

/*************位置有关全局变量***********************/
volatile uint8_t Hall_state=0 ;//霍尔状态
volatile int32_t ang_cos=0,ang_sin=0 ;
volatile int32_t ang_cos_temp=0,ang_sin_temp=0 ;

volatile uint8_t Flag_hall_state_err=0 ;


/*************温度有关全局变量***********************/
uint16 temp_lpf=0 ;
uint16 temp_num=0 ;
int32_t Power_value_Temp=0 ;
uint8_t Flag_Temp_start=0 ;


/**************控制有关标志位************************/
volatile uint8_t Flag_main_cycle=0 ;
uint8_t Flag_temp_cut=0 ;
uint8_t Flag_phase_off=0 ;//相线断路标志位
uint8_t Flag_COM_good=0 ;//通信好标志位

uint8_t Flag_Light_INT;

uint8_t Flag_power_mode=0 ;
uint8_t Flag_brake=0 ;
uint8_t Flag_brake_com=0 ;
uint8_t Flag_brake_bike=0 ;
uint8_t Flag_light=0 ;
uint8_t Flag_LS=0 ;
uint8_t Flag_power_off=0 ;
volatile uint8_t Flag_switch_off=0 ;

uint8_t Flag_test_mode=0 ;



uint8_t temp_motor_low=0 ;//默认无短接
uint8_t Flag_bat_data_err=0 ;


/*************IO引脚电平的全局变量************************/
u8 PIN_Level_power=0 ;
u8 PIN_Level_light=0 ;
u8 PIN_Level_brake=0 ;

u8 PIN_Level_HU=0 ;
u8 PIN_Level_HV=0 ;
u8 PIN_Level_HW=0 ;

u8 PIN_Level_angle_pluse=0 ;
u8 PIN_Level_over_current=0 ;

/**************************************************************************/


/*****主循环控制标志位，TIM1中断中分频置位，主循环中清除******/
//

/************数据接收**********************/



/************全局函数定义**********************/
