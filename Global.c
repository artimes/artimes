/**********************�����õ���ȫ�ֱ���**********************************/
#include "Global.h" 


/************������ص�ȫ�ֱ���**********************/

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

volatile int16_t Iq_ave=0 ;//����ƽ��ֵ
int16_t Iq_ave_ave=0 ;
int16_t Current_lpf10=0 ;
volatile int32_t Iq_ref=0 ;//�ο�����ֵ
Channels Current_Offset=
{
    0,0,0 
}
;



/************������ص�ȫ�ֱ���**********************/
uint8_t FlagA=1,Flag_EN_ANG ;
uint8_t Flag_0toput=0 ;
uint8_t ANG_PIN;
uint8_t Flag_tor_sin_OK=0 ;//0���������� ����2V  ��־λ
uint8_t Flag_tor_start=0 ;
int16_t Torque_AD_lpf=0 ;
int16_t Torque_AD_lpf_protect=0 ;
int16_t Torque_AD_lpf_comp=0 ;//�����ȽϵĴ������˲�ֵ
int16_t Torque_lpf10=0 ;
int32_t Torque_Set=0 ;
volatile _iq TORQUE_K_3=_IQ(1);
volatile _iq TORQUE_K_2=_IQ(0.75);
volatile _iq TORQUE_K_1=_IQ(0.5);

/************��ѹ��ص�ȫ�ֱ���**********************/
uint16_t Vdc_lpf10=0 ;
uint8_t Battery_level=5 ;//��ص����ȼ�
uint16_t Power_value=_IQ(0.21);//��������ֵ�ı���
uint16_t U_delta=_IQ(0.15);//���ӵ�ѹ����
uint8_t Flag_OV_start_ok=0 ;//������ѹ��־λ
uint8_t Flag_UV_start_ok=0 ;//����Ƿѹ��־λ


uint16_t PWM_double=0 ;
uint16_t PWM_max=0 ;
uint16_t PWM_half=0 ;

uint8_t Flag_over_E=0 ;

/************ͨ����ص�ȫ�ֱ���**********************/
volatile uint32_t cnt_no_receive_data_panel=0,cnt_no_receive_data_bat=0 ;//��û�н��յ����ݵ�����µ���
uint8_t Flag_test=0 ;
uint8_t Flagbat=0 ;

/*************�����й�ȫ�ֱ���***********************/
volatile uint8_t Flag_Pha_OC=0 ;//�����������־
uint8_t Flag_warning=0 ;
uint32_t cnt_time_out=0 ;//δ����ʱ��ʱ
uint8_t Flag_run_state=0 ;//ָʾ����״̬
/*************ת���й�ȫ�ֱ���***********************/

uint8_t Dirspd=1 ;//���ת���־λ
uint16_t RPM_lpf=0 ;//���ת�٣����ٺ�
uint16_t RPM_lpf_10=0 ;
uint16_t speed_ave=0 ;//���ת�٣�����ǰ
volatile uint16_t bike_speed=0 ;//�Թ���Ϊ��λ�ĳ���
volatile uint16_t bike_speed_10=0 ;
volatile uint16_t Fre_hall=0 ;//����Ƶ��
volatile uint16_t Fre_hall_10=0 ;//10������Ƶ��
volatile uint16_t Fre_hall_ave=0 ;//ƽ������Ƶ��
volatile uint16_t Fre_hall_10_ave=0 ;//10������Ƶ��


uint8_t speed_low_flag=1 ;//���ٱ�־λ

/*************λ���й�ȫ�ֱ���***********************/
volatile uint8_t Hall_state=0 ;//����״̬
volatile int32_t ang_cos=0,ang_sin=0 ;
volatile int32_t ang_cos_temp=0,ang_sin_temp=0 ;

volatile uint8_t Flag_hall_state_err=0 ;


/*************�¶��й�ȫ�ֱ���***********************/
uint16 temp_lpf=0 ;
uint16 temp_num=0 ;
int32_t Power_value_Temp=0 ;
uint8_t Flag_Temp_start=0 ;


/**************�����йر�־λ************************/
volatile uint8_t Flag_main_cycle=0 ;
uint8_t Flag_temp_cut=0 ;
uint8_t Flag_phase_off=0 ;//���߶�·��־λ
uint8_t Flag_COM_good=0 ;//ͨ�źñ�־λ

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



uint8_t temp_motor_low=0 ;//Ĭ���޶̽�
uint8_t Flag_bat_data_err=0 ;


/*************IO���ŵ�ƽ��ȫ�ֱ���************************/
u8 PIN_Level_power=0 ;
u8 PIN_Level_light=0 ;
u8 PIN_Level_brake=0 ;

u8 PIN_Level_HU=0 ;
u8 PIN_Level_HV=0 ;
u8 PIN_Level_HW=0 ;

u8 PIN_Level_angle_pluse=0 ;
u8 PIN_Level_over_current=0 ;

/**************************************************************************/


/*****��ѭ�����Ʊ�־λ��TIM1�ж��з�Ƶ��λ����ѭ�������******/
//

/************���ݽ���**********************/



/************ȫ�ֺ�������**********************/
