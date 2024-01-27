/*********************************************************************************************************
* ģ�����ƣ�LED.c
* ժ    Ҫ��LEDģ��
* ��ǰ�汾��1.0.0
* ��    �ߣ�Leyutek(COPYRIGHT 2018 - 2021 Leyutek. All rights reserved.)
* ������ڣ�2021��07��01��
* ��    �ݣ�
* ע    �⣺
**********************************************************************************************************
* ȡ���汾��
* ��    �ߣ�
* ������ڣ�
* �޸����ݣ�
* �޸��ļ���
*********************************************************************************************************/

/*********************************************************************************************************
*                                              ����ͷ�ļ�
*********************************************************************************************************/
#include "SPO2_HW_CFG.h"
#include "ADC.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              ö�ٽṹ��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
//��������dacֵ
static unsigned short ir_DA_Val = 100;
static unsigned short red_DA_Val = 100;

//��������źŵ�adcֵ
static unsigned short SPO2_IRData = 0;
static unsigned short SPO2_REDData = 0;
/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/
static void ConfigCSGPIO(void);  //���ú������Ƶ�GPIO
static void ConfigTimer3(unsigned short arr, unsigned short psc);
/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�ConfigCSGPIO
* �������ܣ�����IR_RED��GPIO 
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void ConfigCSGPIO(void)
{
  //ʹ��RCU���ʱ��
  rcu_periph_clock_enable(RCU_GPIOB);                                 //ʹ��GPIOA��ʱ��
  
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);  //����GPIO���ģʽ���ٶ�
  gpio_bit_reset(GPIOB, GPIO_PIN_6);                                    //��LED1Ĭ��״̬����ΪϨ��
  
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);  //����GPIO���ģʽ���ٶ�
  gpio_bit_reset(GPIOB, GPIO_PIN_7);                                  //��LED2Ĭ��״̬����ΪϨ��
}
/*********************************************************************************************************
* �������ƣ�ConfigDACGPIO
* �������ܣ�����IR_RED��GPIO 
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void ConfigDACGPIO(void)
{
  //ʹ��RCU���ʱ��
  rcu_periph_clock_enable(RCU_GPIOA);  //ʹ��GPIOA��ʱ��
  rcu_periph_clock_enable(RCU_DAC);    //ʹ��DAC��ʱ��
  
  //����GPIO���ģʽ���ٶ�
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

  //DAC0����
  dac_deinit();                                         //��λDACģ��
  dac_concurrent_disable();                             //����concurrent mode
  dac_output_buffer_enable(DAC0);                       //ʹ�����������
  dac_trigger_disable(DAC0);                             //�����ⲿ����Դ
  dac_wave_mode_config(DAC0, DAC_WAVE_DISABLE);         //����Wave mode

  //ʹ��DAC0
  dac_enable(DAC0);
}
/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�InitSPO2_HW_CFG
* �������ܣ���ʼ��SPO2Ӳ����ʱ������ģ��
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
void InitSPO2_HW_CFG(void)
{
  ConfigCSGPIO();  //����IR_RED��GPIO
  ConfigDACGPIO();
  ConfigTimer3(999, 59);  //120MHz/(119+1)=1MHz����0������499Ϊ0.5ms
}

/*********************************************************************************************************
* �������ƣ�IR/RED_ON/OFF
* �������ܣ��������ƵĿ���
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void IR_ON(void)
{
  gpio_bit_reset(GPIOB, GPIO_PIN_6);
}

static void IR_OFF(void)
{
  gpio_bit_set(GPIOB, GPIO_PIN_6);
}

static void RED_ON(void)
{
  gpio_bit_reset(GPIOB, GPIO_PIN_7);
}

static void RED_OFF(void)
{
  gpio_bit_set(GPIOB, GPIO_PIN_7);
}
/*********************************************************************************************************
* �������ƣ�ConfigTimer3
* �������ܣ�����TIMER3 
* ���������arr-�Զ���װֵ��psc-Ԥ��Ƶ��ֵ
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺APB1ʱ��Ϊ60MHz�����TIMER3ʱ��Ϊ120MHz
*********************************************************************************************************/
static void ConfigTimer3(unsigned short arr, unsigned short psc)
{
  timer_parameter_struct timer_initpara;               //timer_initpara���ڴ�Ŷ�ʱ���Ĳ���

  //ʹ��RCU���ʱ�� 
  rcu_periph_clock_enable(RCU_TIMER3);                 //ʹ��TIMER2��ʱ��

  //��λTIMER2
  timer_deinit(TIMER3);                                //����TIMER2�����ָ�Ĭ��ֵ
  timer_struct_para_init(&timer_initpara);             //��ʼ��timer_initpara

  //����TIMER2
  timer_initpara.prescaler         = psc;              //����Ԥ��Ƶ��ֵ
  timer_initpara.counterdirection  = TIMER_COUNTER_UP; //�������ϼ���ģʽ
  timer_initpara.period            = arr;              //�����Զ���װ��ֵ
  timer_initpara.clockdivision     = TIMER_CKDIV_DIV1; //����ʱ�ӷָ�
  timer_init(TIMER3, &timer_initpara);                 //���ݲ�����ʼ����ʱ��

  //ʹ�ܶ�ʱ�������ж�
  timer_interrupt_enable(TIMER3, TIMER_INT_UP);        //ʹ�ܶ�ʱ���ĸ����ж�
  nvic_irq_enable(TIMER3_IRQn, 3, 0);                  //����NVIC�������ȼ�
  timer_enable(TIMER3);                                //ʹ�ܶ�ʱ��
}
/*********************************************************************************************************
* �������ƣ�TIMER3_IRQHandler
* �������ܣ�TIMER3�жϷ�����������SPO2ʱ��
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺0.5ms����һ��
*********************************************************************************************************/
void TIMER3_IRQHandler(void)
{
  static int s_cnt = 0;
  if(timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP) == SET)   //�ж϶�ʱ�������ж��Ƿ���
  {
    timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);         //�����ʱ�������жϱ�־
  }

  if(s_cnt == 2)
  {
    dac_data_set(DAC0,DAC_ALIGN_12B_R,red_DA_Val);
    RED_ON();
  }
  else if(s_cnt == 5)
  {
    SPO2_REDData = GetSPO2();
  }
  else if(s_cnt == 6)
  {
    RED_OFF();
  }
  else if(s_cnt == 10)
  {
    dac_data_set(DAC0,DAC_ALIGN_12B_R,ir_DA_Val);
    IR_ON();
  }
  else if(s_cnt == 13)
  {
    SPO2_IRData = GetSPO2();
  }
  else if(s_cnt == 14)
  {
    IR_OFF();
  }
  s_cnt = (s_cnt+1)%20;
}

unsigned short GetSPO2_IRData(void)
{
  return SPO2_IRData;
}

unsigned short GetSPO2_REDData(void)
{
  return SPO2_REDData;
}

void SetRed_DA_Val(unsigned short value)
{
  red_DA_Val = value;
}

void SetIr_DA_Val(unsigned short value)
{
  ir_DA_Val = value;
}
