/*********************************************************************************************************
* 模块名称：LED.c
* 摘    要：LED模块
* 当前版本：1.0.0
* 作    者：Leyutek(COPYRIGHT 2018 - 2021 Leyutek. All rights reserved.)
* 完成日期：2021年07月01日
* 内    容：
* 注    意：
**********************************************************************************************************
* 取代版本：
* 作    者：
* 完成日期：
* 修改内容：
* 修改文件：
*********************************************************************************************************/

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "SPO2_HW_CFG.h"
#include "ADC.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
//红与红外的dac值
static unsigned short ir_DA_Val = 100;
static unsigned short red_DA_Val = 100;

//红与红外信号的adc值
static unsigned short SPO2_IRData = 0;
static unsigned short SPO2_REDData = 0;
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void ConfigCSGPIO(void);  //配置红外与红灯的GPIO
static void ConfigTimer3(unsigned short arr, unsigned short psc);
/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigCSGPIO
* 函数功能：配置IR_RED的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static void ConfigCSGPIO(void)
{
  //使能RCU相关时钟
  rcu_periph_clock_enable(RCU_GPIOB);                                 //使能GPIOA的时钟
  
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);  //设置GPIO输出模式及速度
  gpio_bit_reset(GPIOB, GPIO_PIN_6);                                    //将LED1默认状态设置为熄灭
  
  gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);  //设置GPIO输出模式及速度
  gpio_bit_reset(GPIOB, GPIO_PIN_7);                                  //将LED2默认状态设置为熄灭
}
/*********************************************************************************************************
* 函数名称：ConfigDACGPIO
* 函数功能：配置IR_RED的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static void ConfigDACGPIO(void)
{
  //使能RCU相关时钟
  rcu_periph_clock_enable(RCU_GPIOA);  //使能GPIOA的时钟
  rcu_periph_clock_enable(RCU_DAC);    //使能DAC的时钟
  
  //设置GPIO输出模式及速度
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);

  //DAC0配置
  dac_deinit();                                         //复位DAC模块
  dac_concurrent_disable();                             //禁用concurrent mode
  dac_output_buffer_enable(DAC0);                       //使能输出缓冲区
  dac_trigger_disable(DAC0);                             //禁用外部触发源
  dac_wave_mode_config(DAC0, DAC_WAVE_DISABLE);         //禁用Wave mode

  //使能DAC0
  dac_enable(DAC0);
}
/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitSPO2_HW_CFG
* 函数功能：初始化SPO2硬件、时序配置模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void InitSPO2_HW_CFG(void)
{
  ConfigCSGPIO();  //配置IR_RED的GPIO
  ConfigDACGPIO();
  ConfigTimer3(999, 59);  //120MHz/(119+1)=1MHz，由0计数到499为0.5ms
}

/*********************************************************************************************************
* 函数名称：IR/RED_ON/OFF
* 函数功能：红外与红灯的开关
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
* 函数名称：ConfigTimer3
* 函数功能：配置TIMER3 
* 输入参数：arr-自动重装值，psc-预分频器值
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：APB1时钟为60MHz，因此TIMER3时钟为120MHz
*********************************************************************************************************/
static void ConfigTimer3(unsigned short arr, unsigned short psc)
{
  timer_parameter_struct timer_initpara;               //timer_initpara用于存放定时器的参数

  //使能RCU相关时钟 
  rcu_periph_clock_enable(RCU_TIMER3);                 //使能TIMER2的时钟

  //复位TIMER2
  timer_deinit(TIMER3);                                //设置TIMER2参数恢复默认值
  timer_struct_para_init(&timer_initpara);             //初始化timer_initpara

  //配置TIMER2
  timer_initpara.prescaler         = psc;              //设置预分频器值
  timer_initpara.counterdirection  = TIMER_COUNTER_UP; //设置向上计数模式
  timer_initpara.period            = arr;              //设置自动重装载值
  timer_initpara.clockdivision     = TIMER_CKDIV_DIV1; //设置时钟分割
  timer_init(TIMER3, &timer_initpara);                 //根据参数初始化定时器

  //使能定时器及其中断
  timer_interrupt_enable(TIMER3, TIMER_INT_UP);        //使能定时器的更新中断
  nvic_irq_enable(TIMER3_IRQn, 3, 0);                  //配置NVIC设置优先级
  timer_enable(TIMER3);                                //使能定时器
}
/*********************************************************************************************************
* 函数名称：TIMER3_IRQHandler
* 函数功能：TIMER3中断服务函数，控制SPO2时序
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：0.5ms进入一次
*********************************************************************************************************/
void TIMER3_IRQHandler(void)
{
  static int s_cnt = 0;
  if(timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP) == SET)   //判断定时器更新中断是否发生
  {
    timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);         //清除定时器更新中断标志
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
