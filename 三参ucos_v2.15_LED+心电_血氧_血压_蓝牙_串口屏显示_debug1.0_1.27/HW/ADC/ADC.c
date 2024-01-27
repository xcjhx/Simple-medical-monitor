/*********************************************************************************************************
* 模块名称：ADC.c
* 摘    要：ADC模块
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
#include "ADC.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define ADC_CH_NUM 5 //通道数量，即一共有多少路ADC输入
#define ADC_CH_LEN 1 //通道长度，即一个ADC通道要采集多少个数据

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
static unsigned short s_arrADCBuf[ADC_CH_NUM][ADC_CH_LEN]; //ADC数据缓冲区，由DMA自动搬运

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void ConfigDMA0CH0ForADC0(void); //配置DMA0的通道0
static void ConfigADC0(void);           //配置ADC0

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigDMA0CH0ForADC0
* 函数功能：配置DMA
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注  意：
**********************************************************************************************************/
static void ConfigDMA0CH0ForADC0(void)
{
  //DMA初始化结构体
  dma_parameter_struct dma_init_struct;

  //DMA配置
  rcu_periph_clock_enable(RCU_DMA0);                           //使能DMA0时钟
  dma_deinit(DMA0, DMA_CH0);                                   //初始化结构体设置默认值
  dma_init_struct.direction  = DMA_PERIPHERAL_TO_MEMORY;       //设置DMA数据传输方向
  dma_init_struct.memory_addr  = (uint32_t)s_arrADCBuf;        //内存地址设置
  dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;   //内存增长使能
  dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;       //内存数据位数设置
  dma_init_struct.number       = ADC_CH_NUM * ADC_CH_LEN;      //内存数据量设置
  dma_init_struct.periph_addr  = (uint32_t)&(ADC_RDATA(ADC0)); //外设地址设置
  dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;  //外设地址增长失能
  dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;   //外设数据位数设置
  dma_init_struct.priority   = DMA_PRIORITY_ULTRA_HIGH;        //优先级设置
  dma_init(DMA0, DMA_CH0, &dma_init_struct);                   //初始化结构体

  //DMA模式设置
  dma_circulation_enable(DMA0, DMA_CH0);             //使能循环
  dma_memory_to_memory_disable(DMA0, DMA_CH0);       //失能内存到内存

  //使能DMA
  dma_channel_enable(DMA0, DMA_CH0);
}

/*********************************************************************************************************
* 函数名称：ConfigADC0
* 函数功能：初始化ADC0设备
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注  意：
**********************************************************************************************************/
static void ConfigADC0(void)
{
  //配置ADC时钟源
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);

  //GPIO配置
  rcu_periph_clock_enable(RCU_GPIOA);                             //使能GPIOA时钟
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5); //ADC0引脚配置（PA5）SPO2_VSIG ADC01_IN5
  
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_6); //ADC0引脚配置（PA6）测电池电量 ADC01_IN6
  
  rcu_periph_clock_enable(RCU_GPIOB);                             //使能GPIOB时钟
  gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0); //ADC0引脚配置（PB0）NIBP ADC01_IN8
  gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1); //ADC0引脚配置（PB1）NIBP_PULSE ADC01_IN9
  
  rcu_periph_clock_enable(RCU_GPIOC);                             //使能GPIOC时钟
  gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4); //ADC0引脚配置（PC4）NIBP ADC01_IN8

  //所有ADC独立工作
  adc_mode_config(ADC_MODE_FREE);

  //配置ADC0
  rcu_periph_clock_enable(RCU_ADC0);                                                            //使能ADC0的时钟
  adc_deinit(ADC0);                                                                             //复位ADC0
  adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);                                     //使能ADC扫描，即开启多通道转换
  adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);                               //使能连续采样
  adc_resolution_config(ADC0, ADC_RESOLUTION_12B);                                              //规则组配置，12位分辨率
  adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);                                         //右对齐
  adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, ADC_CH_NUM);                             //规则组长度，即ADC通道数量
  adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);                               //规则组使能外部触发
  adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); //规则组使用软件触发
  adc_oversample_mode_disable(ADC0);                                                            //禁用过采样

  //通道采样顺序配置，采样顺序从0开始 //ADC频率20Mhz 单通道采样时间12.6us，单次循环采样时间为12.6*5=63us，低频采样误差忽略不计；
  //故adc是一直开着的，任务在需要时再到数据缓冲区中读取数据；（5、8、9、14、6）
  adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_5, ADC_SAMPLETIME_239POINT5);
  adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_8, ADC_SAMPLETIME_239POINT5);
  adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_9, ADC_SAMPLETIME_239POINT5);
  adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_14, ADC_SAMPLETIME_239POINT5);
  adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5);
  
  //ADC0使能
  adc_enable(ADC0);

  //使能ADC0校准
  adc_calibration_enable(ADC0);

  //使能DMA
  adc_dma_mode_enable(ADC0);

  //规则组软件触发
  adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitADC
* 函数功能：初始化ADC模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注  意：
**********************************************************************************************************/
void InitADC(void)
{
  //DMA0CH0配置
  ConfigDMA0CH0ForADC0();

  //ADC0配置
  ConfigADC0();
}

/*********************************************************************************************************
* 函数名称：GetSPO2
* 函数功能：获取SPO2ADC转换值
* 输入参数：void
* 输出参数：void
* 返 回 值：ADC转换值
* 创建日期：2021年07月01日
* 注  意：
**********************************************************************************************************/
unsigned short GetSPO2(void)
{
  return s_arrADCBuf[0][0];
}

unsigned short GetCuff(void)
{
  return s_arrADCBuf[1][0];
}

unsigned short GetPulse(void)
{
  return s_arrADCBuf[2][0];
}

unsigned short GetEcg(void)
{
  return s_arrADCBuf[3][0];
}

unsigned short GetVpro(void)
{
  return s_arrADCBuf[4][0];
}
