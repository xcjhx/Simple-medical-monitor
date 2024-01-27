/*********************************************************************************************************
* ģ�����ƣ�ADC.c
* ժ    Ҫ��ADCģ��
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
#include "ADC.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/
#define ADC_CH_NUM 5 //ͨ����������һ���ж���·ADC����
#define ADC_CH_LEN 1 //ͨ�����ȣ���һ��ADCͨ��Ҫ�ɼ����ٸ�����

/*********************************************************************************************************
*                                              ö�ٽṹ��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
static unsigned short s_arrADCBuf[ADC_CH_NUM][ADC_CH_LEN]; //ADC���ݻ���������DMA�Զ�����

/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/
static void ConfigDMA0CH0ForADC0(void); //����DMA0��ͨ��0
static void ConfigADC0(void);           //����ADC0

/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�ConfigDMA0CH0ForADC0
* �������ܣ�����DMA
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע  �⣺
**********************************************************************************************************/
static void ConfigDMA0CH0ForADC0(void)
{
  //DMA��ʼ���ṹ��
  dma_parameter_struct dma_init_struct;

  //DMA����
  rcu_periph_clock_enable(RCU_DMA0);                           //ʹ��DMA0ʱ��
  dma_deinit(DMA0, DMA_CH0);                                   //��ʼ���ṹ������Ĭ��ֵ
  dma_init_struct.direction  = DMA_PERIPHERAL_TO_MEMORY;       //����DMA���ݴ��䷽��
  dma_init_struct.memory_addr  = (uint32_t)s_arrADCBuf;        //�ڴ��ַ����
  dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;   //�ڴ�����ʹ��
  dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;       //�ڴ�����λ������
  dma_init_struct.number       = ADC_CH_NUM * ADC_CH_LEN;      //�ڴ�����������
  dma_init_struct.periph_addr  = (uint32_t)&(ADC_RDATA(ADC0)); //�����ַ����
  dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;  //�����ַ����ʧ��
  dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;   //��������λ������
  dma_init_struct.priority   = DMA_PRIORITY_ULTRA_HIGH;        //���ȼ�����
  dma_init(DMA0, DMA_CH0, &dma_init_struct);                   //��ʼ���ṹ��

  //DMAģʽ����
  dma_circulation_enable(DMA0, DMA_CH0);             //ʹ��ѭ��
  dma_memory_to_memory_disable(DMA0, DMA_CH0);       //ʧ���ڴ浽�ڴ�

  //ʹ��DMA
  dma_channel_enable(DMA0, DMA_CH0);
}

/*********************************************************************************************************
* �������ƣ�ConfigADC0
* �������ܣ���ʼ��ADC0�豸
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע  �⣺
**********************************************************************************************************/
static void ConfigADC0(void)
{
  //����ADCʱ��Դ
  rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);

  //GPIO����
  rcu_periph_clock_enable(RCU_GPIOA);                             //ʹ��GPIOAʱ��
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5); //ADC0�������ã�PA5��SPO2_VSIG ADC01_IN5
  
  gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_6); //ADC0�������ã�PA6�����ص��� ADC01_IN6
  
  rcu_periph_clock_enable(RCU_GPIOB);                             //ʹ��GPIOBʱ��
  gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0); //ADC0�������ã�PB0��NIBP ADC01_IN8
  gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1); //ADC0�������ã�PB1��NIBP_PULSE ADC01_IN9
  
  rcu_periph_clock_enable(RCU_GPIOC);                             //ʹ��GPIOCʱ��
  gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4); //ADC0�������ã�PC4��NIBP ADC01_IN8

  //����ADC��������
  adc_mode_config(ADC_MODE_FREE);

  //����ADC0
  rcu_periph_clock_enable(RCU_ADC0);                                                            //ʹ��ADC0��ʱ��
  adc_deinit(ADC0);                                                                             //��λADC0
  adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);                                     //ʹ��ADCɨ�裬��������ͨ��ת��
  adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);                               //ʹ����������
  adc_resolution_config(ADC0, ADC_RESOLUTION_12B);                                              //���������ã�12λ�ֱ���
  adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);                                         //�Ҷ���
  adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, ADC_CH_NUM);                             //�����鳤�ȣ���ADCͨ������
  adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);                               //������ʹ���ⲿ����
  adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); //������ʹ���������
  adc_oversample_mode_disable(ADC0);                                                            //���ù�����

  //ͨ������˳�����ã�����˳���0��ʼ //ADCƵ��20Mhz ��ͨ������ʱ��12.6us������ѭ������ʱ��Ϊ12.6*5=63us����Ƶ���������Բ��ƣ�
  //��adc��һֱ���ŵģ���������Ҫʱ�ٵ����ݻ������ж�ȡ���ݣ���5��8��9��14��6��
  adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_5, ADC_SAMPLETIME_239POINT5);
  adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_8, ADC_SAMPLETIME_239POINT5);
  adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_9, ADC_SAMPLETIME_239POINT5);
  adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_14, ADC_SAMPLETIME_239POINT5);
  adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_6, ADC_SAMPLETIME_239POINT5);
  
  //ADC0ʹ��
  adc_enable(ADC0);

  //ʹ��ADC0У׼
  adc_calibration_enable(ADC0);

  //ʹ��DMA
  adc_dma_mode_enable(ADC0);

  //�������������
  adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
}

/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�InitADC
* �������ܣ���ʼ��ADCģ��
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע  �⣺
**********************************************************************************************************/
void InitADC(void)
{
  //DMA0CH0����
  ConfigDMA0CH0ForADC0();

  //ADC0����
  ConfigADC0();
}

/*********************************************************************************************************
* �������ƣ�GetSPO2
* �������ܣ���ȡSPO2ADCת��ֵ
* ���������void
* ���������void
* �� �� ֵ��ADCת��ֵ
* �������ڣ�2021��07��01��
* ע  �⣺
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
