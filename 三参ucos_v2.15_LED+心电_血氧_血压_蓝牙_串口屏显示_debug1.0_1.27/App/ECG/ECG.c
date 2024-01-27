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
#include "ECG.h"
#include "ADC.h"
#include "Filter.h"
#include "Calculate.h"
#include "Sprintf.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
//iir50hz�ݲ���ϵ��
extern  const int irr_50hz_notch_NL;
extern  const float irr_50hz_notch_num[3];
extern  const float irr_50hz_notch_den[3];

//iir80hz��ͨ�˲���ϵ��
extern const int irr_tcheb_80hz_lp_NL;
extern const float irr_tcheb_80hz_lp_num[9];
extern const float irr_tcheb_80hz_lp_den[9];

//�ĵ�50hz�ݲ���
static IRR_Filter_str ECG_50hzNotch;
//iir80hz��ͨ�˲���
static IRR_Filter_str ECG_IIR80hzLp;

//iir50hz�ݲ����˲�����ĵ�����
float ECGData;

//�����������
char ECG_ProbeStatus = FALSE;

////�ĵ��ź���ʾ����
//float show_arr[70];
static int s_i = 0;//������ļ�������

//�ĵ�����˲�����
OS_TCB g_tcbECG_SamplTask;               //������ƿ�
static CPU_STK s_arrECG_SamplStack[512]; //����ջ��

//�ĵ�̽ͷ�������
OS_TCB g_tcbECG_ProbeTask;               //������ƿ�
static CPU_STK s_arrECG_ProbeStack[256]; //����ջ��

/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/
static  void  ConfigECGGPIO(void);  //����LED��GPIO
static  void  ECG_SamplTask(void *pArg);
static  void  ECG_ProbeTask(void *pArg);
/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�ConfigECGGPIO
* �������ܣ�����ECG��GPIO 
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static  void  ConfigECGGPIO(void)
{
  //ʹ��RCU���ʱ��
  rcu_periph_clock_enable(RCU_GPIOC);
  
  //����ECG_ZERO���ţ�Ĭ��Ϊ�͵�ƽ--ʹ���ĵ����
  gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);  
  gpio_bit_reset(GPIOC, GPIO_PIN_13); 
  
  //����ECG_LEAD_OFF���ţ���������ģʽ������0--�������ӣ�����1--��������
  gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_5);  
}

/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�InitECG
* �������ܣ���ʼ��LEDģ��
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
void InitECG(void)
{
  //������Ϣ�ṹ��
  typedef struct
  {
    OS_TCB*      tcb;     //������ƿ�
    OS_TASK_PTR  func;    //�����ȿڵ�ַ
    CPU_CHAR*    name;    //��������
    OS_PRIO      prio;    //�������ȼ�
    CPU_STK*     stkBase; //����ջ������ַ
    CPU_STK_SIZE stkSize; //ջ����С
    OS_MSG_QTY   queSize; //�ڽ���Ϣ��������
  }StructTaskInfo;

  //�����б�
  StructTaskInfo taskInfo[] = 
  {
    {&g_tcbECG_SamplTask, ECG_SamplTask, "ECG_SamplTask", 11, s_arrECG_SamplStack, sizeof(s_arrECG_SamplStack) / sizeof(CPU_STK), 0},
    {&g_tcbECG_ProbeTask, ECG_ProbeTask, "ECG_ProbelTask", 30, s_arrECG_ProbeStack, sizeof(s_arrECG_ProbeStack) / sizeof(CPU_STK), 0},
  };
  
  //�ֲ�����
  unsigned int i;
  OS_ERR err;
  
  //Ӳ������
  ConfigECGGPIO();  //����LED��GPIO
  //����50hz�ݲ���
  CreatIRR_Filter_str(irr_50hz_notch_NL,irr_50hz_notch_num,irr_50hz_notch_den,&ECG_50hzNotch);
  //����IIR80hz��ͨ�˲���
  CreatIRR_Filter_str(irr_tcheb_80hz_lp_NL,irr_tcheb_80hz_lp_num,irr_tcheb_80hz_lp_den,&ECG_IIR80hzLp);
  
  //��������
  for(i = 0; i < sizeof(taskInfo) / sizeof(StructTaskInfo); i++)
  {
    OSTaskCreate((OS_TCB*)taskInfo[i].tcb,
                 (CPU_CHAR*)taskInfo[i].name,
                 (OS_TASK_PTR)taskInfo[i].func,
                 (void*)0,
                 (OS_PRIO)taskInfo[i].prio,
                 (CPU_STK*)taskInfo[i].stkBase,
                 (CPU_STK_SIZE)taskInfo[i].stkSize / 10,
                 (CPU_STK_SIZE)taskInfo[i].stkSize,
                 (OS_MSG_QTY)taskInfo[i].queSize,
                 (OS_TICK)0,
                 (void*)0,
                 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP,
                 (OS_ERR*)&err
                );

    //У��
    if(OS_ERR_NONE != err)
    {
      printf("Fail to create %s (%d)\r\n", taskInfo[i].name, err);
      while(1){}
    }
  }
  //��������Ĭ�Ϲ���
  OSTaskSuspend(&g_tcbECG_SamplTask,&err);
}

/*********************************************************************************************************
* �������ƣ�ECG_SamplTask
* �������ܣ��ĵ��źŲ����˲�����
* ���������cnt
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void ECG_SamplTask(void *pArg)
{
  OS_ERR err;
  float y;
  static int i = 0,j;
  static short ECG_Sampl_arr[500];
  char arr_printf[50];
  char arr_printf2[50];
  static short max,min;
  float Period;
  while(1)
  {
    y = IIR_Filter((float)GetEcg(),&ECG_50hzNotch);
    y = IIR_Filter(y,&ECG_IIR80hzLp);
//    printf("%d\r\n",GetEcg());
    //�����������������
    if(i>=500)//10msһ������500�����ݸ���5s���ʿɲ⵽24
    {
      i = 0;
      Period = FindPeriod_by_Peak(ECG_Sampl_arr,500,50,15);
      Period = 60/(Period*0.01);
//      printf("main.t7.txt=\"%d\"\xff\xff\xff",(int)Period);
      sprintf(arr_printf2,"main.t7.txt=\"%d\"\xff\xff\xff",(int)Period);
      printf_s_aus(arr_printf2);
      max = 0;
      min = 4095;
      for(j =0;j<500;j++)
      {
        max = FindMax(max,ECG_Sampl_arr[j]);
        min = FindMin(min,ECG_Sampl_arr[j]);
      }
    }
    
    ECG_Sampl_arr[i] = GetEcg();

    sprintf(arr_printf,"add main.s2.id,0,%d\xff\xff\xff",(int)(5+(GetEcg() - min)/(max*1.0-min)*40));
    printf_s_aus(arr_printf);
    
    i++;
    
    OSTimeDly(10,OS_OPT_TIME_PERIODIC, &err);
  }
}
/*********************************************************************************************************
* �������ƣ�ECG_ProbeTask
* �������ܣ��ĵ絼�����
* ���������cnt
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺0--�������ӣ�1--��������
*********************************************************************************************************/
static void ECG_ProbeTask(void *pArg)
{
  OS_ERR err;
  char arr_printf[100];
  while(1)
  {
    if(gpio_input_bit_get(GPIOC,GPIO_PIN_5))
    {
      //���������ӵ����䣬�����������
      if(ECG_ProbeStatus == TRUE)
      {
        //�����������
        OSTaskSuspend(&g_tcbECG_SamplTask,&err);
        
        sprintf(arr_printf,"main.t7.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf);
      }
      //��������
      ECG_ProbeStatus = FALSE;
    }
    else
    {
      //���������䵽���ӣ������������Ĺ���
      if(ECG_ProbeStatus == FALSE)
      {
        //��λ��������ı���--�������������50hz�ݲ���
        s_i = 0;
        InitIRR_Filter_str(&ECG_50hzNotch);
        InitIRR_Filter_str(&ECG_IIR80hzLp);
        //�����������Ĺ���
        OSTaskResume(&g_tcbECG_SamplTask,&err);
      }
      ECG_ProbeStatus = TRUE;
    }
    OSTimeDlyHMSM(0, 0, 0, 700, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
