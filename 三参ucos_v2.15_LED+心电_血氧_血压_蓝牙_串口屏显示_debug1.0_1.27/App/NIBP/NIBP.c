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
#include "NIBP.h"
#include "NIBP_HW_CFG.h"
#include "Filter.h"
#include "ADC.h"
#include "PolynomialFit.h"
#include "UART0.h"
#include "UART2.h"
#include "Sprintf.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/
#define THRE_NUM 11
//������߽��� 5��-x^5
#define FIT_OLDER 5

/*********************************************************************************************************
*                                              ö�ٽṹ��
*********************************************************************************************************/
typedef struct
{
  double h_pro;
  double l_pro;
}propor;
/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
static char NIBP_I = 0; //��ֵ�����������
static int NIBP_cnt = 0;//���������ʱ����
static char NIBP_flag = 0;//��ʼ������־λ

//�Ľ�ϵ��������ֵ������ϵ��
static double compare_arr[6] = {12,11,7,6,5,0};
static propor pro_arr[6]={0.52,0.78,0.57,0.78,0.58,0.78,0.64,0.78,0.64,0.60,0.64,0.50};

//ƽ��ѹ
double AVE_BP;
double H_BP;
double L_BP;

//�������ϵ��
static double coeff_arr[FIT_OLDER+1] = {0};

//ѹ����ֵ����
static short Pressure_arr_ther_M[THRE_NUM] = {162,145,134,123,114,106,99,92,84,76,70};  
static short Pressure_arr_ther_H[THRE_NUM] = {240,230,220,210,200,190,180,170,160,150,140};  

static short *Pressure_arr_ther = Pressure_arr_ther_M;

//ÿ����ֵ���ݵĲ�ֵ����
static double Pre_PP[THRE_NUM] = {0};
//ppֵ��Ӧ�����ѹֵ
static double cuff_arr[THRE_NUM] = {0};

//ѪѹУ׼ϵ��,��adֵ����ѹֵ
unsigned short ad1 = 2090;//100mmhgʱ��adֵ
unsigned short ad2 = 993; //0mmhgʱ��adֵ
float coef1;
float coef2;

//Ѫѹ������������״̬��־λ
static char MeasureTask_RunStatus = FALSE;

//��ѹ�������
OS_TCB g_tcbNIBP_PreMonitorTask;            //������ƿ�
static CPU_STK s_arrNIBP_PreMonitorStack[256]; //����ջ��

//��ʼ��������
OS_TCB g_tcbNIBP_StarTask;               //������ƿ�
static CPU_STK s_arrNIBP_StarStack[256]; //����ջ��

//Ѫѹ��������
OS_TCB g_tcbNIBP_MeasureTask;               //������ƿ�
static CPU_STK s_arrNIBP_MeasureStack[256]; //����ջ��

//��������������
OS_TCB g_tcbNIBP_PulseSamplTask;               //������ƿ�
static CPU_STK s_arrNIBP_PulseSamplStack[2048]; //����ջ��
/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/      
static float ADtoPre(unsigned short data);//��adֵװ��Ϊ���ѹ

static  void  NIBP_PreMonitorTask(void *pArg);  //��ѹ�������
static  void  NIBP_StarTask(void *pArg);        //��ʼ��������
static  void  NIBP_MeasureTask(void *pArg);     //Ѫѹ��������
static  void  NIBP_PulseSamplTask(void *pArg);  //��������������
/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�ConfigSw1Gpio
* �������ܣ�key1
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void ConfigSw1Gpio(void)
{
  //ʹ��RCU���ʱ��
  rcu_periph_clock_enable(RCU_GPIOB); //ʹ��GPIOB��ʱ��

  gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_14); //����PB14Ϊ��������
}

/*********************************************************************************************************
* �������ƣ�ConfigSw2EXTI
* �������ܣ�key1�ж�
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void ConfigSw21EXTI(void)
{
  rcu_periph_clock_enable(RCU_AF); //ʹ��AFʱ��
 
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_12); //����EXTI12��PB12
  
  exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_FALLING);  //�����ж���

  nvic_irq_enable(EXTI10_15_IRQn, 2, 2);       //ʹ���ⲿ�ж���EXTI0���������ȼ�

  exti_interrupt_flag_clear(EXTI_12); //���Line0�ϵ��жϱ�־λ
}
/*********************************************************************************************************
* �������ƣ�EXTI10_15_IRQHandler
* �������ܣ�EXTI10-15���жϷ�������12��ӦKEY1
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
void EXTI10_15_IRQHandler(void)
{
  static char s_cnt = 0;
  if(RESET != exti_interrupt_flag_get(EXTI_12)) 
  {
    if(s_cnt == 0)
    {
      ad2 = GetCuff();
      printf("0mmhg = %d\r\n",ad2);
    }
    else
    {
      ad1 = GetCuff();
      coef1 = ad2;
      coef2 = 100.00/(ad1 - ad2);
      s_cnt = 0;
      printf("100mmhg = %d\r\n",ad1);
    }
    s_cnt = (s_cnt+1)%2;
    exti_interrupt_flag_clear(EXTI_12);  //���Line0�ϵ��жϱ�־λ
  }
}
/*********************************************************************************************************
* �������ƣ�ADtoPre
* �������ܣ���adֵװ��Ϊ���ѹ
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static float ADtoPre(unsigned short data)
{
  return (data - coef1)*coef2;
}

/*********************************************************************************************************
* �������ƣ�FindMin
* �������ܣ�����Сֵ
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static short FindMin(short a,short b)
{
  if(a > b)
  {
    return b;
  }
  else
  {
    return a;
  }
}
/*********************************************************************************************************
* �������ƣ�FindMax
* �������ܣ������ֵ
* ���������void 
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static short FindMax(short a,short b)
{
  if(a > b)
  {
    return a;
  }
  else
  {
    return b;
  }
}
/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�InitNIBP
* �������ܣ���ʼ��NIBPģ��
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
void InitNIBP(void)
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
    {&g_tcbNIBP_StarTask, NIBP_StarTask, "NIBP_StarTask", 30, s_arrNIBP_StarStack, sizeof(s_arrNIBP_StarStack) / sizeof(CPU_STK), 0},
    {&g_tcbNIBP_PreMonitorTask, NIBP_PreMonitorTask, "NIBP_PreMonitorTask", 30, s_arrNIBP_PreMonitorStack, sizeof(s_arrNIBP_PreMonitorStack) / sizeof(CPU_STK), 0},
  };
  
  //�ֲ�����
  unsigned int i;
  OS_ERR err;
  //Ӳ������
  InitNIBP_HW_CFG();  
  ConfigSw21EXTI();
  ConfigSw1Gpio();
  
  //�������ѹϵ��
  coef1 = ad2;
  coef2 = 100.00/(ad1 - ad2);
  
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
}
/*********************************************************************************************************
* �������ƣ�NIBP_StarTask
* �������ܣ���ʼ��������
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static  void  NIBP_StarTask(void *pArg)
{
  OS_ERR err;
  unsigned char i;
  unsigned char* a = &i;
  
  char arr_printf[100];
  char arr_printf1[100];
  char arr_printf2[100];
  char arr_printf3[100];
  
  while(1)
  {
    i = '0';
    ReadUART2(a,1);
    //���û���ڲ���Ѫѹ
    if(MeasureTask_RunStatus == FALSE)
    {
      if(!gpio_input_bit_get(GPIOB,GPIO_PIN_14) || i == '#')//���sw1����
      {
        Pressure_arr_ther = Pressure_arr_ther_M;
        
        //��λ��ʾ����
        sprintf(arr_printf,"main.t12.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf);
        sprintf(arr_printf1,"main.t13.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf1);
        sprintf(arr_printf2,"main.t14.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf2);
        sprintf(arr_printf3,"main.t15.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf3);
        
        //��λ��ر���
        NIBP_I = 0;
        NIBP_cnt = 0;
        NIBP_flag = 0;
        
        //�ر�������������
        VALVE_F_CLOSE();
        VALVE_S_CLOSE();
        PUMU_ON();
        
        //��ʼ����Ѫѹ��������
        MeasureTask_RunStatus = TRUE;
        OSTaskCreate((OS_TCB*)&g_tcbNIBP_MeasureTask,
                 (CPU_CHAR*)"NIBP_StarTask",
                 (OS_TASK_PTR)NIBP_MeasureTask,
                 (void*)0,
                 (OS_PRIO)11,
                 (CPU_STK*)s_arrNIBP_MeasureStack,
                 (CPU_STK_SIZE)sizeof(s_arrNIBP_MeasureStack) / sizeof(CPU_STK) / 10,
                 (CPU_STK_SIZE)sizeof(s_arrNIBP_MeasureStack) / sizeof(CPU_STK),
                 (OS_MSG_QTY)0,
                 (OS_TICK)0,
                 (void*)0,
                 (OS_OPT)OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP,
                 (OS_ERR*)&err
                );
      }
    }
    else//���Ѫѹ��������������
    {
      if(i == '*')//����;�йر�
      {
        MeasureTask_RunStatus = FALSE;
        //���������ر�����
        VALVE_F_OPEN();
        VALVE_S_OPEN();
        PUMU_OFF();
        
        //��λ��ʾ����
        sprintf(arr_printf,"main.t12.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf);
        sprintf(arr_printf1,"main.t13.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf1);
        sprintf(arr_printf2,"main.t14.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf2);
        sprintf(arr_printf3,"main.t15.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf3);
        
        OSTaskDel(&g_tcbNIBP_MeasureTask,&err);
      }
    }
    OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}

/*********************************************************************************************************
* �������ƣ�NIBP_PreMonitorTask
* �������ܣ���ѹ�������
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static  void  NIBP_PreMonitorTask(void *pArg)
{
  OS_ERR err;
  while(1)
  {
    //�����ѹ����Ӳ����ѹ
    if(ADtoPre(GetCuff())>=299 || (!Protect_read()))
    {
      //�����������ɾ������
      if(MeasureTask_RunStatus == TRUE)
      {
        MeasureTask_RunStatus = FALSE;
        OSTaskDel(&g_tcbNIBP_MeasureTask,&err);
      }
      //�������ر�
      VALVE_F_CLOSE();
      VALVE_S_OPEN();
      PUMU_OFF();
    }
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}

/*********************************************************************************************************
* �������ƣ�NIBP_MeasureTask
* �������ܣ�Ѫѹ��������
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static  void  NIBP_MeasureTask(void *pArg)
{
  OS_ERR err;
  float cuff;         //���ѹ
  static short max,min;
  //��ѹ����ѹ��ƽ��ѹ��Ӧ�����������ֵ
  double h_data;
  double l_data;
  double hight;
  
  int j;
    
  char arr_printf[100];
  char arr_printf1[100];
  char arr_printf2[100];
  char arr_printf3[100];
  while(1)
  {
    cuff = ADtoPre(GetCuff());
    sprintf(arr_printf,"main.t12.txt=\"%d\"\xff\xff\xff",(int)cuff);
    printf_s_aus(arr_printf);
    //����/�����׶Σ������ѹȥ����ֵ
    if(NIBP_flag == 0)
    {
      if(NIBP_I == 0)
      {
        if(cuff>=Pressure_arr_ther[0])
        {
          //�ط����ر�
          PUMU_OFF();
          //���ѹ�Ѿ��ﵽҪ�󣬿��Կ�ʼ����
          NIBP_flag = 1;
        }
      }
      else
      {
        if(cuff<=Pressure_arr_ther[NIBP_I])
        {
          //�ط�
          VALVE_S_CLOSE();
          //���ѹ�Ѿ��ﵽҪ�󣬿��Կ�ʼ����
          NIBP_flag = 1;
        }
      }
    }
    //�Ѵﵽ��ֵ��׼��������
    if(NIBP_flag == 1)
    {
      NIBP_cnt++;
    }
    if(NIBP_cnt == 60)
    {
      //�������Ŵ�ʹ��
      PULSE_enable();
      //�������Сֵ��λ
      max = 0;
      min = 4095;
    }
    if(NIBP_cnt >= 60 && NIBP_cnt<200)
    {
      //����
      //�ҵ������Сֵ
      min = FindMin(min,GetPulse());
      max = FindMax(max,GetPulse());
    }
    //������ϣ���ȥ����һ����ֵ
    if(NIBP_cnt >= 250)
    {
      //������ֵ
      if(NIBP_I >= 1)
      {
        Pre_PP[NIBP_I-1] = (max-min)/10.0;
        cuff_arr[NIBP_I-1] = cuff/10.0;
      }
      //��λ��־����
      NIBP_flag = 0;
      NIBP_cnt = 0;
      //�������Ŵ�ʧ��
      PULSE_disable();
      //����
      VALVE_S_OPEN();
      
      //ȥ�����һ����ֵ����ʾ������ϣ������������������
      if(NIBP_I == THRE_NUM - 1)
      {
        VALVE_F_OPEN();
        
        //����Ѫѹֵ
        //����ʽ�������
        PolynomialFit(cuff_arr,Pre_PP,coeff_arr,THRE_NUM-1,FIT_OLDER);
        //Ѱ�ҳ�ƽ��ѹ--���߷�ֵ��Ӧ��xֵ
        AVE_BP = FindFirstMaxInPolynomial(coeff_arr,cuff_arr[1],cuff_arr[THRE_NUM-1],FIT_OLDER,0.05);
        hight = CalcPolynomial(coeff_arr,AVE_BP,FIT_OLDER);
        //���øĽ�ϵ����������ѹ������ѹ��Ӧ�����������ֵ�������
        for(j = 0;j<6;j++)
        {
          if(AVE_BP > compare_arr[j])
          {
            break;
          }
        }
        h_data = hight * pro_arr[j].h_pro;
        l_data = hight * pro_arr[j].l_pro;
        
        //���ݷ��ֵ��������ѹ������ѹ
        H_BP = Polynomia_y_to_x(coeff_arr,AVE_BP,AVE_BP+4,FIT_OLDER,h_data,0.01,10);
        L_BP = Polynomia_y_to_x(coeff_arr,AVE_BP,AVE_BP-4,FIT_OLDER,l_data,0.01,10);
        
        sprintf(arr_printf1,"main.t13.txt=\"%d\"\xff\xff\xff",(int)(H_BP*10));
        printf_s_aus(arr_printf1);
        sprintf(arr_printf2,"main.t14.txt=\"%d\"\xff\xff\xff",(int)(AVE_BP*10));
        printf_s_aus(arr_printf2);
        sprintf(arr_printf3,"main.t15.txt=\"%d\"\xff\xff\xff",(int)(L_BP*10));
        printf_s_aus(arr_printf3);
        
        //��������(ɾ������)
        MeasureTask_RunStatus = FALSE;
        OSTaskDel(&g_tcbNIBP_MeasureTask,&err);
      }
      NIBP_I = (NIBP_I+1)%THRE_NUM;
      
      //����Ƿ��ѹ�����ǣ��л���ֵ���飬���²���
      if(NIBP_I == 1 && (max-min)/10.0 > 120)
      {
        Pressure_arr_ther = Pressure_arr_ther_H;
        NIBP_I = 0;
        VALVE_S_CLOSE();
        PUMU_ON();
      }
    }

    OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}



