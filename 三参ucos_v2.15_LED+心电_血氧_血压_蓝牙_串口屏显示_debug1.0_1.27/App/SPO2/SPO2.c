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
#include "SPO2.h"
#include "Filter.h"
#include "ADC.h"
#include "Calculate.h"
#include "Sprintf.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/

#define IR_CENTRAL_LINE  900 //��������ɹ�����
#define IR_FINE_ADJ_OFFSET  200   //�����ϸ������ƫ������ֵ

#define RED_CENTRAL_LINE 700 //������ɹ�����
#define RED_FINE_ADJ_OFFSET 200  //���ϸ������ƫ������ֵ

#define IR_INIT_VAL 120
#define RED_INIT_VAL 80
/*********************************************************************************************************
*                                              ö�ٽṹ��
*********************************************************************************************************/

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
//iir20hz��ͨ�˲���ϵ��
extern const int f125_15_20LP_NL;
extern const float f125_15_20LP_num[18];
extern const float f125_15_20LP_den[18];

//iir20hz��ͨ�˲���
static IRR_Filter_str SPO2_IR_f125_15_20hzlp;
static IRR_Filter_str SPO2_RED_f125_15_20hzlp;

//Rֵ��
static float R_arr[10] = {0.480,0.540,0.570,0.600,0.635,0.655,0.690,0.720,0.750,0.777};

//�����������
static char SPO2_ProbeStatus = FALSE;

//��/����������������״̬��־λ
static char REDAdjTask_runStatus = FALSE;
static char IRAdjTask_runStatus = FALSE;

//��/��������ɹ���־λ
static char REDAdj_Suc = FALSE;
static char IRAdj_Suc = FALSE;

//������������״̬��־λ
static char SamplTask_runStatus = FALSE;

//��/������dacֵ
static unsigned short red_DA_Val = RED_INIT_VAL;
static unsigned short ir_DA_Val = IR_INIT_VAL;

//��/������������
static char s_RedCnt = 0;
static char s_IrCnt = 0;

//Ѫ��̽ͷ�������
OS_TCB g_tcbSPO2_ProbeTask;               //������ƿ�
static CPU_STK s_arrSPO2_ProbeStack[128]; //����ջ��

//����������
OS_TCB g_tcbRED_AdjTask;               //������ƿ�
static CPU_STK s_arrRED_AdjStack[256]; //����ջ��

//������������
OS_TCB g_tcbIR_AdjTask;               //������ƿ�
static CPU_STK s_arrIR_AdjStack[256]; //����ջ��

//�ĵ�����˲�����
OS_TCB g_tcbSPO2_SamplTask;               //������ƿ�
static CPU_STK s_arrSPO2_SamplStack[1024]; //����ջ��

/*********************************************************************************************************
*                                              �ڲ���������
*********************************************************************************************************/

/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
static  void  SPO2_ProbeTask(void *pArg);   //̽ͷ���
static  void  RED_AdjTask(void *pArg);       //������
static  void  IR_AdjTask(void *pArg);       //��������
static  void  SPO2_SamplTask(void *pArg);   //Ѫ���źŲ������˲�  

/*********************************************************************************************************
*                                              API����ʵ��
*********************************************************************************************************/
/*********************************************************************************************************
* �������ƣ�InitSPO2
* �������ܣ���ʼ��SPO2ģ��
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
void InitSPO2(void)
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
    {&g_tcbSPO2_SamplTask, SPO2_SamplTask, "SPO2_SamplTask", 11, s_arrSPO2_SamplStack, sizeof(s_arrSPO2_SamplStack) / sizeof(CPU_STK), 0},
    {&g_tcbSPO2_ProbeTask, SPO2_ProbeTask, "SPO2_ProbelTask", 30, s_arrSPO2_ProbeStack, sizeof(s_arrSPO2_ProbeStack) / sizeof(CPU_STK), 0},
    {&g_tcbRED_AdjTask, RED_AdjTask, "RED_AdjTask", 12, s_arrRED_AdjStack, sizeof(s_arrRED_AdjStack) / sizeof(CPU_STK), 0},
    {&g_tcbIR_AdjTask, IR_AdjTask, "IR_AdjTask", 12, s_arrIR_AdjStack, sizeof(s_arrIR_AdjStack) / sizeof(CPU_STK), 0},
  };
  
  //�ֲ�����
  unsigned int i;
  OS_ERR err;
  
  ///��ʼ��Ѫ��ģ���Ӳ����ʱ������
  InitSPO2_HW_CFG();
  //����IIR50hz�ݲ���
  CreatIRR_Filter_str(f125_15_20LP_NL,f125_15_20LP_num,f125_15_20LP_den,&SPO2_IR_f125_15_20hzlp);
  CreatIRR_Filter_str(f125_15_20LP_NL,f125_15_20LP_num,f125_15_20LP_den,&SPO2_RED_f125_15_20hzlp);
  
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
  OSTaskSuspend(&g_tcbRED_AdjTask,&err);
  OSTaskSuspend(&g_tcbIR_AdjTask,&err);
  //��������Ĭ�Ϲ���
  OSTaskSuspend(&g_tcbSPO2_SamplTask,&err);
}

/*********************************************************************************************************
* �������ƣ�SPO2_ProbeTask
* �������ܣ�Ѫ���������
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺adc����100����--�����Կ��Ǻ�������������⣬ȷ���ȶ��ԣ�
*********************************************************************************************************/
static void SPO2_ProbeTask(void *pArg)
{
  OS_ERR err;
  char arr_printf[100];
  while(1)
  {
    if(GetSPO2_IRData()<100 || GetSPO2_REDData()<100 || GetSPO2_IRData()>4000 || GetSPO2_REDData()>4000)
    {
      
//      printf("here\r\n");
      //���������ӵ����䣬�����������
      if(SPO2_ProbeStatus == TRUE)
      {
        //�����������
        if(REDAdjTask_runStatus == TRUE)
        {
          REDAdjTask_runStatus = FALSE;
          OSTaskSuspend(&g_tcbRED_AdjTask,&err);
        }
        if(IRAdjTask_runStatus == TRUE)
        {
          IRAdjTask_runStatus = FALSE;
          OSTaskSuspend(&g_tcbIR_AdjTask,&err);
        }
        //�����������
        if(SamplTask_runStatus == TRUE)
        {
          SamplTask_runStatus = FALSE;
          OSTaskSuspend(&g_tcbSPO2_SamplTask,&err);
        }
        
        sprintf(arr_printf,"main.t6.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf);
        //��/������dacֵ��λ
        red_DA_Val = RED_INIT_VAL;
        SetRed_DA_Val(red_DA_Val);
        ir_DA_Val = IR_INIT_VAL;
        SetIr_DA_Val(ir_DA_Val);
      }
      //��������
      SPO2_ProbeStatus = FALSE;
    }
    else
    {
      //���������䵽���ӣ������������Ĺ���
      if(SPO2_ProbeStatus == FALSE)
      {
        //��λ��������ı���--�������������50hz�ݲ���
        
        s_RedCnt = 0;
        s_IrCnt = 0;
        REDAdj_Suc = FALSE;
        IRAdj_Suc = FALSE;
        //�����������Ĺ���
        REDAdjTask_runStatus = TRUE;
        OSTaskResume(&g_tcbRED_AdjTask,&err);
        IRAdjTask_runStatus = TRUE;
        OSTaskResume(&g_tcbIR_AdjTask,&err);
      }
      SPO2_ProbeStatus = TRUE;
    }
    OSTimeDlyHMSM(0, 0, 0, 700, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}

/*********************************************************************************************************
* �������ƣ�RED_AdjTask
* �������ܣ�������
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void RED_AdjTask(void *pArg)
{
  OS_ERR err;
  while(1)
  {
    if(GetSPO2_REDData()>(RED_CENTRAL_LINE + RED_FINE_ADJ_OFFSET))
    {
      red_DA_Val+=2;
      SetRed_DA_Val(red_DA_Val);
      s_RedCnt = 0;
    }
    else if(GetSPO2_REDData()<(RED_CENTRAL_LINE - RED_FINE_ADJ_OFFSET))
    {
      red_DA_Val-=2;
      SetRed_DA_Val(red_DA_Val);
      s_RedCnt = 0;
    }
    else
    {
      s_RedCnt++;
      if(s_RedCnt>=40)
      {
        s_RedCnt = 0;
        
        REDAdj_Suc = TRUE;
        if(IRAdj_Suc == TRUE)
        {
          //�����������Ĺ���,��ʼ���˲���
          InitIRR_Filter_str(&SPO2_RED_f125_15_20hzlp);
          InitIRR_Filter_str(&SPO2_IR_f125_15_20hzlp);
          SamplTask_runStatus = TRUE;
          OSTaskResume(&g_tcbSPO2_SamplTask,&err);
        }
        
        //�����������
        REDAdjTask_runStatus = FALSE;
        OSTaskSuspend(&g_tcbRED_AdjTask,&err);
      }
    }
    OSTimeDlyHMSM(0, 0, 0, 2, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
/*********************************************************************************************************
* �������ƣ�IR_AdjTask
* �������ܣ���������
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void IR_AdjTask(void *pArg)
{
  OS_ERR err;
  while(1)
  {
    if(GetSPO2_IRData()>(IR_CENTRAL_LINE + IR_FINE_ADJ_OFFSET))
    {
      ir_DA_Val++;
      SetIr_DA_Val(ir_DA_Val);
      s_IrCnt = 0;
    }
    else if(GetSPO2_IRData()<(IR_CENTRAL_LINE - IR_FINE_ADJ_OFFSET))
    {
      ir_DA_Val--;
      SetIr_DA_Val(ir_DA_Val);
      s_IrCnt = 0;
    }
    else
    {
      s_IrCnt++;
      if(s_IrCnt>=40)
      {
        s_IrCnt = 0;
        
        IRAdj_Suc = TRUE;
        if(REDAdj_Suc == TRUE)
        {
          //�����������Ĺ��𣬳�ʼ���˲���
          InitIRR_Filter_str(&SPO2_IR_f125_15_20hzlp);
          InitIRR_Filter_str(&SPO2_RED_f125_15_20hzlp);
          SamplTask_runStatus = TRUE;
          OSTaskResume(&g_tcbSPO2_SamplTask,&err);
        }
        
        //�����������
        IRAdjTask_runStatus = FALSE;
        OSTaskSuspend(&g_tcbIR_AdjTask,&err);
      }
    }
    OSTimeDlyHMSM(0, 0, 0, 2, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
/*********************************************************************************************************
* �������ƣ�SPO2_SamplTask
* �������ܣ�Ѫ���źŲ����˲�����
* ���������void
* ���������void
* �� �� ֵ��void
* �������ڣ�2021��07��01��
* ע    �⣺
*********************************************************************************************************/
static void SPO2_SamplTask(void *pArg)
{
  OS_ERR err;
  float ir_data;
  float red_data;
  static int i = 0;
  static short ir_data_arr[500] = {0};
  static short red_data_arr[500] = {0};
  char spo2;
  float ir_max,ir_min,red_max,red_min;
  char arr_printf[50];
  char arr_printf2[50];
  char arr_printf3[50];
  float R;
  int j;
  while(1)
  {
    if(i>=500)//8msһ������500�����ݸ���5s���ʿɲ⵽24
    {
      i = 0;
      ir_max = 0;
      ir_min = 5000;
      red_max = 0;
      red_min = 5000;
      for(j = 0;j<500;j++)
      {
        ir_max = FindMax(ir_max,ir_data_arr[j]);
        red_max = FindMax(red_max,red_data_arr[j]);
        ir_min = FindMin(ir_min,ir_data_arr[j]);
        red_min = FindMin(red_min,red_data_arr[j]);
//        printf("RED = %d\r\n",(int)ir_max);
      }
      
      R = (((red_max-red_min)/(2047-red_max)))/((ir_max-ir_min)/(2047-ir_max));
      for(j = 0;j<10;j++)
      {
        if(R<R_arr[j])
        {break;}
      }
      spo2 = 99-j;
      sprintf(arr_printf3,"main.t6.txt=\"%d\"\xff\xff\xff",(int)(spo2));
      printf_s_aus(arr_printf3);
    }
    ir_data = IIR_Filter((float)GetSPO2_IRData(),&SPO2_IR_f125_15_20hzlp);
    red_data = IIR_Filter((float)GetSPO2_REDData(),&SPO2_RED_f125_15_20hzlp);
    
    ir_data_arr[i] = ir_data;
    red_data_arr[i] = red_data;
    i++;
    
    sprintf(arr_printf,"add main.s0.id,0,%d\xff\xff\xff",(int)(5+(ir_data - ir_min)/(ir_max*1.0-ir_min)*40));
    printf_s_aus(arr_printf);
    sprintf(arr_printf2,"add main.s1.id,0,%d\xff\xff\xff",(int)(5+(red_data - red_min)/(red_max*1.0-red_min)*40));
    printf_s_aus(arr_printf2);

    OSTimeDly(8,OS_OPT_TIME_PERIODIC, &err);
  }
}
