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
#include "SPO2.h"
#include "Filter.h"
#include "ADC.h"
#include "Calculate.h"
#include "Sprintf.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

#define IR_CENTRAL_LINE  900 //红外光调光成功中线
#define IR_FINE_ADJ_OFFSET  200   //红外光细调上下偏移量初值

#define RED_CENTRAL_LINE 700 //红光调光成功中线
#define RED_FINE_ADJ_OFFSET 200  //红光细调上下偏移量初值

#define IR_INIT_VAL 120
#define RED_INIT_VAL 80
/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
//iir20hz低通滤波器系数
extern const int f125_15_20LP_NL;
extern const float f125_15_20LP_num[18];
extern const float f125_15_20LP_den[18];

//iir20hz低通滤波器
static IRR_Filter_str SPO2_IR_f125_15_20hzlp;
static IRR_Filter_str SPO2_RED_f125_15_20hzlp;

//R值表
static float R_arr[10] = {0.480,0.540,0.570,0.600,0.635,0.655,0.690,0.720,0.750,0.777};

//导联脱落变量
static char SPO2_ProbeStatus = FALSE;

//红/红外光调光任务运行状态标志位
static char REDAdjTask_runStatus = FALSE;
static char IRAdjTask_runStatus = FALSE;

//红/红外光调光成功标志位
static char REDAdj_Suc = FALSE;
static char IRAdj_Suc = FALSE;

//采样任务运行状态标志位
static char SamplTask_runStatus = FALSE;

//红/红外光的dac值
static unsigned short red_DA_Val = RED_INIT_VAL;
static unsigned short ir_DA_Val = IR_INIT_VAL;

//红/红外调光计数器
static char s_RedCnt = 0;
static char s_IrCnt = 0;

//血氧探头检测任务
OS_TCB g_tcbSPO2_ProbeTask;               //任务控制块
static CPU_STK s_arrSPO2_ProbeStack[128]; //任务栈区

//红光调光任务
OS_TCB g_tcbRED_AdjTask;               //任务控制块
static CPU_STK s_arrRED_AdjStack[256]; //任务栈区

//红外光调光任务
OS_TCB g_tcbIR_AdjTask;               //任务控制块
static CPU_STK s_arrIR_AdjStack[256]; //任务栈区

//心电采样滤波任务
OS_TCB g_tcbSPO2_SamplTask;               //任务控制块
static CPU_STK s_arrSPO2_SamplStack[1024]; //任务栈区

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
static  void  SPO2_ProbeTask(void *pArg);   //探头检测
static  void  RED_AdjTask(void *pArg);       //红光调光
static  void  IR_AdjTask(void *pArg);       //红外光调光
static  void  SPO2_SamplTask(void *pArg);   //血氧信号采样与滤波  

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitSPO2
* 函数功能：初始化SPO2模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void InitSPO2(void)
{
  //任务信息结构体
  typedef struct
  {
    OS_TCB*      tcb;     //任务控制块
    OS_TASK_PTR  func;    //任务热口地址
    CPU_CHAR*    name;    //任务名字
    OS_PRIO      prio;    //任务优先级
    CPU_STK*     stkBase; //任务栈区基地址
    CPU_STK_SIZE stkSize; //栈区大小
    OS_MSG_QTY   queSize; //内建消息队列容量
  }StructTaskInfo;

  //任务列表
  StructTaskInfo taskInfo[] = 
  {
    {&g_tcbSPO2_SamplTask, SPO2_SamplTask, "SPO2_SamplTask", 11, s_arrSPO2_SamplStack, sizeof(s_arrSPO2_SamplStack) / sizeof(CPU_STK), 0},
    {&g_tcbSPO2_ProbeTask, SPO2_ProbeTask, "SPO2_ProbelTask", 30, s_arrSPO2_ProbeStack, sizeof(s_arrSPO2_ProbeStack) / sizeof(CPU_STK), 0},
    {&g_tcbRED_AdjTask, RED_AdjTask, "RED_AdjTask", 12, s_arrRED_AdjStack, sizeof(s_arrRED_AdjStack) / sizeof(CPU_STK), 0},
    {&g_tcbIR_AdjTask, IR_AdjTask, "IR_AdjTask", 12, s_arrIR_AdjStack, sizeof(s_arrIR_AdjStack) / sizeof(CPU_STK), 0},
  };
  
  //局部变量
  unsigned int i;
  OS_ERR err;
  
  ///初始化血氧模块的硬件与时序配置
  InitSPO2_HW_CFG();
  //创建IIR50hz陷波器
  CreatIRR_Filter_str(f125_15_20LP_NL,f125_15_20LP_num,f125_15_20LP_den,&SPO2_IR_f125_15_20hzlp);
  CreatIRR_Filter_str(f125_15_20LP_NL,f125_15_20LP_num,f125_15_20LP_den,&SPO2_RED_f125_15_20hzlp);
  
  //创建任务
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

    //校验
    if(OS_ERR_NONE != err)
    {
      printf("Fail to create %s (%d)\r\n", taskInfo[i].name, err);
      while(1){}
    }
  }
  
  //调光任务默认挂起；
  OSTaskSuspend(&g_tcbRED_AdjTask,&err);
  OSTaskSuspend(&g_tcbIR_AdjTask,&err);
  //采样任务默认挂起
  OSTaskSuspend(&g_tcbSPO2_SamplTask,&err);
}

/*********************************************************************************************************
* 函数名称：SPO2_ProbeTask
* 函数功能：血氧导联检测
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：adc低于100脱落--（可以考虑后续加上连续检测，确保稳定性）
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
      //导联从连接到脱落，挂起采样任务
      if(SPO2_ProbeStatus == TRUE)
      {
        //挂起调光任务
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
        //挂起采样任务
        if(SamplTask_runStatus == TRUE)
        {
          SamplTask_runStatus = FALSE;
          OSTaskSuspend(&g_tcbSPO2_SamplTask,&err);
        }
        
        sprintf(arr_printf,"main.t6.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf);
        //红/红外光的dac值复位
        red_DA_Val = RED_INIT_VAL;
        SetRed_DA_Val(red_DA_Val);
        ir_DA_Val = IR_INIT_VAL;
        SetIr_DA_Val(ir_DA_Val);
      }
      //导联脱落
      SPO2_ProbeStatus = FALSE;
    }
    else
    {
      //导联从脱落到连接，解除采样任务的挂起
      if(SPO2_ProbeStatus == FALSE)
      {
        //复位采样任务的变量--数组计数变量，50hz陷波器
        
        s_RedCnt = 0;
        s_IrCnt = 0;
        REDAdj_Suc = FALSE;
        IRAdj_Suc = FALSE;
        //解除调光任务的挂起
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
* 函数名称：RED_AdjTask
* 函数功能：红光调光
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
          //解除采样任务的挂起,初始化滤波器
          InitIRR_Filter_str(&SPO2_RED_f125_15_20hzlp);
          InitIRR_Filter_str(&SPO2_IR_f125_15_20hzlp);
          SamplTask_runStatus = TRUE;
          OSTaskResume(&g_tcbSPO2_SamplTask,&err);
        }
        
        //挂起调光任务
        REDAdjTask_runStatus = FALSE;
        OSTaskSuspend(&g_tcbRED_AdjTask,&err);
      }
    }
    OSTimeDlyHMSM(0, 0, 0, 2, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
/*********************************************************************************************************
* 函数名称：IR_AdjTask
* 函数功能：红外光调光
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
          //解除采样任务的挂起，初始化滤波器
          InitIRR_Filter_str(&SPO2_IR_f125_15_20hzlp);
          InitIRR_Filter_str(&SPO2_RED_f125_15_20hzlp);
          SamplTask_runStatus = TRUE;
          OSTaskResume(&g_tcbSPO2_SamplTask,&err);
        }
        
        //挂起调光任务
        IRAdjTask_runStatus = FALSE;
        OSTaskSuspend(&g_tcbIR_AdjTask,&err);
      }
    }
    OSTimeDlyHMSM(0, 0, 0, 2, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
/*********************************************************************************************************
* 函数名称：SPO2_SamplTask
* 函数功能：血氧信号采样滤波任务
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
    if(i>=500)//8ms一个数据500个数据覆盖5s心率可测到24
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
