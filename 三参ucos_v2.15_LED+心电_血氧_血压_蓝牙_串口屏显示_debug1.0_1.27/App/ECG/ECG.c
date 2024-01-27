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
#include "ECG.h"
#include "ADC.h"
#include "Filter.h"
#include "Calculate.h"
#include "Sprintf.h"
#include "gd32f30x_conf.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
//iir50hz陷波器系数
extern  const int irr_50hz_notch_NL;
extern  const float irr_50hz_notch_num[3];
extern  const float irr_50hz_notch_den[3];

//iir80hz低通滤波器系数
extern const int irr_tcheb_80hz_lp_NL;
extern const float irr_tcheb_80hz_lp_num[9];
extern const float irr_tcheb_80hz_lp_den[9];

//心电50hz陷波器
static IRR_Filter_str ECG_50hzNotch;
//iir80hz低通滤波器
static IRR_Filter_str ECG_IIR80hzLp;

//iir50hz陷波器滤波后的心电数据
float ECGData;

//导联脱落变量
char ECG_ProbeStatus = FALSE;

////心电信号显示数组
//float show_arr[70];
static int s_i = 0;//该数组的计数变量

//心电采样滤波任务
OS_TCB g_tcbECG_SamplTask;               //任务控制块
static CPU_STK s_arrECG_SamplStack[512]; //任务栈区

//心电探头检测任务
OS_TCB g_tcbECG_ProbeTask;               //任务控制块
static CPU_STK s_arrECG_ProbeStack[256]; //任务栈区

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static  void  ConfigECGGPIO(void);  //配置LED的GPIO
static  void  ECG_SamplTask(void *pArg);
static  void  ECG_ProbeTask(void *pArg);
/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigECGGPIO
* 函数功能：配置ECG的GPIO 
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static  void  ConfigECGGPIO(void)
{
  //使能RCU相关时钟
  rcu_periph_clock_enable(RCU_GPIOC);
  
  //配置ECG_ZERO引脚，默认为低电平--使能心电输出
  gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);  
  gpio_bit_reset(GPIOC, GPIO_PIN_13); 
  
  //配置ECG_LEAD_OFF引脚，上拉输入模式；输入0--导联连接，输入1--导联脱落
  gpio_init(GPIOC, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_5);  
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitECG
* 函数功能：初始化LED模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void InitECG(void)
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
    {&g_tcbECG_SamplTask, ECG_SamplTask, "ECG_SamplTask", 11, s_arrECG_SamplStack, sizeof(s_arrECG_SamplStack) / sizeof(CPU_STK), 0},
    {&g_tcbECG_ProbeTask, ECG_ProbeTask, "ECG_ProbelTask", 30, s_arrECG_ProbeStack, sizeof(s_arrECG_ProbeStack) / sizeof(CPU_STK), 0},
  };
  
  //局部变量
  unsigned int i;
  OS_ERR err;
  
  //硬件配置
  ConfigECGGPIO();  //配置LED的GPIO
  //创建50hz陷波器
  CreatIRR_Filter_str(irr_50hz_notch_NL,irr_50hz_notch_num,irr_50hz_notch_den,&ECG_50hzNotch);
  //创建IIR80hz低通滤波器
  CreatIRR_Filter_str(irr_tcheb_80hz_lp_NL,irr_tcheb_80hz_lp_num,irr_tcheb_80hz_lp_den,&ECG_IIR80hzLp);
  
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
  //采样任务默认挂起；
  OSTaskSuspend(&g_tcbECG_SamplTask,&err);
}

/*********************************************************************************************************
* 函数名称：ECG_SamplTask
* 函数功能：心电信号采样滤波任务
* 输入参数：cnt
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
    //数组采满，计算心率
    if(i>=500)//10ms一个数据500个数据覆盖5s心率可测到24
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
* 函数名称：ECG_ProbeTask
* 函数功能：心电导联检测
* 输入参数：cnt
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：0--导联连接，1--导联脱落
*********************************************************************************************************/
static void ECG_ProbeTask(void *pArg)
{
  OS_ERR err;
  char arr_printf[100];
  while(1)
  {
    if(gpio_input_bit_get(GPIOC,GPIO_PIN_5))
    {
      //导联从连接到脱落，挂起采样任务
      if(ECG_ProbeStatus == TRUE)
      {
        //挂起采样任务
        OSTaskSuspend(&g_tcbECG_SamplTask,&err);
        
        sprintf(arr_printf,"main.t7.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf);
      }
      //导联脱落
      ECG_ProbeStatus = FALSE;
    }
    else
    {
      //导联从脱落到连接，解除采样任务的挂起
      if(ECG_ProbeStatus == FALSE)
      {
        //复位采样任务的变量--数组计数变量，50hz陷波器
        s_i = 0;
        InitIRR_Filter_str(&ECG_50hzNotch);
        InitIRR_Filter_str(&ECG_IIR80hzLp);
        //解除采样任务的挂起
        OSTaskResume(&g_tcbECG_SamplTask,&err);
      }
      ECG_ProbeStatus = TRUE;
    }
    OSTimeDlyHMSM(0, 0, 0, 700, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}
