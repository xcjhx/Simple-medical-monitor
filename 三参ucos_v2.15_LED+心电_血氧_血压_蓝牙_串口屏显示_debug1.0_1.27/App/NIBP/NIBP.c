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
*                                              宏定义
*********************************************************************************************************/
#define THRE_NUM 11
//拟合曲线阶数 5阶-x^5
#define FIT_OLDER 5

/*********************************************************************************************************
*                                              枚举结构体
*********************************************************************************************************/
typedef struct
{
  double h_pro;
  double l_pro;
}propor;
/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
static char NIBP_I = 0; //阈值数组计数变量
static int NIBP_cnt = 0;//采样任务计时变量
static char NIBP_flag = 0;//开始采样标志位

//改进系数法的阈值数组与系数
static double compare_arr[6] = {12,11,7,6,5,0};
static propor pro_arr[6]={0.52,0.78,0.57,0.78,0.58,0.78,0.64,0.78,0.64,0.60,0.64,0.50};

//平均压
double AVE_BP;
double H_BP;
double L_BP;

//拟合曲线系数
static double coeff_arr[FIT_OLDER+1] = {0};

//压力阈值数组
static short Pressure_arr_ther_M[THRE_NUM] = {162,145,134,123,114,106,99,92,84,76,70};  
static short Pressure_arr_ther_H[THRE_NUM] = {240,230,220,210,200,190,180,170,160,150,140};  

static short *Pressure_arr_ther = Pressure_arr_ther_M;

//每个阈值阶梯的差值数组
static double Pre_PP[THRE_NUM] = {0};
//pp值对应的袖带压值
static double cuff_arr[THRE_NUM] = {0};

//血压校准系数,从ad值到气压值
unsigned short ad1 = 2090;//100mmhg时的ad值
unsigned short ad2 = 993; //0mmhg时的ad值
float coef1;
float coef2;

//血压测量任务运行状态标志位
static char MeasureTask_RunStatus = FALSE;

//过压监测任务
OS_TCB g_tcbNIBP_PreMonitorTask;            //任务控制块
static CPU_STK s_arrNIBP_PreMonitorStack[256]; //任务栈区

//开始测量任务
OS_TCB g_tcbNIBP_StarTask;               //任务控制块
static CPU_STK s_arrNIBP_StarStack[256]; //任务栈区

//血压测量任务
OS_TCB g_tcbNIBP_MeasureTask;               //任务控制块
static CPU_STK s_arrNIBP_MeasureStack[256]; //任务栈区

//脉搏波采样任务
OS_TCB g_tcbNIBP_PulseSamplTask;               //任务控制块
static CPU_STK s_arrNIBP_PulseSamplStack[2048]; //任务栈区
/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/      
static float ADtoPre(unsigned short data);//将ad值装换为袖带压

static  void  NIBP_PreMonitorTask(void *pArg);  //过压监测任务
static  void  NIBP_StarTask(void *pArg);        //开始测量任务
static  void  NIBP_MeasureTask(void *pArg);     //血压测量任务
static  void  NIBP_PulseSamplTask(void *pArg);  //脉搏波采样任务
/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigSw1Gpio
* 函数功能：key1
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static void ConfigSw1Gpio(void)
{
  //使能RCU相关时钟
  rcu_periph_clock_enable(RCU_GPIOB); //使能GPIOB的时钟

  gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_14); //配置PB14为上拉输入
}

/*********************************************************************************************************
* 函数名称：ConfigSw2EXTI
* 函数功能：key1中断
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static void ConfigSw21EXTI(void)
{
  rcu_periph_clock_enable(RCU_AF); //使能AF时钟
 
  gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_12); //连接EXTI12和PB12
  
  exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_FALLING);  //配置中断线

  nvic_irq_enable(EXTI10_15_IRQn, 2, 2);       //使能外部中断线EXTI0并设置优先级

  exti_interrupt_flag_clear(EXTI_12); //清除Line0上的中断标志位
}
/*********************************************************************************************************
* 函数名称：EXTI10_15_IRQHandler
* 函数功能：EXTI10-15的中断服务函数，12对应KEY1
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
    exti_interrupt_flag_clear(EXTI_12);  //清除Line0上的中断标志位
  }
}
/*********************************************************************************************************
* 函数名称：ADtoPre
* 函数功能：将ad值装换为袖带压
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static float ADtoPre(unsigned short data)
{
  return (data - coef1)*coef2;
}

/*********************************************************************************************************
* 函数名称：FindMin
* 函数功能：求最小值
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
* 函数名称：FindMax
* 函数功能：求最大值
* 输入参数：void 
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitNIBP
* 函数功能：初始化NIBP模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
void InitNIBP(void)
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
    {&g_tcbNIBP_StarTask, NIBP_StarTask, "NIBP_StarTask", 30, s_arrNIBP_StarStack, sizeof(s_arrNIBP_StarStack) / sizeof(CPU_STK), 0},
    {&g_tcbNIBP_PreMonitorTask, NIBP_PreMonitorTask, "NIBP_PreMonitorTask", 30, s_arrNIBP_PreMonitorStack, sizeof(s_arrNIBP_PreMonitorStack) / sizeof(CPU_STK), 0},
  };
  
  //局部变量
  unsigned int i;
  OS_ERR err;
  //硬件配置
  InitNIBP_HW_CFG();  
  ConfigSw21EXTI();
  ConfigSw1Gpio();
  
  //计算袖带压系数
  coef1 = ad2;
  coef2 = 100.00/(ad1 - ad2);
  
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
}
/*********************************************************************************************************
* 函数名称：NIBP_StarTask
* 函数功能：开始测量任务
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
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
    //如果没有在测量血压
    if(MeasureTask_RunStatus == FALSE)
    {
      if(!gpio_input_bit_get(GPIOB,GPIO_PIN_14) || i == '#')//如果sw1按下
      {
        Pressure_arr_ther = Pressure_arr_ther_M;
        
        //复位显示参数
        sprintf(arr_printf,"main.t12.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf);
        sprintf(arr_printf1,"main.t13.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf1);
        sprintf(arr_printf2,"main.t14.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf2);
        sprintf(arr_printf3,"main.t15.txt=\"--\"\xff\xff\xff");
        printf_s_aus(arr_printf3);
        
        //复位相关变量
        NIBP_I = 0;
        NIBP_cnt = 0;
        NIBP_flag = 0;
        
        //关闭气阀，打开气泵
        VALVE_F_CLOSE();
        VALVE_S_CLOSE();
        PUMU_ON();
        
        //开始运行血压测量任务
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
    else//如果血压测量程序在运行
    {
      if(i == '*')//测量途中关闭
      {
        MeasureTask_RunStatus = FALSE;
        //打开气阀，关闭气泵
        VALVE_F_OPEN();
        VALVE_S_OPEN();
        PUMU_OFF();
        
        //复位显示参数
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
* 函数名称：NIBP_PreMonitorTask
* 函数功能：过压监测任务
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static  void  NIBP_PreMonitorTask(void *pArg)
{
  OS_ERR err;
  while(1)
  {
    //软件过压或者硬件过压
    if(ADtoPre(GetCuff())>=299 || (!Protect_read()))
    {
      //挂起测量任务（删除任务）
      if(MeasureTask_RunStatus == TRUE)
      {
        MeasureTask_RunStatus = FALSE;
        OSTaskDel(&g_tcbNIBP_MeasureTask,&err);
      }
      //开阀，关泵
      VALVE_F_CLOSE();
      VALVE_S_OPEN();
      PUMU_OFF();
    }
    OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);
  }
}

/*********************************************************************************************************
* 函数名称：NIBP_MeasureTask
* 函数功能：血压测量任务
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2021年07月01日
* 注    意：
*********************************************************************************************************/
static  void  NIBP_MeasureTask(void *pArg)
{
  OS_ERR err;
  float cuff;         //袖带压
  static short max,min;
  //高压、低压、平均压对应的脉搏波峰峰值
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
    //放气/打气阶段，让袖带压去到阈值
    if(NIBP_flag == 0)
    {
      if(NIBP_I == 0)
      {
        if(cuff>=Pressure_arr_ther[0])
        {
          //关阀，关泵
          PUMU_OFF();
          //袖带压已经达到要求，可以开始采样
          NIBP_flag = 1;
        }
      }
      else
      {
        if(cuff<=Pressure_arr_ther[NIBP_I])
        {
          //关阀
          VALVE_S_CLOSE();
          //袖带压已经达到要求，可以开始采样
          NIBP_flag = 1;
        }
      }
    }
    //已达到阈值，准备采样了
    if(NIBP_flag == 1)
    {
      NIBP_cnt++;
    }
    if(NIBP_cnt == 60)
    {
      //脉搏波放大使能
      PULSE_enable();
      //将最大最小值复位
      max = 0;
      min = 4095;
    }
    if(NIBP_cnt >= 60 && NIBP_cnt<200)
    {
      //采样
      //找到最大最小值
      min = FindMin(min,GetPulse());
      max = FindMax(max,GetPulse());
    }
    //采样完毕，该去往下一个阈值
    if(NIBP_cnt >= 250)
    {
      //计算峰峰值
      if(NIBP_I >= 1)
      {
        Pre_PP[NIBP_I-1] = (max-min)/10.0;
        cuff_arr[NIBP_I-1] = cuff/10.0;
      }
      //复位标志变量
      NIBP_flag = 0;
      NIBP_cnt = 0;
      //脉搏波放大失能
      PULSE_disable();
      //开阀
      VALVE_S_OPEN();
      
      //去到最后一个阈值，表示测量完毕，开阀并挂起测量任务
      if(NIBP_I == THRE_NUM - 1)
      {
        VALVE_F_OPEN();
        
        //计算血压值
        //多项式拟合曲线
        PolynomialFit(cuff_arr,Pre_PP,coeff_arr,THRE_NUM-1,FIT_OLDER);
        //寻找出平均压--曲线峰值对应的x值
        AVE_BP = FindFirstMaxInPolynomial(coeff_arr,cuff_arr[1],cuff_arr[THRE_NUM-1],FIT_OLDER,0.05);
        hight = CalcPolynomial(coeff_arr,AVE_BP,FIT_OLDER);
        //利用改进系数法将收缩压与舒张压对应的脉搏波峰峰值计算出来
        for(j = 0;j<6;j++)
        {
          if(AVE_BP > compare_arr[j])
          {
            break;
          }
        }
        h_data = hight * pro_arr[j].h_pro;
        l_data = hight * pro_arr[j].l_pro;
        
        //根据峰峰值计算收缩压与舒张压
        H_BP = Polynomia_y_to_x(coeff_arr,AVE_BP,AVE_BP+4,FIT_OLDER,h_data,0.01,10);
        L_BP = Polynomia_y_to_x(coeff_arr,AVE_BP,AVE_BP-4,FIT_OLDER,l_data,0.01,10);
        
        sprintf(arr_printf1,"main.t13.txt=\"%d\"\xff\xff\xff",(int)(H_BP*10));
        printf_s_aus(arr_printf1);
        sprintf(arr_printf2,"main.t14.txt=\"%d\"\xff\xff\xff",(int)(AVE_BP*10));
        printf_s_aus(arr_printf2);
        sprintf(arr_printf3,"main.t15.txt=\"%d\"\xff\xff\xff",(int)(L_BP*10));
        printf_s_aus(arr_printf3);
        
        //挂起任务(删除任务)
        MeasureTask_RunStatus = FALSE;
        OSTaskDel(&g_tcbNIBP_MeasureTask,&err);
      }
      NIBP_I = (NIBP_I+1)%THRE_NUM;
      
      //检测是否高压；若是，切换阈值数组，重新测量
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



