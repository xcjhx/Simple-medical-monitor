/*********************************************************************************************************
* 模块名称: BTCtrol.c
* 摘    要: 蓝牙模块
* 当前版本: 1.0.0
* 作    者: SZLY(COPYRIGHT 2019 SZLY. All rights reserved.)
* 完成日期: 2019年08月01日
* 内    容: 
* 注    意: 
**********************************************************************************************************
* 取代版本: 
* 作    者: 
* 完成日期: 
* 修改内容: 
* 修改文件: 
*********************************************************************************************************/

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "Bluetooth.h"
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

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void ConfigBTGPIO(void); //配置蓝牙的GPIO

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: ConfigBTGPIO
* 函数功能: 配置蓝牙的GPIO
* 输入参数: void 
* 输出参数: void
* 返 回 值: void
* 创建日期: 2019年08月01日
* 注    意: 
*********************************************************************************************************/
static void ConfigBTGPIO(void)
{
  //使能RCU相关时钟
  rcu_periph_clock_enable(RCU_GPIOC);                                 //使能GPIOA的时钟
  
//  gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);  //设置GPIO输出模式及速度
//  gpio_bit_reset(GPIOC, GPIO_PIN_4);                                  //将蓝牙复位引脚(BT_RES)拉低
  
  gpio_init(GPIOC, GPIO_MODE_IPD, GPIO_OSPEED_50MHZ, GPIO_PIN_5);  //设置GPIO输入
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称: InitBTCtrol
* 函数功能: 初始化蓝牙模块  
* 输入参数: void 
* 输出参数: void
* 返 回 值: void
* 创建日期: 2019年08月01日
* 注    意: 
*********************************************************************************************************/
void InitBluetooth(void)
{
  ConfigBTGPIO();
}

/*********************************************************************************************************
* 函数名称: BTCheck
* 函数功能: 检查蓝牙是否连接
* 输入参数: void
* 输出参数: void
* 返 回 值: ok:0-未连接，1-连接
* 创建日期: 2019年08月01日
* 注    意:
*********************************************************************************************************/
char BTCheck(void)
{
  char ok = 0;
  if (0 == gpio_input_bit_get(GPIOC, GPIO_PIN_5))
  {
    ok = 0;
  }
  else if (1 == gpio_input_bit_get(GPIOC, GPIO_PIN_5))
  {
    ok = 1;
  }
  return ok;
}
