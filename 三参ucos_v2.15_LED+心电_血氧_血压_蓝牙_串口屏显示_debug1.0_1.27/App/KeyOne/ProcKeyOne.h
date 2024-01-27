/*********************************************************************************************************
* 模块名称：ProcKeyOne.h
* 摘    要：ProcKeyOne模块，进行独立按键处理模块初始化，以及独立按键处理函数实现
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
#ifndef _PROC_KEY_ONE_H_
#define _PROC_KEY_ONE_H_

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              API函数声明
*********************************************************************************************************/
void  InitProcKeyOne(void);   //初始化ProcKeyOne模块
void  ProcKeyDownKey1(void);  //处理KEY1按下的事件，即KEY1按键按下的响应函数 
void  ProcKeyUpKey1(void);    //处理KEY1弹起的事件，即KEY1按键弹起的响应函数
void  ProcKeyDownKey2(void);  //处理KEY2按下的事件，即KEY2按键按下的响应函数 
void  ProcKeyUpKey2(void);    //处理KEY2弹起的事件，即KEY2按键弹起的响应函数
void  ProcKeyDownKey3(void);  //处理KEY3按下的事件，即KEY3按键按下的响应函数 
void  ProcKeyUpKey3(void);    //处理KEY3弹起的事件，即KEY3按键弹起的响应函数
void  ScanKey(void);          //按键扫描

#endif
