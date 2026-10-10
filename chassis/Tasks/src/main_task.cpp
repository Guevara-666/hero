/*
 * @Author: Guevara-666 2662443905@qq.com
 * @Date: 2026-10-07 22:46:22
 * @LastEditors: Guevara-666 2662443905@qq.com
 * @LastEditTime: 2026-10-10 18:37:15
 * @FilePath: \chassis\Tasks\src\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"
#include "imu_task.hpp"

#include "DT7.hpp"
#include "HW_can.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
#include "GM3508.hpp"
#include "PID.hpp"
#include <cstring>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
using namespace hello_world::devices::remote_control;

#define DISABLEMOTOR false
#define ENABLEMOTOR true

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void send_rc_raw_to_can1();
bool Wether_Enable_Motor();

constexpr uint32_t RC_CAN_ID = 0x101;
// 定义全局变量
uint16_t kRxBufLen = 18; // 接收缓冲区长度
DT7 *rc_ptr = nullptr; // 遥控器对象指针
GM3508 motor[4];
Pid pid[5];
Joint_Motor_t joint_motor_3; // DM4310 电机对象
uint8_t rx_buf[18];   // DBUS 接收缓冲区
uint8_t rc_raw[18];              // 遥控器原始数据
volatile bool rc_raw_ready = false;


// 车身数据
float wheel_radius = 0.076f; // 轮子半径
float L = 0.4f; // 轴距
float W = 0.3f; // 轮距


void MainInit(void) {
  CanFilter_Init(&hcan1); //CAN1 过滤器初始化
  CanFilter_Init(&hcan2); //CAN2 过滤器初始化
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  rc_ptr = new DT7(); // 遥控器对象指针

  // GM3508 电机初始化
  for (int i = 0; i < 4; i++) {
    motor[i].Init(i + 1); // 初始化电机对象，设置ID
  }

  // Yaw DM4310 电机初始化
  joint_motor_init(&joint_motor_3, 3, MIT_MODE);

  // PID 初始化
  for (int i = 0; i < 5; i++) {
    pid[i].Init(20.0f, 0.5f, 0.0f, 0.001f);
    pid[i].Limit_Output(16384.0f, -16384.0f);
    pid[i].Derivative_Feedforward(0.0f); // 设置前馈增益
  }
  pid[4].Init(5.0f, 0.0f, 0.5f, 0.001f);
  pid[4].Limit_Output(3.0f, -3.0f);
  pid[4].Derivative_Feedforward(0.0f); // 设置前馈增益
  pid[4].Angle_Normalization();

  ImuInit(); // IMU 初始化

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen); // 开启DMA接收
  HAL_TIM_Base_Start_IT(&htim6);
}

/******************************************************************************/

void MainTask(void) { 
  HAL_IWDG_Refresh(&hiwdg); // 刷新看门狗
  ImuUpdate(); 
  send_rc_raw_to_can1();


  // 电机控制（使能）
  if(Wether_Enable_Motor() == ENABLEMOTOR){
    // ★ 局部变量定义
    float target_h_vel = 0.0f;
    float target_v_vel = 0.0f;
    float target_w_vel = 0.0f;
    float motor1_target_rpm = 0.0f;
    float motor2_target_rpm = 0.0f;
    float motor3_target_rpm = 0.0f;
    float motor4_target_rpm = 0.0f;
    float motor1_current_rpm = 0.0f;
    float motor2_current_rpm = 0.0f;
    float motor3_current_rpm = 0.0f;
    float motor4_current_rpm = 0.0f;

    target_h_vel = rc_ptr->rc_lh() * 10.0f; // 车目标左右速度
    target_v_vel = rc_ptr->rc_lv() * 10.0f; // 车目标前后速度

    if(rc_ptr->rc_r_switch() == 1){ // 遥控器右拨杆向上->底盘跟随
      float yaw_motor_angle = joint_motor_3.para.pos;
      // 角度归一化（DM4310 反馈是弧度，范围 -π ~ π）
      while (yaw_motor_angle >  M_PI) yaw_motor_angle -= 2 * M_PI;
      while (yaw_motor_angle < -M_PI) yaw_motor_angle += 2 * M_PI;

      pid[4].Control(yaw_motor_angle, 0.0f);
      target_w_vel = pid[4].Result();
    }
    else{ // 小陀螺
      target_w_vel = 3.1415926f; // 车目标旋转速度
    }
    // 计算电机目标速度（O型四麦轮）
    motor1_target_rpm = (target_v_vel + target_h_vel - target_w_vel * (L + W)/2.0f) / wheel_radius * 60.0f / (2.0f * 3.1415926f);
    motor2_target_rpm = (target_v_vel - target_h_vel + target_w_vel * (L + W)/2.0f) / wheel_radius * 60.0f / (2.0f * 3.1415926f);
    motor3_target_rpm = (target_v_vel - target_h_vel - target_w_vel * (L + W)/2.0f) / wheel_radius * 60.0f / (2.0f * 3.1415926f);
    motor4_target_rpm = (target_v_vel + target_h_vel + target_w_vel * (L + W)/2.0f) / wheel_radius * 60.0f / (2.0f * 3.1415926f);
    // 获取当前速度
    motor1_current_rpm = motor[0].vel();
    motor2_current_rpm = motor[1].vel();
    motor3_current_rpm = motor[2].vel();
    motor4_current_rpm = motor[3].vel();
    // PID 控制计算（单速度环）
    pid[0].Control(motor1_target_rpm, motor1_current_rpm);
    pid[1].Control(motor2_target_rpm, motor2_current_rpm);
    pid[2].Control(motor3_target_rpm, motor3_current_rpm);
    pid[3].Control(motor4_target_rpm, motor4_current_rpm);
    // 设置电机输出
    motor[0].setInput(pid[0].Result());
    motor[1].setInput(pid[1].Result());
    motor[2].setInput(pid[2].Result());
    motor[3].setInput(pid[3].Result());
  }
  else{
    // 电机控制（禁止）
    motor[0].setInput(0.0f);
    motor[1].setInput(0.0f);
    motor[2].setInput(0.0f);
    motor[3].setInput(0.0f);
  }

  // 准备 CAN 发送数据
  CAN_TxHeaderTypeDef tx_header;
  uint8_t data[8] = {0};
  uint32_t tx_mailbox;

  tx_header.StdId = 0x200;      // id:1~4 电流控制帧报头->0x200
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  motor[0].encode(data);
  motor[1].encode(data);
  motor[2].encode(data);
  motor[3].encode(data);

  HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &tx_mailbox);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart3) {
    if (Size == kRcRxDataLen) {
      HAL_IWDG_Refresh(&hiwdg);  // TODO:在这里刷新看门狗

      rc_ptr->decode(rx_buf);

      // 存到全局变量中
      memcpy(rc_raw, rx_buf, Size);
      rc_raw_ready = true;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header2;
  uint8_t can2_rx_data[8];

  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) ==
        HAL_OK) // 获得接收到的数据头和数据
    {
      // 帧头校验
      if (rx_header2.StdId == 0x200){ 
        motor[0].decode(can2_rx_data);
        motor[1].decode(can2_rx_data);
        motor[2].decode(can2_rx_data);
        motor[3].decode(can2_rx_data);
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO1_MSG_PENDING); // 再次使能FIFO1接收中断
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header1;
  uint8_t can1_rx_data[8];

  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) ==HAL_OK) // 获得接收到的数据头和数据
    {
      // 帧头校验（获取上板CAN1发送信息）
      if (rx_header1.DLC == 8) {
        switch (rx_header1.StdId) {
          case 3:
            dm4310_fbdata(&joint_motor_3, can1_rx_data, 8);
            break;
          default:
            break;
        }
      }
    }
  }
  HAL_CAN_ActivateNotification(
      hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 再次使能FIFO0接收中断
}

bool Wether_Enable_Motor(){

  if (rc_ptr->rc_l_switch() == 1){return DISABLEMOTOR;}
  else                           {return ENABLEMOTOR;}
}



void send_rc_raw_to_can1() {
    if (!rc_raw_ready) return;
    rc_raw_ready = false;

    // 帧1：0x101，8 字节
    {
        CAN_TxHeaderTypeDef h;
        h.StdId = 0x101;
        h.IDE   = CAN_ID_STD;
        h.RTR   = CAN_RTR_DATA;
        h.DLC   = 8;
        uint32_t mb;
        HAL_CAN_AddTxMessage(&hcan1, &h, rc_raw, &mb);
    }

    // 帧2：0x102，8 字节
    {
        CAN_TxHeaderTypeDef h;
        h.StdId = 0x102;
        h.IDE   = CAN_ID_STD;
        h.RTR   = CAN_RTR_DATA;
        h.DLC   = 8;
        uint32_t mb;
        HAL_CAN_AddTxMessage(&hcan1, &h, rc_raw + 8, &mb);
    }

    // 帧3：0x103，2 字节
    {
        CAN_TxHeaderTypeDef h;
        h.StdId = 0x103;
        h.IDE   = CAN_ID_STD;
        h.RTR   = CAN_RTR_DATA;
        h.DLC   = 2;
        uint32_t mb;
        HAL_CAN_AddTxMessage(&hcan1, &h, rc_raw + 16, &mb);
    }
}