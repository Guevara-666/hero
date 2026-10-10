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

#include "DT7.hpp"
#include "HW_can.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
#include "imu_task.hpp"
#include "PID.hpp"

#include <cstring>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#define DISABLEMOTOR 0
#define ENABLEMOTOR  1


using hello_world::devices::remote_control::DT7;

bool Wether_Enable_Motor();

uint8_t rc_raw[18];
uint8_t rc_recv_mask = 0;   // 记录哪几帧到了

constexpr uint8_t MASK_F1 = 0x01;
constexpr uint8_t MASK_F2 = 0x02;
constexpr uint8_t MASK_F3 = 0x04;

// 定义全局变量
uint16_t kRxBufLen = 18; // 接收缓冲区长度
DT7 *rc_ptr = nullptr; // 遥控器对象指针
Pid pid[2];
Joint_Motor_t joint_motor_3; // DM4310 电机对象
Joint_Motor_t joint_motor_9; // DM4310 电机对象

void MainInit(void) {
  CanFilter_Init(&hcan1); //CAN1 过滤器初始化
  CanFilter_Init(&hcan2); //CAN2 过滤器初始化
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  rc_ptr = new DT7(); // 遥控器对象指针

  // DM4310 电机初始化
  joint_motor_3 = Joint_Motor_t();  //id=3
  joint_motor_9 = Joint_Motor_t();  //id=9
  joint_motor_init(&joint_motor_3, 3, SPEED_MODE);
  joint_motor_init(&joint_motor_9, 9, SPEED_MODE);

  // PID 初始化
  pid[0].Init(20.0f, 0.5f, 0.0f, 0.001f);
  pid[0].Derivative_Feedforward(0.0f); // 设置前馈

  pid[1].Init(5.0f, 0.0f, 0.5f, 0.001f);
  pid[1].Derivative_Feedforward(0.0f); // 设置前馈

  ImuInit(); // IMU 初始化

  HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask(void) { 
  float pitch_vel = 0.0f;
  float yaw_vel = 0.0f;
  float current_vel = 0.0f;
  float current_yaw_vel = 0.0f;

  HAL_IWDG_Refresh(&hiwdg); // 刷新看门狗
  ImuUpdate(); 
  if(Wether_Enable_Motor() == ENABLEMOTOR){
    pitch_vel = rc_ptr->rc_rv() * 10.0f; // 云台目标俯仰速度
    yaw_vel = rc_ptr->rc_rh() * 10.0f; // 云台目标偏航速度
    if(rc_ptr->rc_r_switch() == 1){ // 遥控器右拨杆向上->底盘跟随
      current_vel = imu_datas.gyro_vals[Z_AXIS];
      pid[0].Control(current_vel, yaw_vel); // PID 控制计算
      speed_ctrl(&hcan1, joint_motor_3.para.id, pid[0].Result()); // 设置电机输出
    }
    else{ // 小陀螺
      current_yaw_vel = imu_datas.gyro_vals[Z_AXIS];
      pid[1].Control(current_yaw_vel, yaw_vel); // PID 控制计算
      speed_ctrl(&hcan1, joint_motor_3.para.id, pid[1].Result()); // 设置电机输出
    }
    speed_ctrl(&hcan1, joint_motor_9.para.id, pitch_vel); // 设置电机输出
  }
  else{
    speed_ctrl(&hcan1, joint_motor_3.para.id, 0.0f); // 设置电机输出
    speed_ctrl(&hcan1, joint_motor_9.para.id, 0.0f); // 设置电机输出
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
      if (rx_header2.StdId == joint_motor_9.para.id) {
         dm4310_fbdata(&joint_motor_9, can2_rx_data, 8);
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
      if (hcan == &hcan1) {
        switch (rx_header1.StdId) {
          case 3:
            dm4310_fbdata(&joint_motor_3, can1_rx_data, 8);
            break;
          case 0x101:
            memcpy(rc_raw, can1_rx_data, 8);
            rc_recv_mask |= MASK_F1;
            break;
          case 0x102:
            memcpy(rc_raw + 8, can1_rx_data, 8);
            rc_recv_mask |= MASK_F2;
            break;
          case 0x103:
            memcpy(rc_raw + 16, can1_rx_data, 2);
            rc_recv_mask |= MASK_F3;
            break;
          default:
            return;
        }

        // 三帧都到齐 → 解析
        if (rc_recv_mask == (MASK_F1 | MASK_F2 | MASK_F3)) {
          rc_recv_mask = 0;
          rc_ptr->decode(rc_raw);   // ★ 和本地遥控器一样的接口
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
