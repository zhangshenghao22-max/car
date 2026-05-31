#include "robot_init.h"

//Initialize the robot parameter structure
//初始化机器人参数结构体
Robot_Parament_InitTypeDef  Robot_Parament;
/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
   Robot_Init(V550_MEC_wheelspacing, V550_MEC_axlespacing, 0,HALL_30F, Photoelectric_500, Mecanum_75);
}

/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) //
{
    //wheelspacing, Mec_Car is half wheelspacing
    //轮距 麦轮车为半轮距
  Robot_Parament.WheelSpacing=wheelspacing;
    //axlespacing, Mec_Car is half axlespacing
  //轴距 麦轮车为半轴距
  Robot_Parament.AxleSpacing=axlespacing;
    //Rotation radius of omnidirectional trolley
  //全向轮小车旋转半径
  Robot_Parament.OmniTurnRadiaus=omni_turn_radiaus;
    //motor_gear_ratio
    //电机减速比
  Robot_Parament.GearRatio=gearratio;
    //Number_of_encoder_lines
  //编码器精度(编码器线数)
  Robot_Parament.EncoderAccuracy=Accuracy;
    //Diameter of driving wheel
  //主动轮直径
  Robot_Parament.WheelDiameter=tyre_diameter;

    //Encoder value corresponding to 1 turn of motor (wheel)
    //电机(车轮)转1圈对应的编码器数值
    Encoder_precision=Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;//编码器线数*电机减速比
    //Driving wheel circumference
  //主动轮周长
    Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
    //wheelspacing, Mec_Car is half wheelspacing
  //轮距 麦轮车为半轮距
  Wheel_spacing=Robot_Parament.WheelSpacing;
  //axlespacing, Mec_Car is half axlespacing
  //轴距 麦轮车为半轴距
    Axle_spacing=Robot_Parament.AxleSpacing;
    //Rotation radius of omnidirectional trolley
  //全向轮小车旋转半径
    Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus;

    rt_kprintf("ROS CAR START SUCCESSFULLY!!\n");
}