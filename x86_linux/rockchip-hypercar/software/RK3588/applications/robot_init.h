#ifndef __ROBOTINIT_H
#define __ROBOTINIT_H
#include "system.h"
#include <rtthread.h>
#include <rtdevice.h>
//Parameter structure of robot
//机器人参数结构体
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, Mec_Car is half wheelspacing //轮距 麦轮车为半轮距
  float AxleSpacing;       //Axlespacing, Mec_Car is half axlespacing //轴距 麦轮车为半轴距	
  int GearRatio;           //Motor_gear_ratio //电机减速比
  int EncoderAccuracy;     //Number_of_encoder_lines //编码器精度(编码器线数)
  float WheelDiameter;     //Diameter of driving wheel //主动轮直径	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //全向轮小车旋转半径	
}Robot_Parament_InitTypeDef;

// Encoder structure
//编码器结构体
typedef struct  
{
    int A;      
    int B; 
	  int C; 
    int D; 
}Encoder;

//Wheelspacing, Mec_Car is half wheelspacing
//轮距 麦轮是一半
//修正2021.03.30
#define V550_MEC_wheelspacing    0.115

//Axlespacing, Mec_Car is half axlespacing
//轴距 麦轮是一半

#define V550_MEC_axlespacing      0.079

//Motor_gear_ratio
//电机减速比
#define   HALL_30F    30

//Number_of_encoder_lines
//编码器精度
#define		Photoelectric_500 500

//Mecanum wheel tire diameter series
//麦轮轮胎直径
#define		Mecanum_75  0.075f


//The encoder octave depends on the encoder initialization Settings
//编码器倍频数，取决于编码器初始化设置
#define   EncoderMultiples  4
//Encoder data reading frequency
//编码器数据读取频率
#define   CONTROL_FREQUENCY 100

#define PI 3.1415f  //PI //圆周率

void Robot_Select(void);
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

#endif
