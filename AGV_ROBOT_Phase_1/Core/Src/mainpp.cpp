/*
 * main.cpp
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include "mainpp.h"
#include <ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
//#include "kal_man.h"

uint8_t pData;
char dataTX[62];
float num[10];
float a;
int i =0;
int j= 0;
//#include "main.h"
extern UART_HandleTypeDef huart3;
//SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
extern int vanToc1;
char button1[] = "button_1";
char button2[] = "button_2";
char button3[] = "button_3";
double vxRE,vyRE,wzRE;
int countTick = HAL_GetTick();

//void led0_cb(const std_msgs::Float32& msg);
void messageCb( const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;

//Publisher Magneticfield
//sensor_msgs::MagneticField magneticField;
//Publisher IMU
sensor_msgs::Imu imu;


//Publisher Position command
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
//IMU
//ros::Publisher pub_magnetic("imu/mag",&magneticField);
ros::Publisher pub_imu("imu/data", &imu);

//Subscriber and Publisher Velocity
geometry_msgs::Twist raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel",&raw_vel_msg);
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", messageCb);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
  if(huart->Instance == USART3)
  	{
  		if(pData != 'A')
  		{
  			if(pData != 'B')
  			{
  				if((pData >= 46 && pData <= 57) || pData == 32 || pData == 45)
  				{
  					dataTX[i] = pData;
  					i++;
  				}
  			}
  		}

  		if(pData == 'B')
  		{
  			i = 0;
  			char *ptr;
  			ptr = strtok(dataTX," ");
  			while(ptr != NULL)
  			{
  				num[j] = atof(ptr);
  				ptr = strtok(NULL," ");
  				j++;
  				if(j == 10)
  				{
  					j = 0;
  					break;
  				}
  			}
  		}
  	}
  	HAL_UART_Receive_IT(&huart3, &pData, 1);
}

void setup(void)
{
  nh.initNode();
  nh.subscribe(sub_vel);
  nh.advertise(raw_vel_pub);
  nh.advertise(chatter);
  nh.advertise(pub_imu);
  /*nh.advertise(pub_magnetic);
  	MPU_ConfigTypeDef myMpuConfig;
	//1. Initialise the MPU6050 module and I2C
	MPU6050_Init(&hi2c2);
	//2. Configure Accel and Gyro parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
	MPU6050_Config(&myMpuConfig);
  	QMC_init(&module, &hi2c1, 200);*/
}

void loop(void)
{

	if(W1 > 0 && W2 > 0 && W3 > 0 && W4 > 0)
	{
		tien();
	}
	else if(W1 < 0 && W2 < 0 && W3 < 0 && W4 < 0)
	{
		lui();
	}
	else if(W1 < 0 && W2 > 0 && W3 < 0 && W4 > 0)
	{
		xoayTrai();
	}
	else if(W1 > 0 && W2 < 0 && W3 > 0 && W4 < 0)
	{
		xoayPhai();
	}
	else if(W1 > 0 && W2 < 0 && W3 < 0 && W4 > 0)
	{
		ngangPhai();
	}
	else if(W1 < 0 && W2 > 0 && W3 > 0 && W4 < 0)
	{
		ngangTrai();
	}
	else
	{
		dung();
	}

	if(HAL_GetTick() - countTick > 100)
	{
		//publish velocity
		convertAngletoLinear(w1, w2, w3, w4);
		raw_vel_msg.linear.x = vx;
		raw_vel_msg.linear.y = vy;
		raw_vel_msg.angular.z = wz;
		raw_vel_pub.publish(&raw_vel_msg);

		//publish imu/data
		imu.header.frame_id = "imu_link";
		imu.header.stamp = nh.now();
		imu.orientation.w = num[0];
		imu.orientation.x = num[1];
		imu.orientation.y = num[2];
		imu.orientation.z = num[3];
		imu.angular_velocity.x = num[7];
		imu.angular_velocity.y = num[8];
		imu.angular_velocity.z = num[9];
		imu.linear_acceleration.x = num[4];
		imu.linear_acceleration.y = num[5];
		imu.linear_acceleration.z = num[6];
		pub_imu.publish(&imu);
		countTick = HAL_GetTick();
	}

	nh.spinOnce();
}

/*void led0_cb(const std_msgs::Float32& msg)

{

}*/
void messageCb( const geometry_msgs::Twist& cmd_msg)
{
	vxRE = cmd_msg.linear.x; //rad/s
	vyRE = cmd_msg.linear.y; //rad/s
	wzRE = cmd_msg.angular.z; //rad/s
	convertLineartoAngle(vxRE,vyRE,wzRE);
}

void convertLineartoAngle(double Vx, double Vy, double Wz)
{
	W1 = ((invert_r)*(Vx-Vy-(lx+ly)*Wz))/haiPI; //vòng/s
	W2 = ((invert_r)*(Vx+Vy+(lx+ly)*Wz))/haiPI; //vòng/s
	W3 = ((invert_r)*(Vx+Vy-(lx+ly)*Wz))/haiPI; //vòng/s
	W4 = ((invert_r)*(Vx-Vy+(lx+ly)*Wz))/haiPI; //vòng/s
}

void convertAngletoLinear(double w1, double w2, double w3, double w4)
{
	vx = haiPI*(w1 + w2 + w3 + w4)*(rChiaBon); 			//rad/s
	vy = haiPI*(-w1 + w2 + w3 - w4)*(rChiaBon); 		//rad/s
	wz = haiPI*(-w1 + w2 - w3 + w4)*(rChiaBonNhanLxLy); //rad/s
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_10)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0)
		{
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			  str_msg.data = button1;
			  chatter.publish(&str_msg);
		}
	}

	else if(GPIO_Pin == GPIO_PIN_11)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0)
		{
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			  str_msg.data = button2;
			  chatter.publish(&str_msg);
		}
	}

	else if(GPIO_Pin == GPIO_PIN_12)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0)
		{
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			  str_msg.data = button3;
			  chatter.publish(&str_msg);
		}
	}
}


