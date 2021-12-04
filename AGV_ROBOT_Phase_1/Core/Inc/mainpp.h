/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define rChiaBon 0.0125
#define lx 0.1
#define ly 0.1
#define invert_r 21
#define rChiaBonNhanLxLy 0.06
#define haiPI 6.283185307

extern volatile int rSpeed1;
extern volatile int rSpeed2;
extern volatile int rSpeed3;
extern volatile int rSpeed4;
extern volatile int count;
extern volatile int PWM1,PWM2,PWM3,PWM4;
extern volatile int count;

volatile double Vx;
volatile double Vy;
volatile double Wz;

volatile double W1;
volatile double W2;
volatile double W3;
volatile double W4;

volatile double w1;
volatile double w2;
volatile double w3;
volatile double w4;

volatile double vx;
volatile double vy;
volatile double wz;

void setup(void);
void loop(void);
int chuyenChartoInt(char c);
void DC1_thuan(int speed);
void DC1_nghich(int speed);
void DC2_thuan(int speed);
void DC2_nghich(int speed);
void DC3_thuan(int speed);
void DC3_nghich(int speed);
void DC4_thuan(int speed);
void DC4_nghich(int speed);
void ngangPhai();
void ngangTrai();
void lui();
void tien();
void xoayTrai();
void xoayPhai();
void dung();
void xienPhai();
void xienTrai();
void SystemClock_Config(void);
void convertLineartoAngle(double Vx, double Vy, double Wz);
void convertAngletoLinear(double W1, double W2, double W3, double W4);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
