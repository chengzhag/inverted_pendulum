#ifndef INVERTED_PENDULUM
#define INVERTED_PENDULUM

#include "encoder_timer.h"
#include "encoder_motor.h"
#include "PID.hpp"

#define PI		3.14159265358979323846
#define INF_FLOAT 3.402823466e+38F


class EncoderPendulum :public EncoderTimer
{
	unsigned int npr;//ÿȦpos������
	short oldDiff;
	short ddiff;
public:

	//��������ת��������Ϊ���Ա�����ʹ��
	//numPerRound: ÿȦpos������
	EncoderPendulum(TIM_TypeDef *TIMx,
		unsigned int numPerRound = 2000);

	//���ظ���refresh()�������ԽǼ��ٶȽ��м���
	void refresh();

	//��ȡ���Ի���ֵ���Գ�ʼ����Ϊ+-pi�㣬��Χ-pi~pi
	float getRadian();

	//��ȡ���ٶ�
	float getRadianDiff();

	//��ȡ�Ǽ��ٶ�
	float getRadianDDiff();
};

class MotorBeam :public EncoderMotor
{
	unsigned int npr;//ÿȦpos������

public:

	//������������Ի�ȡ�Ի���Ϊ��λ����ԽǶ�ֵ
	//numPerRound: ÿȦpos������
	MotorBeam(TIM_TypeDef *TIMx,
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		unsigned int numPerRound = 1560,
		Encoder_Motor_Target_Typedef controlTarget = Encoder_Motor_Target_Position,
		float refreshInterval = 0.005);

	//��ȡ����ֵ���Գ�ʼ����Ϊ0���ȵ㣬��Χ-nan~+nan
	float getRadian();

	//��ȡ���ٶ�
	float getRadianDiff();

	//����Ŀ��ǶȻ��Ȳ�
	void setRadianDiff(float radian);
};

typedef enum
{
	Inverted_Pendulum_Mode_Disabled,
	Inverted_Pendulum_Mode_Invert,
	Inverted_Pendulum_Mode_Swing,
	Inverted_Pendulum_Mode_Swing_Invert
}Inverted_Pendulum_Mode_Typedef;

class InvertedPendulum
{
	float refreshInt;
	float enRadThres;//����pid�����ĽǶȷ�Χ�������򣬵�λ���ȡ���ʼpi/3
	int mode;
public:
	greg::PID pendulumRadianPID, beamRadianPID,//�Ƕ�PID
		pendulumPalstancePID, beamPalstancePID;//���ٶ�PID
	EncoderPendulum encoder;
	MotorBeam motor;

	InvertedPendulum(TIM_TypeDef *TIMpendulum, TIM_TypeDef *TIMmotor,
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		unsigned int nprPendulum = 2000, unsigned int nprMotor = 1560, float refreshInterval = 0.005);

	//��ʼ���������������PID���ƣ����ó�ʼ��PIDֵ
	void begin();

	//�Ա����������PID������PID����ˢ��
	void refresh();

	//���õ�����ģʽ
	void setMode(Inverted_Pendulum_Mode_Typedef m) {
		mode = m;
	}

	//���ý���pid�����ĽǶȷ�Χ
	void setEnRadThres(float t);

	//��ȡ�ڸ˽Ƕȣ����ȣ�
	float getPendulumRadian();

	//��ȡ�ڸ˽��ٶȣ�����/�룩
	float getPendulumPalstance();

	//��ȡ�ڸ˽Ǽ��ٶȣ�����/��^2��
	float getPendulumAcceleration();

	//��ȡ�����Ƕȣ����ȣ�
	float getBeamRadian();

	//��ȡ�������ٶȣ�����/�룩
	float getBeamPalstance();
};

#endif