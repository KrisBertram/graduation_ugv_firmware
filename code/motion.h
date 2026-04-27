#ifndef CODE_MOTION_H_
#define CODE_MOTION_H_

extern int16 encoder_pulse;
extern float encoder_v;
extern float steer_duty;
extern float motor_duty;

float encoderSpeedCalculate(int16 pulse);

#endif /* CODE_MOTION_H_ */
