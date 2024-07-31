/*
 * timer_ctrl.h
 *
 *  Created on: Jan 8, 2024
 *      Author: catsa
 */

#ifndef APPS_INC_TIMER_CTRL_H_
#define APPS_INC_TIMER_CTRL_H_

typedef struct _pwm_timer{
//	uint8_t get_evt;
//	uint8_t gpio_set;
	int ncount;
	//int	nDelay;
	//int nPluse;
}PWM_Timer;

void PWM_OUT(int channel, int data);
void strobe_timer_init(void);
void pwm_timer_disable(void);
void timer_ctrl_init(void);
//extern PWM_Timer nPWMTimer[];
extern __IO int nPWMTimer[];

#endif /* APPS_INC_TIMER_CTRL_H_ */
