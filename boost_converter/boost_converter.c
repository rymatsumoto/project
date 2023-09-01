/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include <mwio4.h>

#define BDN1 1 //PEVボード
			                //WN WP VN VP UN UP
#define INV_MODE 4092		//11 11 11 11 11 00
#define INV_FREQ 200000
#define DEAD_TIME 0
#define TIMER0_INTERVAL 100
#define TIMER1_INTERVAL 100

volatile int set_pwm_on = 0;
volatile float duty = 0;

float mod_u = 0;
float current = 0;
float Gth = 0.1042;

float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float data[] = {0., 0., 0., 0.};

interrupt void duty_control(void)
{
	C6657_timer0_clear_eventflag();

	if (set_pwm_on == -1)
	{
		mod_u = duty * 2 - 1;
		PEV_inverter_set_uvw(BDN1, mod_u, 0, 0, 0);
	}
}

interrupt void read_current(void)
{
	C6657_timer1_clear_eventflag();

	PEV_ad_in_4ch(BDN1, 0, data);
	current = (2.5 - data[0]) / Gth / 3;
}

void initialize(void)
{
	PEV_ad_set_range(BDN1, range);
	PEV_ad_set_mode(BDN1, 0);

	int_disable();

	PEV_init(BDN1);
    PEV_inverter_disable_int(BDN1);
    PEV_inverter_init(BDN1, INV_FREQ, DEAD_TIME);
    PEV_inverter_control_gate(BDN1, INV_MODE);
    PEV_inverter_set_uvw(BDN1, 0, 0, 0, 0);
    PEV_inverter_enable_int(BDN1);

    C6657_timer0_init(TIMER0_INTERVAL);
	C6657_timer0_init_vector(duty_control, (CSL_IntcVectId)5);
	C6657_timer0_start();

    C6657_timer1_init(TIMER1_INTERVAL);
	C6657_timer1_init_vector(read_current, (CSL_IntcVectId)6);
	C6657_timer1_start();

	C6657_timer0_enable_int();
	C6657_timer1_enable_int();

	int_enable();
}

void MW_main(void)
{
	initialize();

	while(1)
	{
		if (set_pwm_on == 1)
		{
			PEV_inverter_start_pwm(BDN1);
			set_pwm_on = -1;
		}
	}
}