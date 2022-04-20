/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include <mwio4.h>

#define BDN 0
			                //WN WP VN VP UN UP
#define TRANS_MODE 80 		//00 00 01 01 00 00

volatile int set_pwm_on = -1;
// volatile float duty = 0;

void MW_main(void)
{
	PEV_init(BDN);
	PEV_inverter_init(BDN, 85000, 300);

	PEV_inverter_control_gate(BDN, TRANS_MODE);
	PEV_inverter_set_uvw(BDN, 0, 0, 0, 0);

	while(1)
	{
		if (set_pwm_on == 1)
		{
			PEV_inverter_start_pwm(BDN);
			set_pwm_on = -1;
		}
		// PEV_inverter_set_uvw(BDN, duty-1, 1-duty, 0, 0);
	}
}
