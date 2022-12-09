/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include <mwio4.h>

#define BDN1 1
#define BDN2 0
			                //WN WP VN VP UN UP
#define TRANS_MODE 80 		//00 00 01 01 00 00
#define INV_FREQ 85000
#define DEAD_TIME 300

volatile int set_pwm_on = -1;
volatile float duty = 1;

INT32 adc_0_data_peak;
INT32 adc_1_data_peak;
float i1_ampl;
float i2_ampl;
volatile float i1_weight = 1.1;
volatile float i2_weight = 1.1;

interrupt void read_envelope(void)
{
	int0_ack();
    adc_0_data_peak = IPFPGA_read(BDN2, 0x17);
	adc_1_data_peak = IPFPGA_read(BDN2, 0x18);
	i1_ampl = adc_0_data_peak * 125. / 8000 * i1_weight;
	i2_ampl = adc_1_data_peak * 125. / 8000 * i2_weight;
}

void MW_main(void)
{
	int_disable();

	PEV_init(BDN1);
    PEV_inverter_disable_int(BDN1);
    PEV_inverter_init(BDN1, INV_FREQ, DEAD_TIME);
    PEV_inverter_init_int_timing(BDN1, 1, 3, 0);
    PEV_int_init(BDN1, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(read_envelope, (CSL_IntcVectId)8, FALSE);
    PEV_inverter_control_gate(BDN1, TRANS_MODE);
    PEV_inverter_set_uvw(BDN1, 0, 0, 0, 0);
    PEV_inverter_enable_int(BDN1);
    int0_enable_int();

	int_enable();

	while(1)
	{
		if (set_pwm_on == 1)
		{
			PEV_inverter_start_pwm(BDN1);
			set_pwm_on = -1;
		}
		PEV_inverter_set_uvw(BDN1, duty-1, 1-duty, 0, 0);
	}
}