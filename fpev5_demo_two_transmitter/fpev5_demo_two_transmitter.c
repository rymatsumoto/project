/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include <mwio4.h>

#define BDN_PEV1 0 //PEVボード1
#define BDN_PEV2 1 //PEVボード2
#define BDN_FPGA 0 //FPGAボード
			                //WN WP VN VP UN UP
#define TRANS_MODE 80 		//00 00 01 01 00 00
#define INV_FREQ_PEV1 85000
#define INV_FREQ_PEV2 84000
#define DEAD_TIME 600

volatile int set_pwm_on_pev1 = 0;
volatile int set_pwm_on_pev2 = 0;
volatile int set_pwm_on_soft_start_pev1 = 0;
volatile int set_pwm_on_soft_start_pev2 = 0;
volatile float duty_pev1 = 1;
volatile float duty_pev2 = 1;
volatile float duty_ramp_pev1 = 0;
volatile float duty_ramp_pev2 = 0;
volatile float duty_step_pev1 = 0;
volatile float duty_step_pev2 = 0;

INT32 adc_0_data_peak;
INT32 adc_1_data_peak;
float i1_ampl = 0;
float i2_ampl = 0;
volatile float i1_weight = 1.1;
volatile float i2_weight = 1.1;
volatile float i1_ampl_max = 100;
volatile float i2_ampl_max = 100;
volatile int overcurrent_protection = 0;

void read_envelope(void)
{
    adc_0_data_peak = IPFPGA_read(BDN_FPGA, 0x17);
	adc_1_data_peak = IPFPGA_read(BDN_FPGA, 0x18);
	i1_ampl = adc_0_data_peak * 125. / 8000 * i1_weight;
	i2_ampl = adc_1_data_peak * 125. / 8000 * i2_weight;

	if (overcurrent_protection == 1)
	{
		if (i1_ampl > i1_ampl_max)
		{
			PEV_inverter_stop_pwm(BDN_PEV1);
		}
		if (i2_ampl > i2_ampl_max)
		{
			PEV_inverter_stop_pwm(BDN_PEV2);
		}
	}
}

void duty_control(void)
{
	if (set_pwm_on_pev1 == -1)
	{
		PEV_inverter_set_uvw(BDN_PEV1, duty_pev1-1, 1-duty_pev1, 0, 0);
	}
	else if (set_pwm_on_soft_start_pev1 == -1)
	{
		if (duty_ramp_pev1 < duty_pev1)
		{
			duty_ramp_pev1 += duty_step_pev1;
			PEV_inverter_set_uvw(BDN_PEV1, duty_ramp_pev1-1, 1-duty_ramp_pev1, 0, 0);
		}
		else
		{
			PEV_inverter_set_uvw(BDN_PEV1, duty_pev1-1, 1-duty_pev1, 0, 0);
		}
	}

	if (set_pwm_on_pev2 == -1)
	{
		PEV_inverter_set_uvw(BDN_PEV2, duty_pev2-1, 1-duty_pev2, 0, 0);
	}
	else if (set_pwm_on_soft_start_pev2 == -1)
	{
		if (duty_ramp_pev2 < duty_pev2)
		{
			duty_ramp_pev2 += duty_step_pev2;
			PEV_inverter_set_uvw(BDN_PEV2, duty_ramp_pev2-1, 1-duty_ramp_pev2, 0, 0);
		}
		else
		{
			PEV_inverter_set_uvw(BDN_PEV2, duty_pev2-1, 1-duty_pev2, 0, 0);
		}
	}
}

interrupt void sync_interrupt(void)
{
	int0_ack();
	read_envelope();
	duty_control();
}

void initialize(void)
{
	int_disable();

	PEV_init(BDN_PEV1);
    PEV_inverter_disable_int(BDN_PEV1);
    PEV_inverter_init(BDN_PEV1, INV_FREQ_PEV1, DEAD_TIME);
    PEV_inverter_init_int_timing(BDN_PEV1, 1, 3, 0);
    PEV_int_init(BDN_PEV1, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(sync_interrupt, (CSL_IntcVectId)8, FALSE);
    PEV_inverter_control_gate(BDN_PEV1, TRANS_MODE);
    PEV_inverter_set_uvw(BDN_PEV1, -1, 1, 0, 0);
    PEV_inverter_enable_int(BDN_PEV1);
    int0_enable_int();

	PEV_init(BDN_PEV2);
    PEV_inverter_disable_int(BDN_PEV2);
    PEV_inverter_init(BDN_PEV2, INV_FREQ_PEV2, DEAD_TIME);
    PEV_inverter_control_gate(BDN_PEV2, TRANS_MODE);
    PEV_inverter_set_uvw(BDN_PEV2, -1, 1, 0, 0);
    PEV_inverter_enable_int(BDN_PEV2);
    int0_enable_int();

	int_enable();
}

void MW_main(void)
{
	initialize();

	while(1)
	{
		if (set_pwm_on_pev1 == 1)
		{
			PEV_inverter_start_pwm(BDN_PEV1);
			set_pwm_on_pev1 = -1;
		}
		else if (set_pwm_on_soft_start_pev1 == 1)
		{
			PEV_inverter_start_pwm(BDN_PEV1);
			set_pwm_on_soft_start_pev1 = -1;
		}

		if (set_pwm_on_pev2 == 1)
		{
			PEV_inverter_start_pwm(BDN_PEV2);
			set_pwm_on_pev2 = -1;
		}
		else if (set_pwm_on_soft_start_pev2 == 1)
		{
			PEV_inverter_start_pwm(BDN_PEV2);
			set_pwm_on_soft_start_pev2 = -1;
		}
	}
}