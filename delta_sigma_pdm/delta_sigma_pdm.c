/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include <mwio4.h>

#define BDN_PEV 0 //PEVボード
#define BDN_FPGA 1 //FPGAボード
			                //WN WP VN VP UN UP
#define TRANS_MODE 80 		//00 00 01 01 00 00
#define INV_FREQ 83000
#define DEAD_TIME 400

volatile int set_pwm_on = 0;
volatile int pdm_ref = 0;
volatile int ref_full_scale = 1000;

int carrier_cnt_max;

interrupt void update_pdm_ref(void)
{
	C6657_timer0_clear_eventflag();

	IPFPGA_write(BDN_FPGA, 0x15, pdm_ref);
}

void initialize(void)
{
	carrier_cnt_max = 50000000. / INV_FREQ;
	
	IPFPGA_write(BDN_FPGA, 0x01, carrier_cnt_max);
	IPFPGA_write(BDN_FPGA, 0x05, DEAD_TIME/10);
	IPFPGA_write(BDN_FPGA, 0x15, pdm_ref);
	IPFPGA_write(BDN_FPGA, 0x16, ref_full_scale);

	int_disable();
	C6657_timer0_init(1000);
	C6657_timer0_init_vector(update_pdm_ref, (CSL_IntcVectId)6);
	C6657_timer0_start();
	C6657_timer0_enable_int();
	int_enable();
}

void MW_main(void)
{
	initialize();

	while(1)
	{
		if (set_pwm_on == 1)
		{
			IPFPGA_write(BDN_FPGA, 0x06, 1);
			set_pwm_on = -1;
		}
	}
}