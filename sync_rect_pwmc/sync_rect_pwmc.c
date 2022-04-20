//----------------------------------------------------------------------------------------
//title
//　同期整流＋可変キャパシタ制御プログラム
//designer
//　ryo matsumoto
//----------------------------------------------------------------------------------------

#include    <mwio4.h>

#define BDN0 0
#define BDN1 1
			                //WN WP VN VP UN UP
#define TRANS_MODE 80 		//00 00 01 01 00 00
#define INV_FREQ 85700
#define DEAD_TIME 300
#define REF_RECT 550
#define REF_PWMC 260

volatile int set_pwm_on_trans = -1;
volatile int set_pwm_on_rect = -1;
volatile int set_pwm_on_pwmc = -1;

volatile float pwm_out_BDN0;
volatile int wref_BDN0;
volatile int carrier_cnt_max_BDN0;
volatile int carrier_cnt_hlf_BDN0;


//----------------------------------------------------------------------------------------
//　可変キャパシタ制御
//----------------------------------------------------------------------------------------

interrupt void pwm_capacitor_control(void)
{
	C6657_timer1_clear_eventflag();

	wref_BDN0 = pwm_out_BDN0 * carrier_cnt_max_BDN0;
	IPFPGA_write(BDN0, 0x04, wref_BDN0);
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	carrier_cnt_max_BDN0 = 50000000. / INV_FREQ;
	carrier_cnt_hlf_BDN0 = 25000000. / INV_FREQ;
	
	IPFPGA_write(BDN0, 0x01, carrier_cnt_max_BDN0);
	IPFPGA_write(BDN0, 0x02, carrier_cnt_hlf_BDN0);
	IPFPGA_write(BDN0, 0x03, carrier_cnt_hlf_BDN0);
	IPFPGA_write(BDN0, 0x04, carrier_cnt_hlf_BDN0);
	IPFPGA_write(BDN0, 0x05, DEAD_TIME / 10);
	IPFPGA_write(BDN0, 0x08, REF_RECT);
	IPFPGA_write(BDN0, 0x09, REF_PWMC);

	PEV_init(BDN1);

	int_disable();

	PEV_inverter_disable_int(BDN1);

	PEV_inverter_init(BDN1, INV_FREQ, DEAD_TIME);
	// PEV_inverter_init_int_timing(BDN1, 1, 0, 0);
	// PEV_int_init(BDN1, 2, 0, 0, 0, 0, 0, 0, 0);
	// int0_init_vector(read_envelope, (CSL_IntcVectId)7, FALSE);
	PEV_inverter_control_gate(BDN1, TRANS_MODE);
	PEV_inverter_set_uvw(BDN1, 0, 0, 0, 0);

	// C6657_timer0_init(5);
	// C6657_timer0_init_vector(current_control, (CSL_IntcVectId)5);
	// C6657_timer0_start();

	C6657_timer1_init(5000);
	C6657_timer1_init_vector(pwm_capacitor_control, (CSL_IntcVectId)6);
	C6657_timer1_start(); 

	PEV_inverter_enable_int(BDN1);

	int0_enable_int();

	C6657_timer0_enable_int();
	C6657_timer1_enable_int();

	int_enable();
}

//----------------------------------------------------------------------------------------
//　メイン
//----------------------------------------------------------------------------------------

void MW_main(void)
{
	initialize();

	while(1)
	{
		if (set_pwm_on_trans == 1)
		{
			PEV_inverter_start_pwm(BDN1);
			set_pwm_on_trans = -1;
		}
		if (set_pwm_on_rect == 1)
		{
			IPFPGA_write(BDN0, 0x06, 1);
			set_pwm_on_rect = -1;
		}
		if (set_pwm_on_pwmc == 1)
		{
			IPFPGA_write(BDN0, 0x07, 1);
			set_pwm_on_pwmc = -1;
		}
	}
}
