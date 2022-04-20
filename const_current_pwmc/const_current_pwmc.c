//----------------------------------------------------------------------------------------
//title
//　両側電流制御＋可変キャパシタ制御プログラム
//designer
//　ryo matsumoto
//----------------------------------------------------------------------------------------

#include    <mwio4.h>

#define BDN0 1              //受電側FPGAボード
#define BDN1 0              //送電側PEVボード
			                //WN WP VN VP UN UP
#define TRANS_MODE 80 		//00 00 01 01 00 00
#define INV_FREQ 85000
#define DEAD_TIME 300
#define REF_RECT 520
#define REF_PWMC 260
#define SMA_LEN 5

INT32 adc_0_data_peak;
INT32 adc_1_data_peak;

float i1_ampl;
float i2_ampl;
float i1_ampl_lst[SMA_LEN];
float i2_ampl_lst[SMA_LEN];
float i1_ampl_avg = 0;
float i2_ampl_avg = 0;

volatile int set_pwm_on_trans = -1;
volatile int set_pwm_on_rect = -1;
volatile int set_pwm_on_pwmc = -1;
volatile int control_on = -1;
volatile int pwmc_search_on = -1;
volatile float i1_ampl_ref;
volatile float i2_ampl_ref;
volatile float proportionalGain = 0.01;
volatile float integralGain = 0.003;

volatile float i1_proportional = 0;
volatile float i2_proportional = 0;
volatile float i1_integral = 0;
volatile float i2_integral = 0;
volatile float i1_difference;
volatile float i2_difference;

volatile float inv_out_BDN1;
volatile float inv_mod_BDN1;
volatile float pwm_out_BDN0;
volatile float inv_out_BDN0;
volatile float inv_out_BDN0_last = 0;

volatile int up_down = 1;

volatile int uref_BDN0;
volatile int vref_BDN0;
volatile int wref_BDN0;

volatile int carrier_cnt_max;
volatile int carrier_cnt_hlf;

volatile float i1_weight = 1.05;
volatile float i2_weight = 1.15;

//----------------------------------------------------------------------------------------
//　電流包絡線検波
//----------------------------------------------------------------------------------------

interrupt void read_envelope(void)
{
	int i;

	float i1_ampl_lst_tmp[SMA_LEN];
	float i2_ampl_lst_tmp[SMA_LEN];
	float i1_ampl_sum = 0;
	float i2_ampl_sum = 0;

	int0_ack();

	adc_0_data_peak = IPFPGA_read(BDN0, 0x17);
	adc_1_data_peak = IPFPGA_read(BDN0, 0x18);

	i1_ampl = adc_0_data_peak * 125. / 8000;
	i2_ampl = adc_1_data_peak * 125. / 8000;

	for (i = 0; i < SMA_LEN - 1; ++i)               // 移動平均リストの各要素を一つずつ左へシフト
	{ 
		i1_ampl_lst_tmp[i] = i1_ampl_lst[i+1];
		i2_ampl_lst_tmp[i] = i2_ampl_lst[i+1];
	}

	i1_ampl_lst_tmp[SMA_LEN-1] = i1_ampl;           // 移動平均リストの末尾に最新の測定値を追加
	i2_ampl_lst_tmp[SMA_LEN-1] = i2_ampl;

	for (i = 0; i < SMA_LEN; ++i)
	{
		i1_ampl_lst[i] = i1_ampl_lst_tmp[i];
		i2_ampl_lst[i] = i2_ampl_lst_tmp[i];
	}

	for (i = 0; i < SMA_LEN; ++i)                   // 移動平均リストの和を計算
	{
		i1_ampl_sum += i1_ampl_lst[i];
		i2_ampl_sum += i2_ampl_lst[i];
	}

	i1_ampl_avg = i1_ampl_sum / SMA_LEN;            // 要素数で除算して移動平均を計算
	i2_ampl_avg = i2_ampl_sum / SMA_LEN;

	i1_ampl_avg = i1_ampl_avg * i1_weight;
	i2_ampl_avg = i2_ampl_avg * i2_weight;
}

//----------------------------------------------------------------------------------------
//　電流制御
//----------------------------------------------------------------------------------------

interrupt void current_control(void)
{
	// float proportional;
	// float integral;
	// float difference;
	// float inv_out_BDN1;
	// float inv_mod_BDN1;

	C6657_timer0_clear_eventflag();

	if (control_on == 1)
	{
		i1_difference = i1_ampl_ref - i1_ampl_avg;
		i1_proportional = i1_difference * proportionalGain;          // 比例ゲイン演算
		i1_integral = i1_integral + i1_difference * integralGain;    // 積分ゲイン演算
		inv_out_BDN1 = i1_proportional + i1_integral;

		i2_difference = i2_ampl_avg - i2_ampl_ref;
		i2_proportional = i2_difference * proportionalGain;          // 比例ゲイン演算
		i2_integral = i2_integral + i2_difference * integralGain;    // 積分ゲイン演算
		inv_out_BDN0 = i2_proportional + i2_integral;

		if (inv_out_BDN1 > 1)
		{
			inv_out_BDN1 = 1;
		}
		else if (inv_out_BDN1 < 0)
		{
			inv_out_BDN1 = 0;
		}

		if (inv_out_BDN0 > 1)
		{
			inv_out_BDN0 = 1;
		}
		else if (inv_out_BDN0 < 0)
		{
			inv_out_BDN0 = 0;
		}

		inv_mod_BDN1 = 1 - inv_out_BDN1;
		uref_BDN0 = inv_out_BDN0 * carrier_cnt_hlf;
		vref_BDN0 = carrier_cnt_max - uref_BDN0;

		PEV_inverter_set_uvw(BDN1, -inv_mod_BDN1, inv_mod_BDN1, 0, 0);
		IPFPGA_write(BDN0, 0x02, uref_BDN0);
		IPFPGA_write(BDN0, 0x03, vref_BDN0);
	}
}

//----------------------------------------------------------------------------------------
//　可変キャパシタ制御
//----------------------------------------------------------------------------------------

interrupt void pwm_capacitor_control(void)
{
	C6657_timer1_clear_eventflag();

	if (pwmc_search_on == 1)
	{
		if (inv_out_BDN0 >= inv_out_BDN0_last && up_down == 1)
		{
			pwm_out_BDN0 = pwm_out_BDN0 + 0.01;
			up_down = 1;
		}
		else if (inv_out_BDN0 >= inv_out_BDN0_last && up_down == -1)
		{
			pwm_out_BDN0 = pwm_out_BDN0 - 0.01;
			up_down = -1;
		}
		else if (inv_out_BDN0 < inv_out_BDN0_last && up_down == 1)
		{
			pwm_out_BDN0 = pwm_out_BDN0 - 0.01;
			up_down = -1;
		}
		else if (inv_out_BDN0 < inv_out_BDN0_last && up_down == -1)
		{
			pwm_out_BDN0 = pwm_out_BDN0 + 0.01;
			up_down = 1;
		}

		if (pwm_out_BDN0 > 1)
		{
			pwm_out_BDN0 = 1;
		}
		else if (pwm_out_BDN0 < 0)
		{
			pwm_out_BDN0 = 0;
		}

		wref_BDN0 = pwm_out_BDN0 * carrier_cnt_max;
		IPFPGA_write(BDN0, 0x04, wref_BDN0);

		inv_out_BDN0_last = inv_out_BDN0;
	}

	else
	{
		wref_BDN0 = pwm_out_BDN0 * carrier_cnt_max;
		IPFPGA_write(BDN0, 0x04, wref_BDN0);
	}
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	int i;

	for (i = 0; i < SMA_LEN; ++i)
	{
		i1_ampl_lst[i] = 0;
		i2_ampl_lst[i] = 0;
	}

	carrier_cnt_max = 50000000. / INV_FREQ;
	carrier_cnt_hlf = 25000000. / INV_FREQ;

	IPFPGA_write(BDN0, 0x01, carrier_cnt_max);
	IPFPGA_write(BDN0, 0x02, carrier_cnt_hlf);
	IPFPGA_write(BDN0, 0x03, carrier_cnt_hlf);
	IPFPGA_write(BDN0, 0x04, carrier_cnt_hlf);
	IPFPGA_write(BDN0, 0x05, DEAD_TIME / 10);
	IPFPGA_write(BDN0, 0x08, REF_RECT);
	IPFPGA_write(BDN0, 0x09, REF_PWMC);

	PEV_init(BDN1);

	int_disable();

	PEV_inverter_disable_int(BDN1);

	PEV_inverter_init(BDN1, INV_FREQ, DEAD_TIME);
	PEV_inverter_init_int_timing(BDN1, 1, 0, 0);
	PEV_int_init(BDN1, 2, 0, 0, 0, 0, 0, 0, 0);
	int0_init_vector(read_envelope, (CSL_IntcVectId)7, FALSE);
	PEV_inverter_control_gate(BDN1, TRANS_MODE);
	PEV_inverter_set_uvw(BDN1, 0, 0, 0, 0);

	C6657_timer0_init(5);
	C6657_timer0_init_vector(current_control, (CSL_IntcVectId)5);
	C6657_timer0_start();

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
