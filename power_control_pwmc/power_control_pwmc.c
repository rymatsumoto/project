//----------------------------------------------------------------------------------------
//title
// 受電電力制御プログラム
//designer
// ryo matsumoto
//----------------------------------------------------------------------------------------

#include    <mwio4.h>

#define BDN0 0				//送電側FPGAボード
#define BDN1 1				//受電側FPGAボード
#define BDN2 0				//PEVボード

#define INV_FREQ 85000
#define DEAD_TIME 300
#define REF_PWMC 270
#define TIMER0_INTERVAL 5
#define TIMER1_INTERVAL 10000
#define TIMER2_INTERVAL 1000

volatile int REF_ZC_CNT = 68;
volatile int LPF_CUTTOFF = 100;   // LPFカットオフ周波数 [Hz]
volatile float P_O_STEP_SIZE = 0.01;
volatile float PWM_OUT_MIN = 0.2;
volatile float PWM_OUT_MAX = 1;
volatile float PWMC_INIT_STEP_SIZE = 0.001;
volatile float P_O_A0 = 0;
volatile float P_O_A1 = 0.01;

volatile int set_pwm_on_trans = -1;
volatile int set_pwm_on_tx_pwmc = -1;
volatile int set_pwm_on_rx_pwmc = -1;
volatile int pwmc_tx_control_on = -1;
volatile int pwmc_rx_control_on = -1;

volatile float pwm_out_BDN0 = 0;
volatile float pwm_out_BDN1 = 0;
volatile int wref_BDN0 = 0;
volatile int wref_BDN1 = 0;
volatile int carrier_cnt_max;
volatile int carrier_cnt_hlf;

volatile int zc_cnt;
volatile int zc_cnt_std;

volatile float proportionalGain = 0.03;
volatile float integralGain = 0.03;
float proportional = 0;
float integral = 0;

float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float data[] = {0., 0., 0., 0.};

volatile float i2dc_ref = 0;
float i2dc_crnt = 0;
float i2dc_prvs = 0;
float i2dc_lpf_crnt = 0;
float i2dc_lpf_prvs = 0;
float lpf_T;
float lpf_A;
float lpf_B;

float i2dc_p_o_crnt = 0;
float i2dc_p_o_prvs = 0;
volatile int up_down = 1;
float i2dc_error = 0;

volatile int count_start = -1;

volatile int counter = -1;
volatile int trans_start = 0;
volatile int pwmc_tx_init_start = 0;
volatile int pwmc_rx_init_start = 0;
volatile int pwmc_tx_control_start = 0;
volatile int pwmc_rx_control_start = 0;
volatile int pwmc_rx_update_start = 0;

volatile float pwmc_rx_update = 0;

int pwmc_tx_init_mode = 0;
int pwmc_rx_init_mode = 0;

float pwm_out_BDN0_ramp = 0;
float pwm_out_BDN1_ramp = 0;

//----------------------------------------------------------------------------------------
// 送電側可変キャパシタ制御（TIMER0）
//----------------------------------------------------------------------------------------

void pwmc_tx_control(void)
{
	if (pwmc_tx_control_on == 1) {
		PEV_pio_out(BDN2, 0x0FFFF);
		proportional = zc_cnt_std * proportionalGain;       // 比例ゲイン演算
		integral = integral + zc_cnt_std * integralGain;    // 積分ゲイン演算
		wref_BDN0 = proportional + integral;

		if (wref_BDN0 > carrier_cnt_max) {
			wref_BDN0 = carrier_cnt_max;
		}
		else if (wref_BDN0 < 0) {
			wref_BDN0 = 0;
		}

		IPFPGA_write(BDN0, 0x04, wref_BDN0);
		pwm_out_BDN0 = (float)wref_BDN0 / carrier_cnt_max;
	}

	else {
		if (pwmc_tx_init_mode == 2){
			wref_BDN0 = pwm_out_BDN0 * carrier_cnt_max;
			IPFPGA_write(BDN0, 0x04, wref_BDN0);
		}
	}
}

//----------------------------------------------------------------------------------------
// I2dcを取得（TIMER0）
//----------------------------------------------------------------------------------------

void read_i2dc(void)
{
	lpf_T = 1 / (2 * 3.14 * LPF_CUTTOFF);
	lpf_A = TIMER0_INTERVAL * 1e-6 / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T);
	lpf_B = (TIMER0_INTERVAL * 1e-6-2 * lpf_T) / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T);

	PEV_ad_in_4ch(BDN2, 0, data);
	i2dc_crnt = data[0] * 25;							// 2V 50A
	i2dc_lpf_crnt = lpf_A * i2dc_crnt + lpf_A * i2dc_prvs - lpf_B * i2dc_lpf_prvs;

	i2dc_prvs = i2dc_crnt;
	i2dc_lpf_prvs = i2dc_lpf_crnt;
}

//----------------------------------------------------------------------------------------
// 送電側制御＋I2dc取得（TIMER0）
//----------------------------------------------------------------------------------------

interrupt void pwmc_tx_control_read_i2dc(void)
{
	C6657_timer0_clear_eventflag();

	pwmc_tx_control();
	read_i2dc();
}

//----------------------------------------------------------------------------------------
// 受電側可変キャパシタ制御（TIMER1）
//----------------------------------------------------------------------------------------

interrupt void pwmc_rx_control(void)
{
	C6657_timer1_clear_eventflag();

	if (pwmc_rx_control_on == 1) {

		i2dc_p_o_crnt = i2dc_lpf_crnt;

		i2dc_error = mwabs(i2dc_ref - i2dc_lpf_crnt);
		P_O_STEP_SIZE = P_O_A1 * i2dc_error + P_O_A0;

		if (up_down == 1)
		{
			if (i2dc_p_o_crnt >= i2dc_p_o_prvs)
			{
				if (i2dc_p_o_crnt >= i2dc_ref) {
					pwm_out_BDN1 = pwm_out_BDN1 - P_O_STEP_SIZE;
					up_down = -1;
				}
				else {
					pwm_out_BDN1 = pwm_out_BDN1 + P_O_STEP_SIZE;
					up_down = 1;
				}
			}
			else
			{
				if (i2dc_p_o_crnt >= i2dc_ref) {
					pwm_out_BDN1 = pwm_out_BDN1 + P_O_STEP_SIZE;
					up_down = 1;
				}
				else {
					pwm_out_BDN1 = pwm_out_BDN1 - P_O_STEP_SIZE;
					up_down = -1;
				}
			}
		}
		else if (up_down == -1)
		{
			if (i2dc_p_o_crnt >= i2dc_p_o_prvs)
			{
				if (i2dc_p_o_crnt >= i2dc_ref) {
					pwm_out_BDN1 = pwm_out_BDN1 + P_O_STEP_SIZE;
					up_down = 1;
				}
				else {
					pwm_out_BDN1 = pwm_out_BDN1 - P_O_STEP_SIZE;
					up_down = -1;
				}
			}
			else
			{
				if (i2dc_p_o_crnt >= i2dc_ref) {
					pwm_out_BDN1 = pwm_out_BDN1 - P_O_STEP_SIZE;
					up_down = -1;
				}
				else {
					pwm_out_BDN1 = pwm_out_BDN1 + P_O_STEP_SIZE;
					up_down = 1;
				}
			}
		}

		if (pwm_out_BDN1 > PWM_OUT_MAX) {
			pwm_out_BDN1 = PWM_OUT_MAX;
		}
		else if (pwm_out_BDN1 < PWM_OUT_MIN) {
			pwm_out_BDN1 = PWM_OUT_MIN;
		}

		wref_BDN1 = pwm_out_BDN1 * carrier_cnt_max;
		IPFPGA_write(BDN1, 0x04, wref_BDN1);

		i2dc_p_o_prvs = i2dc_p_o_crnt;

		// if (P_O_STEP_SIZE > 0.001)
		// {
		// 	P_O_STEP_SIZE -= 0.0001;
		// }
	}

	else {
		if (pwmc_rx_init_mode == 2){
			wref_BDN1 = pwm_out_BDN1 * carrier_cnt_max;
			IPFPGA_write(BDN1, 0x04, wref_BDN1);
		}
	}
}

//----------------------------------------------------------------------------------------
// カウンター（TIMER2）
//----------------------------------------------------------------------------------------

interrupt void increment_counter(void)
{
	C6657_timer2_clear_eventflag();

	if (count_start == 1){
		counter = counter + 1;
	}
}

//----------------------------------------------------------------------------------------
// PWMC初期化（キャリア同期割り込み）
//----------------------------------------------------------------------------------------

void pwmc_init(void)
{
	if (pwmc_tx_init_mode == 1)
	{
		if (pwm_out_BDN0_ramp < pwm_out_BDN0) {
			pwm_out_BDN0_ramp += PWMC_INIT_STEP_SIZE;
			wref_BDN0 = pwm_out_BDN0_ramp * carrier_cnt_max;
			IPFPGA_write(BDN0, 0x04, wref_BDN0);
		}
		else {
			pwmc_tx_init_mode = 2;
		}
	}

	if (pwmc_rx_init_mode == 1)
	{
		if (pwm_out_BDN1_ramp < pwm_out_BDN1) {
			pwm_out_BDN1_ramp += PWMC_INIT_STEP_SIZE;
			wref_BDN1 = pwm_out_BDN1_ramp * carrier_cnt_max;
			IPFPGA_write(BDN1, 0x04, wref_BDN1);
		}
		else {
			pwmc_rx_init_mode = 2;
		}
	}
}

//----------------------------------------------------------------------------------------
// V1とI1の位相差を取得（キャリア同期割り込み）
//----------------------------------------------------------------------------------------

void read_zc_cnt(void)
{
	zc_cnt = IPFPGA_read(BDN0, 0x19);

	if (zc_cnt <= carrier_cnt_max) {
		zc_cnt_std = REF_ZC_CNT - zc_cnt;
	}
	else if (carrier_cnt_max < zc_cnt && zc_cnt <= carrier_cnt_max * 2) {
		zc_cnt_std = carrier_cnt_max * 2 + REF_ZC_CNT - zc_cnt;
	}
	else {
		zc_cnt_std = REF_ZC_CNT;
	}
}

//----------------------------------------------------------------------------------------
// PWMC初期化＋V1とI1の位相差を取得（キャリア同期割り込み）
//----------------------------------------------------------------------------------------

interrupt void pwmc_init_read_zc_cnt(void)
{
	int0_ack();

	pwmc_init();
	read_zc_cnt();
}

//----------------------------------------------------------------------------------------
// 初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	PEV_ad_set_range(BDN2, range);
	PEV_ad_set_mode(BDN2, 0);

	carrier_cnt_max = 50000000. / INV_FREQ;
	carrier_cnt_hlf = 25000000. / INV_FREQ;
	
	IPFPGA_write(BDN0, 0x01, carrier_cnt_max);
	IPFPGA_write(BDN0, 0x02, carrier_cnt_hlf);
	IPFPGA_write(BDN0, 0x03, carrier_cnt_hlf);
	IPFPGA_write(BDN0, 0x04, wref_BDN0);
	IPFPGA_write(BDN0, 0x05, DEAD_TIME / 10);
	IPFPGA_write(BDN0, 0x09, REF_PWMC);

	IPFPGA_write(BDN1, 0x01, carrier_cnt_max);
	IPFPGA_write(BDN1, 0x02, carrier_cnt_hlf);
	IPFPGA_write(BDN1, 0x03, carrier_cnt_hlf);
	IPFPGA_write(BDN1, 0x04, wref_BDN1);
	IPFPGA_write(BDN1, 0x05, DEAD_TIME / 10);
	IPFPGA_write(BDN1, 0x09, REF_PWMC);

	PEV_init(BDN2);

	int_disable();

	PEV_inverter_disable_int(BDN2);

	PEV_inverter_init(BDN2, INV_FREQ, DEAD_TIME);
	PEV_inverter_init_int_timing(BDN2, 1, 0, 0);
	PEV_int_init(BDN2, 2, 0, 0, 0, 0, 0, 0, 0);
	int0_init_vector(pwmc_init_read_zc_cnt, (CSL_IntcVectId)6, FALSE);

	C6657_timer0_init(TIMER0_INTERVAL);
	C6657_timer0_init_vector(pwmc_tx_control_read_i2dc, (CSL_IntcVectId)5);
	C6657_timer0_start();

	C6657_timer1_init(TIMER1_INTERVAL);
	C6657_timer1_init_vector(pwmc_rx_control, (CSL_IntcVectId)7);
	C6657_timer1_start();

	C6657_timer2_init(TIMER2_INTERVAL);
	C6657_timer2_init_vector(increment_counter, (CSL_IntcVectId)8);
	C6657_timer2_start();

	PEV_inverter_enable_int(BDN2);

	int0_enable_int();

	C6657_timer0_enable_int();
	C6657_timer1_enable_int();
	C6657_timer2_enable_int();

	int_enable();
}

//----------------------------------------------------------------------------------------
// メイン
//----------------------------------------------------------------------------------------

void MW_main(void)
{
	initialize();

	while(1)
	{
		if (counter == trans_start) {
			IPFPGA_write(BDN0, 0x06, 1);
		}
		if (counter == pwmc_tx_init_start) {
			IPFPGA_write(BDN0, 0x07, 1);
			pwmc_tx_init_mode = 1;
		}
		if (counter == pwmc_rx_init_start) {
			IPFPGA_write(BDN1, 0x07, 1);
			pwmc_rx_init_mode = 1;
		}
		if (counter == pwmc_tx_control_start) {
			pwmc_tx_control_on = 1;
		}
		if (counter == pwmc_rx_control_start) {
			pwmc_rx_control_on = 1;
		}
		if (counter == pwmc_rx_update_start) {
			pwm_out_BDN1 = pwmc_rx_update;
		}
	}
}
