//----------------------------------------------------------------------------------------
//title
// 両側可変キャパシタ制御プログラム
//designer
// ryo matsumoto
//----------------------------------------------------------------------------------------

#include    <mwio4.h>

#define BDN0 0				//送電側FPGAボード
#define BDN1 1				//受電側FPGAボード
#define BDN2 0				//PEVボード

#define INV_FREQ 85000
#define DEAD_TIME 300
// #define REF_RECT 1108
#define REF_PWMC 260
// #define REF_ZC_CNT 70

#define SMA_LEN 20

volatile int REF_RECT = 1108;
volatile int REF_ZC_CNT = 80;

volatile int set_pwm_on_trans = -1;
volatile int set_pwm_on_rect = -1;
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

volatile float i2dc;
float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float data[] = {0., 0., 0., 0.};

float i2dc_lst[SMA_LEN];
float i2dc_lst_tmp[SMA_LEN];
float i2dc_avg = 0;
float i2dc_sum = 0;
float i2dc_avg_last = 0;

volatile int up_down = 1;

volatile int count_start = -1;

volatile int counter = -1;
volatile int trans_start = 0;
volatile int rect_start = 0;
volatile int pwmc_tx_init_start = 0;
volatile int pwmc_rx_init_start = 0;
volatile int pwmc_tx_control_start = 0;
volatile int pwmc_rx_control_start = 0;
volatile int pwmc_rx_update_start = 0;

volatile float pwmc_rx_update = 0;

//----------------------------------------------------------------------------------------
// 送電側可変キャパシタ制御
//----------------------------------------------------------------------------------------

interrupt void pwmc_tx_control(void)
{
	C6657_timer0_clear_eventflag();

	if (pwmc_tx_control_on == 1) {
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
		wref_BDN0 = pwm_out_BDN0 * carrier_cnt_max;
		IPFPGA_write(BDN0, 0x04, wref_BDN0);
	}
}
//----------------------------------------------------------------------------------------
// 受電側可変キャパシタ制御
//----------------------------------------------------------------------------------------

interrupt void pwmc_rx_control(void)
{
	C6657_timer1_clear_eventflag();

	if (pwmc_rx_control_on == 1) {
		if (i2dc_avg < i2dc_avg_last && up_down == 1)
		{
			pwm_out_BDN1 = pwm_out_BDN1 + 0.005;
			up_down = 1;
		}
		else if (i2dc_avg < i2dc_avg_last && up_down == -1)
		{
			pwm_out_BDN1 = pwm_out_BDN1 - 0.005;
			up_down = -1;
		}
		else if (i2dc_avg >= i2dc_avg_last && up_down == 1)
		{
			pwm_out_BDN1 = pwm_out_BDN1 - 0.005;
			up_down = -1;
		}
		else if (i2dc_avg >= i2dc_avg_last && up_down == -1)
		{
			pwm_out_BDN1 = pwm_out_BDN1 + 0.005;
			up_down = 1;
		}

		if (pwm_out_BDN1 > 1) {
			pwm_out_BDN1 = 1;
		}
		else if (pwm_out_BDN1 < 0) {
			pwm_out_BDN1 = 0;
		}

		wref_BDN1 = pwm_out_BDN1 * carrier_cnt_max;
		IPFPGA_write(BDN1, 0x04, wref_BDN1);

		i2dc_avg_last = i2dc_avg;
	}

	else {
		wref_BDN1 = pwm_out_BDN1 * carrier_cnt_max;
		IPFPGA_write(BDN1, 0x04, wref_BDN1);
	}

	if (count_start == 1){
		counter = counter + 1;
	}
}

//----------------------------------------------------------------------------------------
// V1とI1の位相差を取得
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
// I2dcの移動平均を取得
//----------------------------------------------------------------------------------------

void read_i2dc_avg(void)
{
	PEV_ad_in_4ch(BDN2, 0, data);
	i2dc = data[0] * 25;							// 2V 50A

	int i;

	for (i = 0; i < SMA_LEN - 1; ++i) {             // 移動平均リストの各要素を一つずつ左へシフト 
		i2dc_lst_tmp[i] = i2dc_lst[i+1];
	}
	i2dc_lst_tmp[SMA_LEN-1] = i2dc;	                // 移動平均リストの末尾に最新の測定値を追加
	for (i = 0; i < SMA_LEN; ++i) {
		i2dc_lst[i] = i2dc_lst_tmp[i];
	}
	i2dc_sum = 0;
	for (i = 0; i < SMA_LEN; ++i) {                 // 移動平均リストの和
		i2dc_sum += i2dc_lst[i];
	}
	i2dc_avg = i2dc_sum / SMA_LEN; 		            // 要素数で除算
}

//----------------------------------------------------------------------------------------
// フィードバック情報取得
//----------------------------------------------------------------------------------------

interrupt void read_feedback_signal(void)
{
	int0_ack();
	read_zc_cnt();
	read_i2dc_avg();
}

//----------------------------------------------------------------------------------------
// 初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	PEV_ad_set_range(BDN2, range);
	PEV_ad_set_mode(BDN2, 1);

	int i;

	for (i = 0; i < SMA_LEN; ++i) {
		i2dc_lst[i] = 0;
	}

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
	IPFPGA_write(BDN1, 0x08, REF_RECT);
	IPFPGA_write(BDN1, 0x09, REF_PWMC);

	PEV_init(BDN2);

	int_disable();

	PEV_inverter_disable_int(BDN2);

	PEV_inverter_init(BDN2, INV_FREQ, DEAD_TIME);
	PEV_inverter_init_int_timing(BDN2, 1, 0, 0);
	PEV_int_init(BDN2, 2, 0, 0, 0, 0, 0, 0, 0);
	int0_init_vector(read_feedback_signal, (CSL_IntcVectId)6, FALSE);

	C6657_timer0_init(5);
	C6657_timer0_init_vector(pwmc_tx_control, (CSL_IntcVectId)5);
	C6657_timer0_start();

	C6657_timer1_init(1000);
	C6657_timer1_init_vector(pwmc_rx_control, (CSL_IntcVectId)7);
	C6657_timer1_start();

	PEV_inverter_enable_int(BDN2);

	int0_enable_int();

	C6657_timer0_enable_int();
	C6657_timer1_enable_int();

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
		if (counter == rect_start) {
			IPFPGA_write(BDN1, 0x08, REF_RECT);
			IPFPGA_write(BDN1, 0x06, 1);
		}
		if (counter == pwmc_tx_init_start) {
			IPFPGA_write(BDN0, 0x07, 1);
		}
		if (counter == pwmc_rx_init_start) {
			IPFPGA_write(BDN1, 0x07, 1);
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
