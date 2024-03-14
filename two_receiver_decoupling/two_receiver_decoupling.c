/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include <mwio4.h>

#define BDN_PEV 0   //PEVボード
#define BDN_FPGA1 0 //FPGAボードRX1
#define BDN_FPGA2 1 //FPGAボードRX2

			                //WN WP VN VP UN UP
#define TRANS_MODE 80 		//00 00 01 01 00 00
#define INV_FREQ 100000
#define DEAD_TIME 500
#define TIMER0_INTERVAL 1e3
#define TIMER1_INTERVAL 200e3
#define TIMER2_INTERVAL 1e3
#define RL1 10
#define RL2 10

#define UP 1
#define DOWN -1
#define UPDATE_RX1 1
#define UPDATE_RX2 2

#define FPGA_CLK_FREQ 100e6
#define FRAC_WIDTH 32

#define DUTY_LOG_SIZE 50
#define DUTY_UPDATE_LOG_SIZE 10
#define DUTY_UPDATE_LOG_INIT_VALUE 100
#define STABILITY_CHECK_LOG_SIZE 50
#define POWER_LOG_SIZE 100

#define PI 3.14
#define EPSILON 0.0001

#define P_O_STEP_SIZE 0.05

volatile int REF_PWMC_RX1 = 225;
volatile int REF_PWMC_RX2 = 225;
volatile int LPF_CUTTOFF_dsp = 1e3;
volatile int LPF_CUTTOFF_0 = 200e3;
volatile int LPF_CUTTOFF_1 = 1e2;
volatile int LPF_CUTTOFF_2 = 1e2;

volatile int start_current_control = 0;
volatile int start_pwmc_rx1_init = 0;
volatile int start_pwmc_rx2_init = 0;
volatile int start_pwmc_control = 0;
int current_control_on = 0;
int pwmc_control_on = 0;

INT32 ad_0_data_lpf_peak;
INT32 ad_1_data_lpf_crnt;
INT32 ad_2_data_lpf_crnt;
INT32 ad_3_data_lpf_crnt;
float itx_ampl = 0;
volatile float itx_weight = 1.23;
volatile float itx_ampl_ref;

float error_crnt = 0;
float error_prvs = 0;
float error_integral = 0;
float vtx_ampl_ref = 0;
float inv_mod = 0;
float period = 1 / (float)INV_FREQ;
volatile float vdc = 30;
volatile float proportionalGain = 1;
volatile float integralGain = 1e3;

int state = 0;
volatile float pwm_out_rx1 = 0;
volatile float pwm_out_rx2 = 0;
volatile int wref_rx1 = 0;
volatile int wref_rx2 = 0;
volatile int carrier_cnt_max;
volatile int carrier_cnt_hlf;
int up_down = 1;
int up_down_rx1 = 1;
int up_down_rx2 = 1;

float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float data[] = {0., 0., 0., 0.};
float dc_current_rx1_lpf;
float dc_current_rx2_lpf;

float lpf_T_dsp;
float lpf_A_dsp;
float lpf_B_dsp;

float lpf_T_0;
float lpf_A_0;
float lpf_B_0;

float lpf_T_1;
float lpf_A_1;
float lpf_B_1;

float lpf_T_2;
float lpf_A_2;
float lpf_B_2;

const float Gth = 0.1042;

float power_rx1 = 0;
float power_rx2 = 0;
float power_total = 0;
float power_total_last = 0;
float max_power = 0;
int update_mode = 1;

float duty_update_log_rx1[DUTY_UPDATE_LOG_SIZE];
float duty_update_log_rx2[DUTY_UPDATE_LOG_SIZE];

float duty_log_rx1[DUTY_LOG_SIZE];
float duty_log_rx2[DUTY_LOG_SIZE];

int stability_check_log[STABILITY_CHECK_LOG_SIZE];
int stability_check = 0;

float duty_update_avg_rx1 = DUTY_UPDATE_LOG_INIT_VALUE;
float duty_update_avg_rx2 = DUTY_UPDATE_LOG_INIT_VALUE;

float duty_avg_rx1 = 0;
float duty_avg_rx2 = 0;

float stability_thld = P_O_STEP_SIZE * 2 / DUTY_UPDATE_LOG_SIZE + EPSILON;

float power_log[POWER_LOG_SIZE];
float power_total_avg;

//----------------------------------------------------------------------------------------
//　移動平均
//----------------------------------------------------------------------------------------

float moving_average(float* moving_average_array_p, int array_size, float update_value)
{
	int i;
	float sum = 0;

	for (i = 0; i < array_size-1; i++){
		moving_average_array_p[i] = moving_average_array_p[i+1];
	}
	moving_average_array_p[array_size-1] = update_value;

	for (i = 0; i < array_size; i++){
		sum += moving_average_array_p[i];
	}

	return sum / array_size;
}

//----------------------------------------------------------------------------------------
//　arccos関数
//----------------------------------------------------------------------------------------

float arccos(float x)
{
    float y = mwarctan(mwsqrt(1-x*x) / x);
    return y;
}

//----------------------------------------------------------------------------------------
//　FPGAに書き込むLPF定数の計算
//----------------------------------------------------------------------------------------

int convert_binary(float f)
{
    int b[FRAC_WIDTH];
    int d = 0;
    int q;
    float r = f;
	int i;
	int j;

    float divisor = 0.5;

    for (i = 0; i < FRAC_WIDTH; i++)
    {
        q = r / divisor;
        b[i] = q;
        r = r - divisor * q;
        divisor = divisor / 2;
    }

    for (i = 0; i < FRAC_WIDTH; i++)
    {
        int pow = 1;

        for (j = 0; j < i; j++)
        {
            pow = 2 * pow;
        }
        d += b[FRAC_WIDTH-1-i] * pow;
    }

    return d;
}

//----------------------------------------------------------------------------------------
//　リミッター
//----------------------------------------------------------------------------------------

float limitter(float input_value, float lower_limit, float upper_limit)
{
	float output_value;

	if (input_value > upper_limit) {
		output_value = upper_limit;
	}
	else if (input_value < lower_limit) {
		output_value = lower_limit;
	}
	else {
		output_value = input_value;
	}

	return output_value;
}

//----------------------------------------------------------------------------------------
//　出力電流読み出し
//----------------------------------------------------------------------------------------

interrupt void read_dc_current(void)
{
	C6657_timer0_clear_eventflag();

	// PEV_ad_in_4ch(BDN_PEV, 0, data);
	// dc_current_rx1 = (2.5 - data[0]) / Gth / 3;
	// dc_current_rx2 = (2.5 - data[1]) / Gth / 3;

	// lpf_T = 1 / (2 * 3.14 * LPF_CUTTOFF);
	// lpf_A = TIMER0_INTERVAL * 1e-6 / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T);
	// lpf_B = (TIMER0_INTERVAL * 1e-6 - 2 * lpf_T) / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T);

	// dc_current_rx1_lpf = lpf_A * dc_current_rx1 + lpf_A * dc_current_rx1_prvs - lpf_B * dc_current_rx1_lpf_prvs;
	// dc_current_rx2_lpf = lpf_A * dc_current_rx2 + lpf_A * dc_current_rx2_prvs - lpf_B * dc_current_rx2_lpf_prvs;

	// dc_current_rx1_prvs = dc_current_rx1;
	// dc_current_rx2_prvs = dc_current_rx2;
	// dc_current_rx1_lpf_prvs = dc_current_rx1_lpf;
	// dc_current_rx2_lpf_prvs = dc_current_rx2_lpf;

	ad_1_data_lpf_crnt = IPFPGA_read(BDN_FPGA1, 0x18);
	ad_2_data_lpf_crnt = IPFPGA_read(BDN_FPGA1, 0x19);

	dc_current_rx1_lpf = (2.5 - ad_1_data_lpf_crnt * 5. / 8000.) / Gth / 3;
	dc_current_rx2_lpf = (2.5 - ad_2_data_lpf_crnt * 5. / 8000.) / Gth / 3;

	power_rx1 = RL1 * dc_current_rx1_lpf * dc_current_rx1_lpf;
	power_rx2 = RL2 * dc_current_rx2_lpf * dc_current_rx2_lpf;

	power_total = power_rx1 + power_rx2;

	power_total_avg = moving_average(power_log, POWER_LOG_SIZE, power_total);
}

//----------------------------------------------------------------------------------------
//　可変キャパシタ制御
//----------------------------------------------------------------------------------------

interrupt void pwmc_control(void)
{
	C6657_timer1_clear_eventflag();

	if (pwmc_control_on == 1) {

		if (stability_check == 0) {
			if (update_mode == UPDATE_RX1) {
				if (power_total_avg >= power_total_last) {
					if (up_down_rx2 == UP) {
						up_down_rx2 = UP;
					}
					else if (up_down_rx2 == DOWN) {
						up_down_rx2 = DOWN;
					}
				}
				else if (power_total_avg < power_total_last) {
					if (up_down_rx2 == UP) {
						up_down_rx2 = DOWN;
					}
					else if (up_down_rx2 == DOWN) {
						up_down_rx2 = UP;
					}
				}

				if (up_down_rx1 == UP) {
					pwm_out_rx1 = pwm_out_rx1 + P_O_STEP_SIZE;
					duty_update_avg_rx1 = moving_average(duty_update_log_rx1, DUTY_UPDATE_LOG_SIZE, P_O_STEP_SIZE);
				}
				else if (up_down_rx1 == DOWN) {
					pwm_out_rx1 = pwm_out_rx1 - P_O_STEP_SIZE;
					duty_update_avg_rx1 = moving_average(duty_update_log_rx1, DUTY_UPDATE_LOG_SIZE, -P_O_STEP_SIZE);
				}

				update_mode = UPDATE_RX2;
			}

			else if (update_mode == UPDATE_RX2) {
				if (power_total_avg >= power_total_last) {
					if (up_down_rx1 == UP) {
						up_down_rx1 = UP;
					}
					else if (up_down_rx1 == DOWN) {
						up_down_rx1 = DOWN;
					}
				}
				else if (power_total_avg < power_total_last) {
					if (up_down_rx1 == UP) {
						up_down_rx1 = DOWN;
					}
					else if (up_down_rx1 == DOWN) {
						up_down_rx1 = UP;
					}
				}

				if (up_down_rx2 == UP) {
					pwm_out_rx2 = pwm_out_rx2 + P_O_STEP_SIZE;
					duty_update_avg_rx2 = moving_average(duty_update_log_rx2, DUTY_UPDATE_LOG_SIZE, P_O_STEP_SIZE);
				}
				else if (up_down_rx2 == DOWN) {
					pwm_out_rx2 = pwm_out_rx2 - P_O_STEP_SIZE;
					duty_update_avg_rx2 = moving_average(duty_update_log_rx2, DUTY_UPDATE_LOG_SIZE, -P_O_STEP_SIZE);
				}

				update_mode = UPDATE_RX1;
			}

			pwm_out_rx1 = limitter(pwm_out_rx1, 0, 1);
			pwm_out_rx2 = limitter(pwm_out_rx2, 0, 1);

			duty_avg_rx1 = moving_average(duty_log_rx1, DUTY_LOG_SIZE, pwm_out_rx1); // pwm_out_rx1の移動平均を求める
			duty_avg_rx2 = moving_average(duty_log_rx2, DUTY_LOG_SIZE, pwm_out_rx2); // pwm_out_rx2の移動平均を求める

			power_total_last = power_total_avg;

			// stability_check_logを更新
			int i;
			for (i = 0; i < STABILITY_CHECK_LOG_SIZE - 1; i++) {
				stability_check_log[i] = stability_check_log[i+1];
			}

			// duty_update_avg_rx1とduty_update_avg_rx2がともにstability_thld以下かチェック
			stability_check_log[STABILITY_CHECK_LOG_SIZE-1] = 0;
			if (duty_update_avg_rx1 >= -stability_thld && duty_update_avg_rx1 <= stability_thld) {
				if (duty_update_avg_rx2 >= -stability_thld && duty_update_avg_rx2 <= stability_thld) {
					stability_check_log[STABILITY_CHECK_LOG_SIZE-1] = 1;
				}
			}	
			
			// stability_check_logの要素が全て1ならstability_checkを1にする
			stability_check = 1;
			for (i = 0; i < STABILITY_CHECK_LOG_SIZE; i++) {
				stability_check = stability_check * stability_check_log[i];
			}
		}
		else if (stability_check == 1) {
			pwm_out_rx1 = duty_avg_rx1;
			pwm_out_rx2 = duty_avg_rx2;
		}
	}

	wref_rx1 = pwm_out_rx1 * carrier_cnt_max;
	wref_rx2 = pwm_out_rx2 * carrier_cnt_max;

	IPFPGA_write(BDN_FPGA1, 0x04, wref_rx1);
	IPFPGA_write(BDN_FPGA2, 0x04, wref_rx2);
}

//----------------------------------------------------------------------------------------
//　電流制御
//----------------------------------------------------------------------------------------

interrupt void current_control(void)
{
	int0_ack();

    ad_0_data_lpf_peak = IPFPGA_read(BDN_FPGA1, 0x17);
	itx_ampl = ad_0_data_lpf_peak * 125. / 8000 * itx_weight;

    if (current_control_on == 1)
    {
        error_crnt = itx_ampl_ref - itx_ampl;
        error_integral = error_integral + (error_crnt + error_prvs) / 2 * period;
        vtx_ampl_ref = proportionalGain * error_crnt + integralGain * error_integral;
        error_prvs = error_crnt;

		vtx_ampl_ref = limitter(vtx_ampl_ref, 0, vdc * 4 / PI);

        inv_mod = 2 / PI * arccos(PI * vtx_ampl_ref / (4 * vdc));

        inv_mod = limitter(inv_mod, 0, 1);

        PEV_inverter_set_uvw(BDN_PEV, -inv_mod, inv_mod, 0, 0);
    }
}

//----------------------------------------------------------------------------------------
//　FPGA設定の更新
//----------------------------------------------------------------------------------------

interrupt void update_fpga_config(void)
{
	C6657_timer2_clear_eventflag();

	lpf_T_dsp = 1 / (2 * PI * LPF_CUTTOFF_dsp);
	lpf_A_dsp = 1. / INV_FREQ / (1. / INV_FREQ + 2 * lpf_T_dsp);
	lpf_B_dsp = (2 * lpf_T_dsp - 1. / INV_FREQ) / (2 * lpf_T_dsp + 1. / INV_FREQ);

	lpf_T_0 = 1 / (2 * PI * LPF_CUTTOFF_0);
	lpf_A_0 = 1. / FPGA_CLK_FREQ / (1. / FPGA_CLK_FREQ + 2 * lpf_T_0);
	lpf_B_0 = (2 * lpf_T_0 - 1. / FPGA_CLK_FREQ) / (2 * lpf_T_0 + 1. / FPGA_CLK_FREQ);

	lpf_T_1 = 1 / (2 * PI * LPF_CUTTOFF_1);
	lpf_A_1 = 1. / FPGA_CLK_FREQ / (1. / FPGA_CLK_FREQ + 2 * lpf_T_1);
	lpf_B_1 = (2 * lpf_T_1 - 1. / FPGA_CLK_FREQ) / (2 * lpf_T_1 + 1. / FPGA_CLK_FREQ);

	lpf_T_2 = 1 / (2 * PI * LPF_CUTTOFF_2);
	lpf_A_2 = 1. / FPGA_CLK_FREQ / (1. / FPGA_CLK_FREQ + 2 * lpf_T_2);
	lpf_B_2 = (2 * lpf_T_2 - 1. / FPGA_CLK_FREQ) / (2 * lpf_T_2 + 1. / FPGA_CLK_FREQ);

	IPFPGA_write(BDN_FPGA1, 0x15, convert_binary(lpf_A_0));
	IPFPGA_write(BDN_FPGA1, 0x16, convert_binary(lpf_B_0));
	
	IPFPGA_write(BDN_FPGA1, 0x17, convert_binary(lpf_A_1));
	IPFPGA_write(BDN_FPGA1, 0x18, convert_binary(lpf_B_1));

	IPFPGA_write(BDN_FPGA1, 0x19, convert_binary(lpf_A_2));
	IPFPGA_write(BDN_FPGA1, 0x1a, convert_binary(lpf_B_2));

	IPFPGA_write(BDN_FPGA1, 0x09, REF_PWMC_RX1);
	IPFPGA_write(BDN_FPGA2, 0x09, REF_PWMC_RX2);
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	int i;

	for (i = 0; i < DUTY_UPDATE_LOG_SIZE; i++) {
		duty_update_log_rx1[i] = DUTY_UPDATE_LOG_INIT_VALUE;
		duty_update_log_rx2[i] = DUTY_UPDATE_LOG_INIT_VALUE;
	}
	for (i = 0; i < DUTY_LOG_SIZE; i++) {
		duty_log_rx1[i] = 0;
		duty_log_rx2[i] = 0;
	}
	for (i = 0; i < STABILITY_CHECK_LOG_SIZE; i++) {
		stability_check_log[i] = 0;
	}
	for (i = 0; i < POWER_LOG_SIZE; i++) {
		power_log[i] = 0;
	}

	PEV_ad_set_range(BDN_PEV, range);
	PEV_ad_set_mode(BDN_PEV, 0);

	carrier_cnt_max = 50000000. / INV_FREQ;
	carrier_cnt_hlf = 25000000. / INV_FREQ;
	
	IPFPGA_write(BDN_FPGA1, 0x01, carrier_cnt_max);
	IPFPGA_write(BDN_FPGA1, 0x02, carrier_cnt_hlf);
	IPFPGA_write(BDN_FPGA1, 0x03, carrier_cnt_hlf);
	IPFPGA_write(BDN_FPGA1, 0x04, wref_rx1);
	IPFPGA_write(BDN_FPGA1, 0x05, DEAD_TIME / 10);
	IPFPGA_write(BDN_FPGA1, 0x09, REF_PWMC_RX1);

	IPFPGA_write(BDN_FPGA2, 0x01, carrier_cnt_max);
	IPFPGA_write(BDN_FPGA2, 0x02, carrier_cnt_hlf);
	IPFPGA_write(BDN_FPGA2, 0x03, carrier_cnt_hlf);
	IPFPGA_write(BDN_FPGA2, 0x04, wref_rx2);
	IPFPGA_write(BDN_FPGA2, 0x05, DEAD_TIME / 10);
	IPFPGA_write(BDN_FPGA2, 0x09, REF_PWMC_RX2);

	int_disable();

	PEV_init(BDN_PEV);
    PEV_inverter_disable_int(BDN_PEV);
    PEV_inverter_init(BDN_PEV, INV_FREQ, DEAD_TIME);
    PEV_inverter_init_int_timing(BDN_PEV, 1, 0, 0);
    PEV_int_init(BDN_PEV, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(current_control, (CSL_IntcVectId)4, FALSE);
    PEV_inverter_control_gate(BDN_PEV, TRANS_MODE);
    PEV_inverter_set_uvw(BDN_PEV, 0, 0, 0, 0);

	C6657_timer0_init(TIMER0_INTERVAL);
	C6657_timer0_init_vector(read_dc_current, (CSL_IntcVectId)5);
	C6657_timer0_start();

	C6657_timer1_init(TIMER1_INTERVAL);
	C6657_timer1_init_vector(pwmc_control, (CSL_IntcVectId)6);
	C6657_timer1_start();

	C6657_timer2_init(TIMER2_INTERVAL);
	C6657_timer2_init_vector(update_fpga_config, (CSL_IntcVectId)7);
	C6657_timer2_start();

    PEV_inverter_enable_int(BDN_PEV);
    int0_enable_int();
    C6657_timer0_enable_int();
	C6657_timer1_enable_int();
	C6657_timer2_enable_int();
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
		if (start_current_control == 1) {
			PEV_inverter_start_pwm(BDN_PEV);
			current_control_on = 1;
			start_current_control = -1;
		}
		if (start_pwmc_rx1_init == 1) {
			IPFPGA_write(BDN_FPGA1, 0x07, 1);
			start_pwmc_rx1_init = -1;
		}
		if (start_pwmc_rx2_init == 1) {
			IPFPGA_write(BDN_FPGA2, 0x07, 1);
			start_pwmc_rx2_init = -1;
		}
		if (start_pwmc_control == 1) {
			pwmc_control_on = 1;
			start_pwmc_control = -1;
		}
	}
}
