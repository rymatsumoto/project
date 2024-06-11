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
#define TIMER0_INTERVAL 10
#define TIMER1_INTERVAL 300e3
#define TIMER2_INTERVAL 1e3
#define RL1 10
#define RL2 10

#define FPGA_CLK_FREQ 100e6
#define FRAC_WIDTH 32

#define POWER_LOG_SIZE 200

#define REF_PWMC_RX1 225
#define REF_PWMC_RX2 225

#define LPF_CUTTOFF_dsp 2
#define LPF_CUTTOFF_0 200e3
#define LPF_CUTTOFF_1 1e2
#define LPF_CUTTOFF_2 1e2

#define PI 3.14

#define INCREMENT_RX1 1
#define DECREMENT_RX1 2
#define INCREMENT_RX2 3
#define DECREMENT_RX2 4
#define UPDATE_RX1_RX2 5
#define STOP 6

#define DELTA 0.025

volatile float alpha = 1e-5;
volatile float epsilon = 1e-3;

volatile int start_inv_w_o_control = 0;
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

volatile float pwm_out_rx1 = 0;
volatile float pwm_out_rx2 = 0;
volatile int wref_rx1 = 0;
volatile int wref_rx2 = 0;
volatile int carrier_cnt_max;
volatile int carrier_cnt_hlf;

float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float data[] = {0., 0., 0., 0.};
float dc_current_rx1 = 0;
float dc_current_rx2 = 0;
float dc_current_rx1_lpf = 0;
float dc_current_rx2_lpf = 0;
float dc_current_rx1_prvs = 0;
float dc_current_rx2_prvs = 0;
float dc_current_rx1_lpf_prvs = 0;
float dc_current_rx2_lpf_prvs = 0;

volatile float dc_current_rx1_weight = 1;
volatile float dc_current_rx2_weight = 1;

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
float power_total_avg = 0;
float power_total_updated = 0;

float power_log[POWER_LOG_SIZE];

volatile float duty_w_o_control = 1;

int state = INCREMENT_RX1;

float power_total_increment_rx1 = 0;
float power_total_decrement_rx1 = 0;
float power_total_increment_rx2 = 0;
float power_total_decrement_rx2 = 0;

float gradient_rx1 = 0;
float gradient_rx2 = 0;

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

	ad_1_data_lpf_crnt = IPFPGA_read(BDN_FPGA1, 0x18);
	ad_2_data_lpf_crnt = IPFPGA_read(BDN_FPGA1, 0x19);

	dc_current_rx1 = (2.5 - ad_1_data_lpf_crnt * 5. / 8000.) / Gth / 3 * dc_current_rx1_weight;
	dc_current_rx2 = (2.5 - ad_2_data_lpf_crnt * 5. / 8000.) / Gth / 3 * dc_current_rx2_weight;;

	dc_current_rx1_lpf = lpf_A_dsp * dc_current_rx1 + lpf_A_dsp * dc_current_rx1_prvs + lpf_B_dsp * dc_current_rx1_lpf_prvs;
	dc_current_rx2_lpf = lpf_A_dsp * dc_current_rx2 + lpf_A_dsp * dc_current_rx2_prvs + lpf_B_dsp * dc_current_rx2_lpf_prvs;

	dc_current_rx1_prvs = dc_current_rx1;
	dc_current_rx2_prvs = dc_current_rx2;
	dc_current_rx1_lpf_prvs = dc_current_rx1_lpf;
	dc_current_rx2_lpf_prvs = dc_current_rx2_lpf;
}

//----------------------------------------------------------------------------------------
//　可変キャパシタ制御
//----------------------------------------------------------------------------------------

interrupt void pwmc_control(void)
{
	C6657_timer1_clear_eventflag();

	if (pwmc_control_on == 1) {

		power_total_updated = power_total_avg;

		if (state == INCREMENT_RX1) {
			pwm_out_rx1 += DELTA;
			state = DECREMENT_RX1;
		}
		else if (state == DECREMENT_RX1) {
			power_total_increment_rx1 = power_total_updated;
			pwm_out_rx1 -= 2*DELTA;
			state = INCREMENT_RX2;
		}
		else if (state == INCREMENT_RX2) {
			power_total_decrement_rx1 = power_total_updated;
			pwm_out_rx1 += DELTA;
			pwm_out_rx2 += DELTA;
			state = DECREMENT_RX2;
		}
		else if (state == DECREMENT_RX2) {
			power_total_increment_rx2 = power_total_updated;
			pwm_out_rx2 -= 2*DELTA;
			state = UPDATE_RX1_RX2;
		}
		else if (state == UPDATE_RX1_RX2) {
			power_total_decrement_rx2 = power_total_updated;
			pwm_out_rx2 += DELTA;
			gradient_rx1 = (power_total_increment_rx1 - power_total_decrement_rx1) / (2 * DELTA);
			gradient_rx2 = (power_total_increment_rx2 - power_total_decrement_rx2) / (2 * DELTA);
			if (mwabs(gradient_rx1) < epsilon && mwabs(gradient_rx2) < epsilon) {
				state = STOP;
			}
			else {
				pwm_out_rx1 += alpha * gradient_rx1;
				pwm_out_rx2 += alpha * gradient_rx2;
				state = INCREMENT_RX1;
			}
		}

		pwm_out_rx1 = limitter(pwm_out_rx1, 0, 1);
		pwm_out_rx2 = limitter(pwm_out_rx2, 0, 1);
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
	else
	{
		PEV_inverter_set_uvw(BDN_PEV, duty_w_o_control-1, 1-duty_w_o_control, 0, 0);
	}
}

//----------------------------------------------------------------------------------------
//　受電電力の合計を計算
//----------------------------------------------------------------------------------------

interrupt void calculate_total_power(void)
{
	C6657_timer2_clear_eventflag();

	power_rx1 = RL1 * dc_current_rx1_lpf * dc_current_rx1_lpf;
	power_rx2 = RL2 * dc_current_rx2_lpf * dc_current_rx2_lpf;

	power_total = power_rx1 + power_rx2;

	power_total_avg = moving_average(power_log, POWER_LOG_SIZE, power_total);
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	int i;

	for (i = 0; i < POWER_LOG_SIZE; i++) {
		power_log[i] = 0;
	}

	lpf_T_dsp = 1 / (2 * PI * LPF_CUTTOFF_dsp);
	lpf_A_dsp = TIMER0_INTERVAL * 1e-6 / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T_dsp);
	lpf_B_dsp = (2 * lpf_T_dsp - TIMER0_INTERVAL * 1e-6) / (2 * lpf_T_dsp + TIMER0_INTERVAL * 1e-6);

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
	C6657_timer2_init_vector(calculate_total_power, (CSL_IntcVectId)7);
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
		if (start_inv_w_o_control == 1) {
			PEV_inverter_start_pwm(BDN_PEV);
			start_inv_w_o_control = -1;
		}
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
