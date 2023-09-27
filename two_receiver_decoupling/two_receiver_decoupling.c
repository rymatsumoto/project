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
#define TIMER0_INTERVAL 5
#define TIMER1_INTERVAL 10000
#define TIMER2_INTERVAL 1000
#define RL1 10
#define RL2 10

volatile int REF_PWMC_RX1 = 160;
volatile int REF_PWMC_RX2 = 160;
volatile int LPF_CUTTOFF = 1000;
volatile float P_O_STEP_SIZE = 0.005;

volatile int start_current_control = 0;
volatile int start_pwmc_rx1_init = 0;
volatile int start_pwmc_rx2_init = 0;
volatile int start_pwmc_control = 0;
int current_control_on = 0;
int pwmc_control_on = 0;

INT32 adc_0_data_peak;
float itx_ampl = 0;
volatile float itx_weight = 1.1;
volatile float itx_ampl_ref;

float error_crnt = 0;
float error_prvs = 0;
float error_integral = 0;
float vtx_ampl_ref = 0;
float inv_mod = 0;
float period = 1 / (float)INV_FREQ;
const float pi = 3.14;
volatile float vdc;
volatile float proportionalGain;
volatile float integralGain;

int state = 0;
volatile float pwm_out_rx1 = 0;
volatile float pwm_out_rx2 = 0;
volatile int wref_rx1 = 0;
volatile int wref_rx2 = 0;
volatile int carrier_cnt_max;
volatile int carrier_cnt_hlf;

float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float data[] = {0., 0., 0., 0.};
float dc_current_rx1;
float dc_current_rx2;
float dc_current_rx1_prvs = 0;
float dc_current_rx2_prvs = 0;
float dc_current_rx1_lpf;
float dc_current_rx2_lpf;
float dc_current_rx1_lpf_prvs = 0;
float dc_current_rx2_lpf_prvs = 0;
float lpf_T;
float lpf_A;
float lpf_B;
const float Gth = 0.1042;

// volatile int count_start = 0;
// int counter = -1;

float power_rx1 = 0;
float power_rx2 = 0;
float max_power = 0;
float power_state_1 = 0;
float power_state_2 = 0;
float power_state_3 = 0;
float power_state_4 = 0;

//----------------------------------------------------------------------------------------
//　max関数
//----------------------------------------------------------------------------------------

float max(float a, float b)
{
    if (a >= b) {
        return a;
    }
    else if (a < b) {
        return b;
    }
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
//　出力電流読み出し
//----------------------------------------------------------------------------------------

interrupt void read_dc_current(void)
{
	C6657_timer0_clear_eventflag();

	PEV_ad_in_4ch(BDN_PEV, 0, data);
	dc_current_rx1 = (2.5 - data[0]) / Gth / 3;
	dc_current_rx2 = (2.5 - data[1]) / Gth / 3;

	lpf_T = 1 / (2 * 3.14 * LPF_CUTTOFF);
	lpf_A = TIMER0_INTERVAL * 1e-6 / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T);
	lpf_B = (TIMER0_INTERVAL * 1e-6 - 2 * lpf_T) / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T);

	dc_current_rx1_lpf = lpf_A * dc_current_rx1 + lpf_A * dc_current_rx1_prvs - lpf_B * dc_current_rx1_lpf_prvs;
	dc_current_rx2_lpf = lpf_A * dc_current_rx2 + lpf_A * dc_current_rx2_prvs - lpf_B * dc_current_rx2_lpf_prvs;

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
		if (state == 0) {
			pwm_out_rx1 = pwm_out_rx1 + P_O_STEP_SIZE;
			pwm_out_rx2 = pwm_out_rx2 + P_O_STEP_SIZE;
			state = 1;
		}
		else if (state == 1) {
			power_rx1 = RL1 * dc_current_rx1_lpf * dc_current_rx1_lpf;
			power_rx2 = RL2 * dc_current_rx2_lpf * dc_current_rx2_lpf;
			power_state_1 = power_rx1 + power_rx2;
			pwm_out_rx2 = pwm_out_rx2 - P_O_STEP_SIZE * 2;
			state = 2;
		}
		else if (state == 2) {
			power_rx1 = RL1 * dc_current_rx1_lpf * dc_current_rx1_lpf;
			power_rx2 = RL2 * dc_current_rx2_lpf * dc_current_rx2_lpf;
			power_state_2 = power_rx1 + power_rx2;
			pwm_out_rx1 = pwm_out_rx1 - P_O_STEP_SIZE * 2;
			state = 3;
		}
		else if (state == 3) {
			power_rx1 = RL1 * dc_current_rx1_lpf * dc_current_rx1_lpf;
			power_rx2 = RL2 * dc_current_rx2_lpf * dc_current_rx2_lpf;
			power_state_3 = power_rx1 + power_rx2;
			pwm_out_rx2 = pwm_out_rx2 + P_O_STEP_SIZE * 2;
			state = 4;
		}
		else if (state == 4) {
			power_rx1 = RL1 * dc_current_rx1_lpf * dc_current_rx1_lpf;
			power_rx2 = RL2 * dc_current_rx2_lpf * dc_current_rx2_lpf;
			power_state_4 = power_rx1 + power_rx2;
			pwm_out_rx1 = pwm_out_rx1 + P_O_STEP_SIZE;
			pwm_out_rx2 = pwm_out_rx2 - P_O_STEP_SIZE;

			max_power = max(max(power_state_1, power_state_2), max(power_state_3, power_state_4));

			if (max_power == power_state_1) {
				pwm_out_rx1 = pwm_out_rx1 + P_O_STEP_SIZE;
				pwm_out_rx2 = pwm_out_rx2 + P_O_STEP_SIZE;
			}
			else if (max_power == power_state_2) {
				pwm_out_rx1 = pwm_out_rx1 + P_O_STEP_SIZE;
				pwm_out_rx2 = pwm_out_rx2 - P_O_STEP_SIZE;
			}
			else if (max_power == power_state_3) {
				pwm_out_rx1 = pwm_out_rx1 - P_O_STEP_SIZE;
				pwm_out_rx2 = pwm_out_rx2 - P_O_STEP_SIZE;
			}
			else if (max_power == power_state_4) {
				pwm_out_rx1 = pwm_out_rx1 - P_O_STEP_SIZE;
				pwm_out_rx2 = pwm_out_rx2 + P_O_STEP_SIZE;
			}
			state = 0;
		}
		wref_rx1 = pwm_out_rx1 * carrier_cnt_max;
		wref_rx2 = pwm_out_rx2 * carrier_cnt_max;

		IPFPGA_write(BDN_FPGA1, 0x04, wref_rx1);
		IPFPGA_write(BDN_FPGA2, 0x04, wref_rx2);
	}

	else {
		wref_rx1 = pwm_out_rx1 * carrier_cnt_max;
		wref_rx2 = pwm_out_rx2 * carrier_cnt_max;

		IPFPGA_write(BDN_FPGA1, 0x04, wref_rx1);
		IPFPGA_write(BDN_FPGA2, 0x04, wref_rx2);
	}
}

//----------------------------------------------------------------------------------------
//　電流制御
//----------------------------------------------------------------------------------------

interrupt void current_control(void)
{
	int0_ack();

    adc_0_data_peak = IPFPGA_read(BDN_FPGA1, 0x17);
	itx_ampl = adc_0_data_peak * 125. / 8000 * itx_weight;

    if (current_control_on == 1)
    {
        error_crnt = itx_ampl_ref - itx_ampl;
        error_integral = error_integral + (error_crnt + error_prvs) / 2 * period;
        vtx_ampl_ref = proportionalGain * error_crnt + integralGain * error_integral;
        error_prvs = error_crnt;

        if (vtx_ampl_ref < 0)
        {
            vtx_ampl_ref = 0;
        }
        else if (vtx_ampl_ref > vdc * 4 / pi)
        {
            vtx_ampl_ref = vdc * 4 / pi;
        }

        inv_mod = 2 / pi * arccos(pi * vtx_ampl_ref / (4 * vdc));

        if (inv_mod < 0)
        {
            inv_mod = 0;
        }
        else if (inv_mod > 1)
        {
            inv_mod = 1;
        }

        PEV_inverter_set_uvw(BDN_PEV, -inv_mod, inv_mod, 0, 0);
    }
}

//----------------------------------------------------------------------------------------
// カウンター
//----------------------------------------------------------------------------------------

// interrupt void increment_counter(void)
// {
// 	C6657_timer2_clear_eventflag();

// 	if (count_start == 1){
// 		counter = counter + 1;
// 	}
// }

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
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
    PEV_inverter_init_int_timing(BDN_PEV, 1, 3, 0);
    PEV_int_init(BDN_PEV, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(current_control, (CSL_IntcVectId)8, FALSE);
    PEV_inverter_control_gate(BDN_PEV, TRANS_MODE);
    PEV_inverter_set_uvw(BDN_PEV, 0, 0, 0, 0);

	C6657_timer0_init(TIMER0_INTERVAL);
	C6657_timer0_init_vector(read_dc_current, (CSL_IntcVectId)5);
	C6657_timer0_start();

	C6657_timer1_init(TIMER1_INTERVAL);
	C6657_timer1_init_vector(pwmc_control, (CSL_IntcVectId)6);
	C6657_timer1_start();

	// C6657_timer2_init(TIMER2_INTERVAL);
	// C6657_timer2_init_vector(increment_counter, (CSL_IntcVectId)7);
	// C6657_timer2_start();

    PEV_inverter_enable_int(BDN_PEV);
    int0_enable_int();
    C6657_timer0_enable_int();
	C6657_timer1_enable_int();
	// C6657_timer2_enable_int();
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
