/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include <mwio4.h>

#define BDN_PEV  0 //PEVボード
#define BDN_FPGA 0 //FPGAボード

#define INV_FREQ 83000
#define DEAD_TIME 500
#define FRAC_WIDTH 16

#define TIMER0_INTERVAL 1000
#define TIMER1_INTERVAL 10
#define TIMER2_INTERVAL 10

#define PI 3.14

#define Vdc 40
#define ATTENUATION 1.1

int peak_count_i1 = 230;

float tau;
float lpf_a;
float lpf_b;
volatile int lpf_cuttoff = 250;
float Ts_lpf = 1. / INV_FREQ / 2;

int v1d_scale = Vdc * 4. / PI * 64 / ATTENUATION;

volatile int set_pwm_on_w_control = 0;
volatile int set_pwm_on_w_o_control = 0;
int control_on = 0;
volatile int pdm_ref = 0;
volatile int ref_full_scale = 1000;

int carrier_cnt_max;

float Plpf = 0;
volatile float Pref = 0;

float error_crnt = 0;
float error_prvs = 0;
float error_integral = 0;
float period = TIMER1_INTERVAL * 1e-6;
float v1_ampl_ref = 0;
float pdm_duty_ref = 0;
volatile float proportionalGain;
volatile float integralGain;
volatile int Pref_update = 0;
int counter = 0;
volatile int update_Pref_count = 1000;
volatile int update_start = 0;

const float Gth = 0.1042;
float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float data[] = {0., 0., 0., 0.};
float tx_dc_current = 0;
float rx_dc_current = 0;
float input_dc_power = 0;
float output_dc_power = 0;

//----------------------------------------------------------------------------------------
//　FPGAに書き込む定数の計算
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
//　LPF時定数を更新
//----------------------------------------------------------------------------------------

interrupt void update_lpf(void)
{
	C6657_timer0_clear_eventflag();

	tau = 1. / (2 * PI * lpf_cuttoff);
	lpf_a = Ts_lpf / (Ts_lpf + 2 * tau);
	lpf_b = (2 * tau - Ts_lpf) / (2 * tau + Ts_lpf);

	IPFPGA_write(BDN_FPGA, 0x16, convert_binary(lpf_a));
	IPFPGA_write(BDN_FPGA, 0x17, convert_binary(lpf_b));	
}

//----------------------------------------------------------------------------------------
//　PDMデューティを更新
//----------------------------------------------------------------------------------------

interrupt void update_pdm(void)
{
	C6657_timer1_clear_eventflag();

	Plpf = IPFPGA_read(BDN_FPGA, 0x17) * ATTENUATION * ATTENUATION / 64. / 64.;

	if (control_on == 1) {
		if (Pref_update != 0)
		{
			counter += 1;
			if (counter >= update_Pref_count)
			{
				Pref = Pref_update;
			}
		}
		// if (update_start == 1) {
		// 	Pref = Pref_update;
		// }
		error_crnt = Pref - Plpf;
		error_integral = error_integral + (error_crnt + error_prvs) / 2 * period;
        v1_ampl_ref = proportionalGain * error_crnt + integralGain * error_integral;
        error_prvs = error_crnt;

		pdm_duty_ref = v1_ampl_ref / (Vdc * 4. / PI);

        if (pdm_duty_ref < 0)
        {
            pdm_duty_ref = 0;
        }
        else if (pdm_duty_ref > 1)
        {
            pdm_duty_ref = 1;
        }

		pdm_ref = pdm_duty_ref * ref_full_scale;
		IPFPGA_write(BDN_FPGA, 0x19, pdm_ref);
	}
	else {
		IPFPGA_write(BDN_FPGA, 0x19, pdm_ref);
	}
}

//----------------------------------------------------------------------------------------
//　DC電流読み出し
//----------------------------------------------------------------------------------------

interrupt void read_dc_current(void)
{
	C6657_timer2_clear_eventflag();

    PEV_ad_in_4ch(BDN_PEV, 0, data);
    tx_dc_current = data[0] * 25;
    rx_dc_current = data[1] * 25;

    input_dc_power = tx_dc_current * Vdc;
    output_dc_power = rx_dc_current * Vdc;
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
    PEV_ad_set_range(BDN_PEV, range);
    PEV_ad_set_mode(BDN_PEV, 0);

	tau = 1. / (2 * PI * lpf_cuttoff);
	lpf_a = Ts_lpf / (Ts_lpf + 2 * tau);
	lpf_b = (2 * tau - Ts_lpf) / (2 * tau + Ts_lpf);

	carrier_cnt_max = 50000000. / INV_FREQ;
	
	IPFPGA_write(BDN_FPGA, 0x01, carrier_cnt_max);
	IPFPGA_write(BDN_FPGA, 0x05, DEAD_TIME/10);
	IPFPGA_write(BDN_FPGA, 0x11, peak_count_i1);
	IPFPGA_write(BDN_FPGA, 0x16, convert_binary(lpf_a));
	IPFPGA_write(BDN_FPGA, 0x17, convert_binary(lpf_b));
	IPFPGA_write(BDN_FPGA, 0x18, v1d_scale);
	IPFPGA_write(BDN_FPGA, 0x19, pdm_ref);
	IPFPGA_write(BDN_FPGA, 0x20, ref_full_scale);

	int_disable();

	C6657_timer0_init(TIMER0_INTERVAL);
	C6657_timer0_init_vector(update_lpf, (CSL_IntcVectId)6);
	C6657_timer0_start();
	C6657_timer0_enable_int();

	C6657_timer1_init(TIMER1_INTERVAL);
	C6657_timer1_init_vector(update_pdm, (CSL_IntcVectId)7);
	C6657_timer1_start();
	C6657_timer1_enable_int();

	C6657_timer2_init(TIMER2_INTERVAL);
	C6657_timer2_init_vector(read_dc_current, (CSL_IntcVectId)8);
	C6657_timer2_start();
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
		if (set_pwm_on_w_o_control == 1)
		{
			IPFPGA_write(BDN_FPGA, 0x06, 1);
			control_on = 0;
			set_pwm_on_w_o_control = -1;
		}
		else if (set_pwm_on_w_control == 1)
		{
			IPFPGA_write(BDN_FPGA, 0x06, 1);
			control_on = 1;
			set_pwm_on_w_control = -1;
		}
	}
}