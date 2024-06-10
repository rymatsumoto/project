/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/
#include <mwio4.h>

#define BDN_FPGA 1 //FPGAボード

#define INV_FREQ 83000
#define DEAD_TIME 500
#define FRAC_WIDTH 16

#define TIMER0_INTERVAL 1000
#define TIMER1_INTERVAL 4000

#define PI 3.14

#define Vdc 40
#define ATTENUATION 1.108

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

float p1_lpf = 0;
volatile float p1_ref = 0;

float error_crnt = 0;
float error_prvs = 0;
float error_integral = 0;
float period = TIMER1_INTERVAL * 1e-6;
float v1_ampl_ref = 0;
float pdm_duty_ref = 0;
volatile float proportionalGain;
volatile float integralGain;

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

	p1_lpf = IPFPGA_read(BDN_FPGA, 0x17) * ATTENUATION * ATTENUATION / 64. / 64.;

	if (control_on == 1) {
		error_crnt = p1_ref - p1_lpf;
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
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{

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