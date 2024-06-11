/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/

#include <mwio4.h>

#define BDN_FPGA 1

#define INV_FREQ 83000
#define DEAD_TIME 500
#define FRAC_WIDTH 16

#define TIMER0_INTERVAL 10

#define PI 3.14

int carrier_cnt_max;
int carrier_cnt_hlf;

int peak_count_i1 = 230;
int peak_count_i2 = 530;

#define Vdc 40
#define L1 30e-6
#define L2 30e-6
#define M 6.4e-6
#define C1 122e-9
#define C2 122e-9
#define R1 50e-3
#define R2 50e-3

#define ATTENUATION 1.108

float wr = INV_FREQ * PI * 2;
float Ts_mpc = 1. / INV_FREQ / 2;
float Ts_lpf = 1. / INV_FREQ / 2;

float mpc_a;
float mpc_b;
float mpc_c;
float tau;
float lpf_a;
float lpf_b;
int v1d_scale = Vdc * 4. / PI * 64 / ATTENUATION;

volatile int trans_start = 0;

float p1_lpf = 0;
volatile int Pref = 50;
volatile int lpf_cuttoff = 10e3;
volatile int Pref_update = 0;

int counter = 0;
volatile int update_Pref_count = 1000;

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
//　FPGA設定変更
//----------------------------------------------------------------------------------------

interrupt void fpga_config(void)
{
	C6657_timer0_clear_eventflag();

	p1_lpf = IPFPGA_read(BDN_FPGA, 0x17) * ATTENUATION * ATTENUATION / 64. / 64.;

	tau = 1. / (2 * PI * lpf_cuttoff);
	lpf_a = Ts_lpf / (Ts_lpf + 2 * tau);
	lpf_b = (2 * tau - Ts_lpf) / (2 * tau + Ts_lpf);

	IPFPGA_write(BDN_FPGA, 0x16, convert_binary(lpf_a));
	IPFPGA_write(BDN_FPGA, 0x17, convert_binary(lpf_b));

	if (Pref_update == 0)
	{
		IPFPGA_write(BDN_FPGA, 0x19, Pref * 64 * 64 / ATTENUATION / ATTENUATION);
	}
	else
	{
		if (trans_start == -1)
		{
			counter += 1;
			if (counter < update_Pref_count)
			{
				IPFPGA_write(BDN_FPGA, 0x19, Pref * 64 * 64 / ATTENUATION / ATTENUATION);
			}
			if (counter >= update_Pref_count)
			{
				IPFPGA_write(BDN_FPGA, 0x19, Pref_update * 64 * 64 / ATTENUATION / ATTENUATION);
			}
		}
	}
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	mpc_a = 1 - R1 * Ts_mpc / (2 * L1);
	mpc_b = wr * M * Ts_mpc / (2 * L1);
	mpc_c = Ts_mpc / (2 * L1);

	tau = 1. / (2 * PI * lpf_cuttoff);
	lpf_a = Ts_lpf / (Ts_lpf + 2 * tau);
	lpf_b = (2 * tau - Ts_lpf) / (2 * tau + Ts_lpf);

	carrier_cnt_max = 50000000. / INV_FREQ;
	carrier_cnt_hlf = 25000000. / INV_FREQ;

	IPFPGA_write(BDN_FPGA, 0x01, carrier_cnt_max);
	IPFPGA_write(BDN_FPGA, 0x02, carrier_cnt_hlf);
	IPFPGA_write(BDN_FPGA, 0x03, carrier_cnt_hlf);
	IPFPGA_write(BDN_FPGA, 0x05, DEAD_TIME / 10);
	IPFPGA_write(BDN_FPGA, 0x11, peak_count_i1);
	IPFPGA_write(BDN_FPGA, 0x12, peak_count_i2);
	IPFPGA_write(BDN_FPGA, 0x13, convert_binary(mpc_a));
	IPFPGA_write(BDN_FPGA, 0x14, convert_binary(mpc_b));
	IPFPGA_write(BDN_FPGA, 0x15, convert_binary(mpc_c));
	IPFPGA_write(BDN_FPGA, 0x16, convert_binary(lpf_a));
	IPFPGA_write(BDN_FPGA, 0x17, convert_binary(lpf_b));
	IPFPGA_write(BDN_FPGA, 0x18, v1d_scale);
	IPFPGA_write(BDN_FPGA, 0x19, Pref * 64 * 64 / ATTENUATION / ATTENUATION);

	int_disable();

	C6657_timer0_init(TIMER0_INTERVAL);
	C6657_timer0_init_vector(fpga_config, (CSL_IntcVectId)5);
	C6657_timer0_start();
	C6657_timer0_enable_int();

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
		if (trans_start == 1) {
			IPFPGA_write(BDN_FPGA, 0x06, 1);
			trans_start = -1;
		}
	}
}
