/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/

// サーボパック設定
// 内部設定速度1 845 ... 5kmph
// ソフトスタート加速・減速時間 2000
// モータ最高速度 2000

#include <mwio4.h>

#define BDN0 1 //検知用PEVボード
#define BDN1 2 //給電用PEVボード
#define BDN2 0 //FPGAボード
                            //WN WP VN VP UN UP
#define TRANS_MODE 80       //00 00 01 01 00 00

#define INV_FREQ 85000
#define DEAD_TIME 600
#define TIMER_0_INTERVAL 10
#define TIMER_1_INTERVAL 500
#define TIMER_2_INTERVAL 1000

#define SLEEP 0
#define STANDBY 1
#define DETECT 2
#define PAUSE 3
#define TRANSMIT 4
#define STOP 5

int pause_time_us = 0;
int transmit_time_us = 0;
volatile int pause_time_thld_us = 5e4;
volatile int transmit_time_thld_us = 1e5;

INT32 adc_0_data_peak;
INT32 adc_1_data_peak;
float i1_ampl_50A = 0;
float i1_ampl_200A = 0;
volatile float i1_ampl_max = 50;
volatile float i1_weight = 1.1;

volatile int inverter_mode = 0;

volatile float duty_pulse = 0.1;
volatile float duty_trans = 1;
volatile float duty_step = 0.01;
volatile float duty_ramp = 0;

volatile float i1_ampl_thld_detect = 2;
volatile float i1_ampl_thld_transmit = 10;

volatile int enable_soft_start = 0;
volatile int overcurrent_protection = 0;

volatile int search_pulse_on = 0;

//----------------------------------------------------------------------------------------
//　時間計測
//----------------------------------------------------------------------------------------

interrupt void encoding_time_count(void)
{
	C6657_timer2_clear_eventflag();

	if (inverter_mode == PAUSE)
	{
		pause_time_us += TIMER_2_INTERVAL;
	}
	else if (inverter_mode == TRANSMIT)
	{
		transmit_time_us += TIMER_2_INTERVAL;
	}
}

//----------------------------------------------------------------------------------------
//　ソフトスタート
//----------------------------------------------------------------------------------------

void soft_start(void)
{
	if (inverter_mode == TRANSMIT)
	{
		if (enable_soft_start == 0)
		{
			PEV_inverter_set_uvw(BDN1, duty_trans-1, 1-duty_trans, 0, 0);
		}
		else if (enable_soft_start == 1)
		{
			if (duty_ramp < duty_trans)
			{
				duty_ramp += duty_step;
				PEV_inverter_set_uvw(BDN1, duty_ramp-1, 1-duty_ramp, 0, 0);
			}
			else
			{
				PEV_inverter_set_uvw(BDN1, duty_trans-1, 1-duty_trans, 0, 0);
			}
		}
	}
}

//----------------------------------------------------------------------------------------
//　パルス印加または給電を開始
//----------------------------------------------------------------------------------------

interrupt void turn_on_inverter(void)
{
	C6657_timer1_clear_eventflag();

	if (inverter_mode == STANDBY)
	{
		PEV_inverter_set_uvw(BDN0, duty_pulse-1, 1-duty_pulse, 0, 0);
		PEV_inverter_start_pwm(BDN0);
		inverter_mode = DETECT;
	}
	else if (inverter_mode == DETECT)
	{
		PEV_inverter_stop_pwm(BDN0);
		inverter_mode = PAUSE;
	}
	else if (inverter_mode == PAUSE)
	{
		if (pause_time_us > pause_time_thld_us)
		{
			PEV_inverter_set_uvw(BDN1, -1, 1, 0, 0);
			PEV_inverter_start_pwm(BDN1);
			inverter_mode = TRANSMIT;
		}
	}
}

//----------------------------------------------------------------------------------------
//　パルス印加または給電を終了
//----------------------------------------------------------------------------------------

interrupt void turn_off_inverter(void)
{
	C6657_timer0_clear_eventflag();

	if (inverter_mode == DETECT)
	{
		if (i1_ampl_50A > i1_ampl_thld_detect)
		{
			PEV_inverter_stop_pwm(BDN0);
			inverter_mode = STANDBY;
		}
	}
	else if (inverter_mode == TRANSMIT)
	{
		// if (i1_ampl_200A > i1_ampl_thld_transmit && transmit_time_us > transmit_time_thld_us)
		if (transmit_time_us > transmit_time_thld_us)
		{
			PEV_inverter_stop_pwm(BDN1);
			inverter_mode = STOP;
		}
	}
}

//----------------------------------------------------------------------------------------
//　電流包絡線検波
//----------------------------------------------------------------------------------------

void read_envelope(void)
{
    adc_0_data_peak = IPFPGA_read(BDN2, 0x17);
	adc_1_data_peak = IPFPGA_read(BDN2, 0x18);
	i1_ampl_50A = adc_0_data_peak * 125. / 8000 * i1_weight;
	i1_ampl_200A = adc_1_data_peak * 500. / 8000 * i1_weight;

	if (overcurrent_protection == 1)
	{
		if (i1_ampl_200A > i1_ampl_max)
		{
			PEV_inverter_stop_pwm(BDN0);
			PEV_inverter_stop_pwm(BDN1);
			inverter_mode = STOP;
		}
	}
}

//----------------------------------------------------------------------------------------
//　キャリア同期割込み
//----------------------------------------------------------------------------------------

interrupt void sync_interrupt(void)
{
	int0_ack();
	read_envelope();
	soft_start();
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	int_disable();

	PEV_init(BDN0);
    PEV_inverter_disable_int(BDN0);
    PEV_inverter_init(BDN0, INV_FREQ, DEAD_TIME);
    PEV_inverter_init_int_timing(BDN0, 1, 0, 0);
    PEV_int_init(BDN0, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(sync_interrupt, (CSL_IntcVectId)8, FALSE);
    PEV_inverter_control_gate(BDN0, TRANS_MODE);
    PEV_inverter_set_uvw(BDN0, -1, 1, 0, 0);
    PEV_inverter_enable_int(BDN0);
    int0_enable_int();

	PEV_init(BDN1);
	PEV_inverter_init(BDN1, INV_FREQ, DEAD_TIME);
	PEV_inverter_control_gate(BDN1, TRANS_MODE);
	PEV_inverter_set_uvw(BDN1, -1, 1, 0, 0);

	C6657_timer0_init(TIMER_0_INTERVAL);
	C6657_timer0_init_vector(turn_off_inverter, (CSL_IntcVectId)7);
	C6657_timer0_start();
	C6657_timer0_enable_int();

	C6657_timer1_init(TIMER_1_INTERVAL);
	C6657_timer1_init_vector(turn_on_inverter, (CSL_IntcVectId)6);
	C6657_timer1_start();
	C6657_timer1_enable_int();

	C6657_timer2_init(TIMER_2_INTERVAL);
	C6657_timer2_init_vector(encoding_time_count, (CSL_IntcVectId)5);
	C6657_timer2_start();
	C6657_timer2_enable_int();

	int_enable();
}

//----------------------------------------------------------------------------------------
//　メイン
//----------------------------------------------------------------------------------------

int MW_main(void)
{
	initialize();

	while (1)
	{
		if (search_pulse_on == 1)
		{
			inverter_mode = STANDBY;
			search_pulse_on = -1;
		}
	}
}