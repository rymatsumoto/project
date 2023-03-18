/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/

// サーボパック設定
// 内部設定速度1 845 : 5kmph 
// 内部設定速度1 50 : 82.25 mmps
// ソフトスタート加速・減速時間 2000
// モータ最高速度 2000

// 5 kmphのときは900mmからスタート
// 82.25 mmpsのときは1200mmからスタート
// 以下のパラメータはV1dc=V2dc=20Vのとき

#include <mwio4.h>

#define BDN_PEV_TX 1 //送電側PEVボード
#define BDN_PEV_RX 0 //受電側PEVボード
#define BDN_FPGA 0   //FPGAボード
                            //WN WP VN VP UN UP
#define TRANS_MODE 80       //00 00 01 01 00 00
#define RECT_MODE  255      //00 00 11 11 11 11
#define SHORT_MODE 187      //00 00 10 11 10 11

#define OFF 0
#define CN1_40 0
#define CN1_41 1
#define CN1_45 2
#define CN1_46 3

#define INV_FREQ 85000
#define DEAD_TIME 300

#define TIMER_0_INTERVAL 100
#define TIMER_1_INTERVAL 5
#define TIMER_2_INTERVAL 500

#define SLEEP 0
#define STANDBY 1
#define DETECT 2
#define PAUSE 3
#define TRANSMIT 4
#define STOP 5

int run_time_us = 0;

float time_fwd_start_us = 0;
float time_wait_us = 3e6;
float time_fwd_decel_start_us;
float time_fwd_end_us;
float time_bwd_start_us;
float time_bwd_decel_start_us;
float time_bwd_end_us;
int pause_time_us = 0;
int transmit_time_us = 0;

// volatile int pause_time_thld_us = 5e4; // 5 kmph
// volatile int transmit_time_thld_us = 1e5; // 5kmph
// volatile int inverter_start_count = 90000; // 5 kmph

volatile int pause_time_thld_us = 0; // 82.25 mmps
volatile int transmit_time_thld_us = 1e6; // 82.25 mmps
volatile int inverter_start_count = 5e4; // 82.25 mmps

int bench_mode = 0;
volatile int servo_on = 0;
int encoder_count = 0;

INT32 adc_0_data_peak;
INT32 adc_1_data_peak;
float i1_ampl = 0;
float i2_ampl = 0;
volatile float i1_ampl_max = 50;
volatile float i2_ampl_max = 50;
volatile float i1_weight = 1.1;
volatile float i2_weight = 1.1;

volatile int inverter_mode = 0;

volatile float duty_pulse = 0.1;
volatile float duty_trans = 1;
volatile float duty_step = 0.01;
volatile float duty_ramp = 0;

volatile float i1_ampl_thld_detect = 1.5;
volatile float i1_ampl_thld_transmit = 10;

volatile int enable_soft_start = 0;
volatile int overcurrent_protection = 0;

//----------------------------------------------------------------------------------------
//　エンコーダ読み出し & 走行開始からの時間計測（timer0）
//----------------------------------------------------------------------------------------

interrupt void count_run_time(void)
{
	C6657_timer0_clear_eventflag();

	encoder_count = PEV_abz_read(BDN_PEV_TX);

	if (servo_on == 1)
	{
		run_time_us += TIMER_0_INTERVAL;
	}
}

//----------------------------------------------------------------------------------------
//　路面側のPAUSE時間，路面側の給電時間を計測（timer1）
//----------------------------------------------------------------------------------------

void count_trans_time(void)
{
	if (inverter_mode == PAUSE)
	{
		pause_time_us += TIMER_1_INTERVAL;
	}
	else if (inverter_mode == TRANSMIT)
	{
		transmit_time_us += TIMER_1_INTERVAL;
	}
}

//----------------------------------------------------------------------------------------
//　パルス印加または給電を終了（timer1）
//----------------------------------------------------------------------------------------

void turn_off_inverter(void)
{
	if (inverter_mode == DETECT)
	{
		if (i1_ampl > i1_ampl_thld_detect)
		{
			PEV_inverter_stop_pwm(BDN_PEV_TX);
			inverter_mode = STANDBY;
		}
	}
	else if (inverter_mode == TRANSMIT)
	{
		if (i1_ampl > i1_ampl_thld_transmit && transmit_time_us > transmit_time_thld_us)
		{
			PEV_inverter_stop_pwm(BDN_PEV_TX);
			inverter_mode = STOP;
		}
	}
}

//----------------------------------------------------------------------------------------
//　timer1の割込み関数
//----------------------------------------------------------------------------------------

interrupt void timer_1_int_function(void)
{
	C6657_timer1_clear_eventflag();

	turn_off_inverter();
	count_trans_time();
}

//----------------------------------------------------------------------------------------
//　パルス印加または給電を開始（timer2）
//----------------------------------------------------------------------------------------

interrupt void turn_on_inverter(void)
{
	C6657_timer2_clear_eventflag();

	if (inverter_mode == STANDBY)
	{
		PEV_inverter_set_uvw(BDN_PEV_TX, duty_pulse-1, 1-duty_pulse, 0, 0);
		PEV_inverter_start_pwm(BDN_PEV_TX);
		inverter_mode = DETECT;
	}
	else if (inverter_mode == DETECT)
	{
		PEV_inverter_stop_pwm(BDN_PEV_TX);
		inverter_mode = PAUSE;
	}
	else if (inverter_mode == PAUSE)
	{
		if (pause_time_us > pause_time_thld_us)
		{
			PEV_inverter_set_uvw(BDN_PEV_TX, -1, 1, 0, 0);
			PEV_inverter_control_gate(BDN_PEV_RX, RECT_MODE);
			PEV_inverter_start_pwm(BDN_PEV_TX);
			PEV_inverter_start_pwm(BDN_PEV_RX);
			inverter_mode = TRANSMIT;
		}
	}
}

//----------------------------------------------------------------------------------------
//　ソフトスタート（キャリア同期割込み）
//----------------------------------------------------------------------------------------

void soft_start(void)
{
	if (inverter_mode == TRANSMIT)
	{
		if (enable_soft_start == 0)
		{
			PEV_inverter_set_uvw(BDN_PEV_TX, duty_trans-1, 1-duty_trans, 0, 0);
		}
		else if (enable_soft_start == 1)
		{
			if (duty_ramp < duty_trans)
			{
				duty_ramp += duty_step;
				PEV_inverter_set_uvw(BDN_PEV_TX, duty_ramp-1, 1-duty_ramp, 0, 0);
			}
			else
			{
				PEV_inverter_set_uvw(BDN_PEV_TX, duty_trans-1, 1-duty_trans, 0, 0);
			}
		}
	}
}

//----------------------------------------------------------------------------------------
//　電流包絡線検波（キャリア同期割込み）
//----------------------------------------------------------------------------------------

void read_envelope(void)
{
    adc_0_data_peak = IPFPGA_read(BDN_FPGA, 0x17);
	adc_1_data_peak = IPFPGA_read(BDN_FPGA, 0x18);

	i1_ampl = adc_0_data_peak * 125. / 8000 * i1_weight;
	i2_ampl = adc_1_data_peak * 125. / 8000 * i2_weight;

	if (overcurrent_protection == 1)
	{
		if (i1_ampl > i1_ampl_max || i2_ampl > i2_ampl_max)
		{
			PEV_inverter_stop_pwm(BDN_PEV_TX);
			PEV_inverter_stop_pwm(BDN_PEV_RX);
			inverter_mode = STOP;
		}
	}
}

//----------------------------------------------------------------------------------------
//　キャリア同期割込み関数
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

	PEV_init(BDN_PEV_TX);
    PEV_inverter_disable_int(BDN_PEV_TX);
    PEV_inverter_init(BDN_PEV_TX, INV_FREQ, DEAD_TIME);
    PEV_inverter_init_int_timing(BDN_PEV_TX, 1, 0, 0);
    PEV_int_init(BDN_PEV_TX, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(sync_interrupt, (CSL_IntcVectId)8, FALSE);
    PEV_inverter_control_gate(BDN_PEV_TX, TRANS_MODE);
    PEV_inverter_set_uvw(BDN_PEV_TX, 0, 0, 0, 0);
    PEV_inverter_enable_int(BDN_PEV_TX);
    int0_enable_int();

	PEV_init(BDN_PEV_RX);
	PEV_inverter_init(BDN_PEV_RX, INV_FREQ, DEAD_TIME);
	PEV_inverter_control_gate(BDN_PEV_RX, SHORT_MODE);

	C6657_timer0_init(TIMER_0_INTERVAL);
	C6657_timer0_init_vector(count_run_time, (CSL_IntcVectId)7);
	C6657_timer0_start();
	C6657_timer0_enable_int();

	C6657_timer1_init(TIMER_1_INTERVAL);
	C6657_timer1_init_vector(timer_1_int_function, (CSL_IntcVectId)6);
	C6657_timer1_start();
	C6657_timer1_enable_int();

	C6657_timer2_init(TIMER_2_INTERVAL);
	C6657_timer2_init_vector(turn_on_inverter, (CSL_IntcVectId)5);
	C6657_timer2_start();
	C6657_timer2_enable_int();

	int_enable();

	PEV_abz_init_maxcount(BDN_PEV_TX, 100100100);
	PEV_abz_set_mode(BDN_PEV_TX, 0, 1);

	PEV_pio_out(BDN_PEV_TX, OFF);
	PEV_pio_set_bit(BDN_PEV_TX, CN1_40);

	PEV_inverter_start_pwm(BDN_PEV_RX);

}

//----------------------------------------------------------------------------------------
//　メイン
//----------------------------------------------------------------------------------------

int MW_main(void)
{
	initialize();

	// 5 kmph
	// time_fwd_decel_start_us = time_fwd_start_us + 18e5;
	// time_fwd_end_us = time_fwd_decel_start_us + 16e4;
	// time_bwd_start_us = time_fwd_end_us + time_wait_us;
	// time_bwd_decel_start_us = time_bwd_start_us + 18e5;
	// time_bwd_end_us = time_bwd_decel_start_us + 16e4;

	// 82.25 mmps
	time_fwd_decel_start_us = time_fwd_start_us + 22e6;
	time_fwd_end_us = time_fwd_decel_start_us + 1e6;
	time_bwd_start_us = time_fwd_end_us + time_wait_us;
	time_bwd_decel_start_us = time_bwd_start_us + 22e6;
	time_bwd_end_us = time_bwd_decel_start_us + 1e6;

	while (1)
	{
		if (servo_on == 1)
		{
			if (run_time_us <= time_fwd_start_us) //往路の走行開始前
			{
				PEV_abz_clear(BDN_PEV_TX);
				bench_mode = 1;
			}
			else if (run_time_us < time_fwd_decel_start_us) //往路の加速・定速区間
			{
				PEV_pio_clr_bit(BDN_PEV_TX, CN1_41);
				PEV_pio_set_bit(BDN_PEV_TX, CN1_46);
				bench_mode = 2;
			}
			else if (run_time_us < time_fwd_end_us + time_wait_us / 2) //往路の減速区間
			{
				PEV_pio_clr_bit(BDN_PEV_TX, CN1_41);
				PEV_pio_clr_bit(BDN_PEV_TX, CN1_46);
				bench_mode = 3;
			}
			else if (run_time_us < time_bwd_start_us) //往路の走行開始前
			{
				PEV_pio_clr_bit(BDN_PEV_TX, CN1_41);
				PEV_pio_clr_bit(BDN_PEV_TX, CN1_46);
				bench_mode = 4;
			}
			else if (run_time_us < time_bwd_decel_start_us) //往路の加速・定速区間
			{
				PEV_pio_set_bit(BDN_PEV_TX, CN1_41);
				PEV_pio_set_bit(BDN_PEV_TX, CN1_46);
				bench_mode = 5;
			}
			else if (run_time_us < time_bwd_end_us + time_wait_us / 2) //往路の減速区間
			{
				PEV_pio_set_bit(BDN_PEV_TX, CN1_41);
				PEV_pio_clr_bit(BDN_PEV_TX, CN1_46);
				bench_mode = 6;
			}
			else //動作終了
			{
				PEV_pio_out(BDN_PEV_TX, OFF);
				bench_mode = 7;
			}
		}

		if (encoder_count > inverter_start_count && inverter_mode == SLEEP)
		{
			inverter_mode = STANDBY;
		}
	}
}