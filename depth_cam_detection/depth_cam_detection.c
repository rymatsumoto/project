/*********************************************************
 Title:		Template source file for c6657
 Copyright:	1999-2013  Myway Plus Co.,Ltd.

 Abstract:
	This C source code is the template program
	that is used when you generate a project newly.

 Model:	Expert4(c6657)
*********************************************************/

// サーボパック設定
// 内部設定速度1 50
// ソフトスタート加速・減速時間 2000
// モータ最高速度 2000

#include <mwio4.h>

#define BDN0 1 //送電側PEVボード
#define BDN1 2 //受電側PEVボード
#define BDN2 1 //FPGAボード
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

#define TIMER_0_INTERVAL 1000
#define TIMER_1_INTERVAL 1000
#define TIMER_2_INTERVAL 1000

#define STANDBY 0
#define RUN_START 1
#define TRANSMIT_START 2
#define TRANSMIT_END 3

#define RPI_SIGNAL_HIGH 3.0
#define RPI_SIGNAL_LOW 1.0

int max_velocity_rpm = 847;
float travel_length_m = 2.2;
float const_velocity_length_m = 2.0;

float process_time_us = 0;
float time_fwd_start_us = 0;
float time_wait_us = 2e6;
float time_fwd_decel_start_us;
float time_fwd_end_us;
float time_bwd_start_us;
float time_bwd_decel_start_us;
float time_bwd_end_us;
int transmit_time_us = 0;
volatile int transmit_time_thld_us = 3.6e6;

int bench_mode = 0;
volatile int servo_on = 0;
int encoder_count = 0;
int encoder_previous = 0;
int encoder_diff = 0;

volatile int state = 0;

volatile float inv_mod_BDN0_pulse = 0.9;
volatile float inv_mod_BDN0_trans = 0;

float data[] = {0., 0., 0., 0.};
float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float rpi_signal_crnt = 0;
float rpi_signal_prvs = 0;

volatile int inverter_start_count = 60e3;

//----------------------------------------------------------------------------------------
//　時間計測
//----------------------------------------------------------------------------------------

interrupt void time_count(void)
{
	C6657_timer0_clear_eventflag();

	if (servo_on == 1)
	{
		process_time_us += TIMER_0_INTERVAL;
	}
}

//----------------------------------------------------------------------------------------
//　給電タイミング制御
//----------------------------------------------------------------------------------------

interrupt void state_control(void)
{
	C6657_timer1_clear_eventflag();

	PEV_ad_in_4ch(BDN0, 0, data);
	rpi_signal_crnt = data[0];

	if (state == STANDBY)
	{
		if (rpi_signal_prvs < RPI_SIGNAL_LOW && rpi_signal_crnt > RPI_SIGNAL_HIGH)
		{
			servo_on = 1;
			state = RUN_START;
		}
	}
	else if (state == RUN_START)
	{
		if (rpi_signal_prvs < RPI_SIGNAL_LOW && rpi_signal_crnt > RPI_SIGNAL_HIGH)
		{
			PEV_inverter_set_uvw(BDN0, -inv_mod_BDN0_trans, inv_mod_BDN0_trans, 0, 0);
			PEV_inverter_start_pwm(BDN0);
			state = TRANSMIT_START;
		}
	}
	else if (state == TRANSMIT_START)
	{
		transmit_time_us += TIMER_1_INTERVAL;
		if (transmit_time_us > transmit_time_thld_us)
		{
			PEV_inverter_stop_pwm(BDN0);
			state = TRANSMIT_END;
		}
	}

	rpi_signal_prvs = rpi_signal_crnt;
}

//----------------------------------------------------------------------------------------
//　エンコーダ読み出し
//----------------------------------------------------------------------------------------

interrupt void encoding(void)
{
	C6657_timer2_clear_eventflag();

	encoder_count = PEV_abz_read(BDN0);
	encoder_diff = encoder_count - encoder_previous;

	encoder_previous = encoder_count;

	if (inverter_start_count < encoder_count && encoder_count < inverter_start_count + 0.5e3)
	{
		PEV_inverter_set_uvw(BDN0, -inv_mod_BDN0_pulse, inv_mod_BDN0_pulse, 0, 0);
		PEV_inverter_start_pwm(BDN0);
	}
	else if (inverter_start_count + 0.5e3 < encoder_count && encoder_count < inverter_start_count + 1e3)
	{
		PEV_inverter_stop_pwm(BDN0);
	}
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	PEV_ad_set_range(BDN0, range);
	PEV_ad_set_mode(BDN0, 0);

	int_disable();

	PEV_init(BDN0);
    PEV_inverter_init(BDN0, INV_FREQ, DEAD_TIME);
    PEV_inverter_control_gate(BDN0, TRANS_MODE);
    PEV_inverter_set_uvw(BDN0, 0, 0, 0, 0);

	C6657_timer0_init(TIMER_0_INTERVAL);
	C6657_timer0_init_vector(time_count, (CSL_IntcVectId)5);
	C6657_timer0_start();
	C6657_timer0_enable_int();

	C6657_timer1_init(TIMER_1_INTERVAL);
	C6657_timer1_init_vector(state_control, (CSL_IntcVectId)6);
	C6657_timer1_start();
	C6657_timer1_enable_int();

	C6657_timer2_init(TIMER_2_INTERVAL);
	C6657_timer2_init_vector(encoding, (CSL_IntcVectId)7);
	C6657_timer2_start();
	C6657_timer2_enable_int();

	int_enable();

	PEV_abz_init_maxcount(BDN0, 100100100);
	PEV_abz_set_mode(BDN0, 0, 1);

	PEV_pio_out(BDN0, OFF);
	PEV_pio_set_bit(BDN0, CN1_40);

}

//----------------------------------------------------------------------------------------
//　メイン
//----------------------------------------------------------------------------------------

int MW_main(void)
{
	initialize();

	time_fwd_decel_start_us = time_fwd_start_us + 22e6;
	time_fwd_end_us = time_fwd_decel_start_us + 1e6;
	time_bwd_start_us = time_fwd_end_us + time_wait_us;
	time_bwd_decel_start_us = time_bwd_start_us + 22e6;
	time_bwd_end_us = time_bwd_decel_start_us + 1e6;

	state = STANDBY;

	while (1)
	{
		if (servo_on == 1)
		{
			if (process_time_us <= time_fwd_start_us) //往路の走行開始前
			{
				PEV_abz_clear(BDN0);
				bench_mode = 1;
			}
			else if (process_time_us < time_fwd_decel_start_us) //往路の加速・定速区間
			{
				PEV_pio_clr_bit(BDN0, CN1_41);
				PEV_pio_set_bit(BDN0, CN1_46);
				bench_mode = 2;
			}
			else if (process_time_us < time_fwd_end_us + time_wait_us / 2) //往路の減速区間
			{
				PEV_pio_clr_bit(BDN0, CN1_41);
				PEV_pio_clr_bit(BDN0, CN1_46);
				bench_mode = 3;
			}
			else if (process_time_us < time_bwd_start_us) //往路の走行開始前
			{
				PEV_pio_clr_bit(BDN0, CN1_41);
				PEV_pio_clr_bit(BDN0, CN1_46);
				bench_mode = 4;
			}
			else if (process_time_us < time_bwd_decel_start_us) //往路の加速・定速区間
			{
				PEV_pio_set_bit(BDN0, CN1_41);
				PEV_pio_set_bit(BDN0, CN1_46);
				bench_mode = 5;
			}
			else if (process_time_us < time_bwd_end_us + time_wait_us / 2) //往路の減速区間
			{
				PEV_pio_set_bit(BDN0, CN1_41);
				PEV_pio_clr_bit(BDN0, CN1_46);
				bench_mode = 6;
			}
			else //動作終了
			{
				PEV_pio_out(BDN0, OFF);
				bench_mode = 7;
			}
		}
	}
}