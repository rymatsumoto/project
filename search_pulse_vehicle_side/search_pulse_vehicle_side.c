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

#define BDN0 1 //路面側PEVボード
#define BDN1 2 //車両側PEVボード
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
#define Motor_Radius 0.025
#define Reduction_Ratio 0.56
#define INV_FREQ 85000
#define DEAD_TIME 300
#define TIMER_0_INTERVAL 10
#define TIMER_1_INTERVAL 500
#define TIMER_2_INTERVAL 1000

#define SLEEP 0
#define STANDBY 1
#define DETECT 2
#define PAUSE 3
#define TRANSMIT 4
#define SHORT 5
#define RECT 6
#define STOP 7

int max_velocity_rpm = 847;
float travel_length_m = 2.2;
float const_velocity_length_m = 2.0;

int time_now_us = 0;
float time_init_us = 0;
float time_fwd_start_us = 1e6;
float time_wait_us = 3e6;
float time_fwd_decel_start_us;
float time_fwd_end_us;
float time_bwd_start_us;
float time_bwd_decel_start_us;
float time_bwd_end_us;
int pause_time_us = 0;
int transmit_time_us = 0;
// volatile int pause_time_thld_us = 5e4; // 5 kmph
volatile int pause_time_thld_us = 5e5; // depth_cam_detection
// volatile int transmit_time_thld_us = 1e5; // 5kmph
volatile int transmit_time_thld_us = 1e6; // depth_cam_detection

float max_velocity_m_us;
int r_bench_mode = 0;
int r_servo_on = 0;
volatile int w_servo_on = 0;
int encoder_count = 0;
int encoder_previous = 0;
int encoder_diff = 0;

INT32 adc_0_data_peak;
INT32 adc_1_data_peak;
float i1_ampl;
float i2_ampl;
volatile float i1_weight = 1.1;
volatile float i2_weight = 1.1;

volatile int inverter_mode_tx = 0;
volatile int inverter_mode_rx = 0;
// volatile int inverter_start_count = 90000; // 5 kmph
volatile int inverter_start_count = 60e3; // depth_cam_detection

volatile float inv_mod_BDN0_trans = 0;
volatile float inv_mod_BDN1_pulse = 0.9;

volatile float i1_ampl_thld_detect = 3;
volatile float i2_ampl_thld_detect = 5;
volatile float i1_ampl_thld_transmit = 10;

//----------------------------------------------------------------------------------------
//　エンコーダ読み出し & 時間計測
//----------------------------------------------------------------------------------------

void encoding(void)
{
	encoder_count = PEV_abz_read(BDN0);
	encoder_diff = encoder_count - encoder_previous;
	encoder_previous = encoder_count;
}

void time_count(void)
{
	time_now_us += TIMER_2_INTERVAL;

	if (r_servo_on == 0)
	{
		time_init_us += TIMER_2_INTERVAL;
	}
	if (inverter_mode_tx == PAUSE)
	{
		pause_time_us += TIMER_2_INTERVAL;
	}
	else if (inverter_mode_tx == TRANSMIT)
	{
		transmit_time_us += TIMER_2_INTERVAL;
	}
}

interrupt void encoding_time_count(void)
{
	C6657_timer2_clear_eventflag();
	encoding();
	time_count();
}

//----------------------------------------------------------------------------------------
//　車両側インバータ　パルス開始または整流モード遷移
//----------------------------------------------------------------------------------------

interrupt void rx_inverter_turn_on(void)
{
	C6657_timer1_clear_eventflag();

	if (inverter_mode_rx == STANDBY)
	{
		PEV_inverter_set_uvw(BDN1, -inv_mod_BDN1_pulse, inv_mod_BDN1_pulse, 0, 0);
		PEV_inverter_control_gate(BDN0, SHORT_MODE);
		PEV_inverter_start_pwm(BDN0);
		PEV_inverter_start_pwm(BDN1);
		inverter_mode_tx = SHORT;
		inverter_mode_rx = DETECT;
	}
	else if (inverter_mode_rx == DETECT)
	{
		PEV_inverter_stop_pwm(BDN1);
		PEV_inverter_control_gate(BDN1, RECT_MODE);
		PEV_inverter_start_pwm(BDN1);
		inverter_mode_rx = RECT;
	}
}

//----------------------------------------------------------------------------------------
//　路面側インバータ　給電開始または停止　＆　車両側インバータ　パルス停止
//----------------------------------------------------------------------------------------

void tx_inverter_turn_on_off(void)
{
	if (inverter_mode_tx == SHORT)
	{
		if (i1_ampl > i1_ampl_thld_detect)
		{
			inverter_mode_tx = PAUSE;
		}
	}
	else if (inverter_mode_tx == PAUSE)
	{
		if (pause_time_us > pause_time_thld_us)
		{
			PEV_inverter_stop_pwm(BDN0);
			PEV_inverter_control_gate(BDN0, TRANS_MODE);
			PEV_inverter_set_uvw(BDN0, -inv_mod_BDN0_trans, inv_mod_BDN0_trans, 0, 0);
			PEV_inverter_start_pwm(BDN0);
			inverter_mode_tx = TRANSMIT;
		}
	}
	else if (inverter_mode_tx == TRANSMIT)
	{
		if (i1_ampl > i1_ampl_thld_transmit && transmit_time_us > transmit_time_thld_us)
		{
			PEV_inverter_stop_pwm(BDN0);
			inverter_mode_tx = STOP;
		}
	}
}

void rx_inverter_turn_off(void)
{
	if (inverter_mode_rx == DETECT)
	{
		if (i2_ampl > i2_ampl_thld_detect)
		{
			PEV_inverter_stop_pwm(BDN1);
			inverter_mode_rx = STANDBY;
		}
	}
}

interrupt void tx_inverter_turn_on_off_rx_inverter_turn_off(void)
{
	C6657_timer0_clear_eventflag();
	tx_inverter_turn_on_off();
	rx_inverter_turn_off();
}

//----------------------------------------------------------------------------------------
//　電流包絡線検波
//----------------------------------------------------------------------------------------

interrupt void read_envelope(void)
{
	int0_ack();
    adc_0_data_peak = IPFPGA_read(BDN2, 0x17);
	adc_1_data_peak = IPFPGA_read(BDN2, 0x18);
	i1_ampl = adc_0_data_peak * 125. / 8000 * i1_weight;
	i2_ampl = adc_1_data_peak * 125. / 8000 * i2_weight;
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
    int0_init_vector(read_envelope, (CSL_IntcVectId)8, FALSE);
    PEV_inverter_control_gate(BDN0, TRANS_MODE);
    PEV_inverter_set_uvw(BDN0, 0, 0, 0, 0);
    PEV_inverter_enable_int(BDN0);
    int0_enable_int();

	PEV_init(BDN1);
	PEV_inverter_init(BDN1, INV_FREQ, DEAD_TIME);
	PEV_inverter_control_gate(BDN1, TRANS_MODE);
	PEV_inverter_set_uvw(BDN1, 0, 0, 0, 0);

	C6657_timer0_init(TIMER_0_INTERVAL);
	C6657_timer0_init_vector(tx_inverter_turn_on_off_rx_inverter_turn_off, (CSL_IntcVectId)7);
	C6657_timer0_start();
	C6657_timer0_enable_int();

	C6657_timer1_init(TIMER_1_INTERVAL);
	C6657_timer1_init_vector(rx_inverter_turn_on, (CSL_IntcVectId)6);
	C6657_timer1_start();
	C6657_timer1_enable_int();

	C6657_timer2_init(TIMER_2_INTERVAL);
	C6657_timer2_init_vector(encoding_time_count, (CSL_IntcVectId)5);
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

	// max_velocity_m_us = (float)max_velocity_rpm * Motor_Radius * Reduction_Ratio * 2 * 3.14 * 1e-6 / 60.0;

	// time_fwd_decel_start_us = time_fwd_start_us + travel_length_m / max_velocity_m_us;
	// time_fwd_end_us = time_fwd_start_us + (2 * travel_length_m - const_velocity_length_m) / max_velocity_m_us;
	// time_bwd_start_us = time_fwd_end_us + time_wait_us;
	// time_bwd_decel_start_us = time_bwd_start_us + travel_length_m / max_velocity_m_us;
	// time_bwd_end_us = time_bwd_start_us + (2 * travel_length_m - const_velocity_length_m) / max_velocity_m_us;

	time_fwd_decel_start_us = time_fwd_start_us + 22e6;
	time_fwd_end_us = time_fwd_decel_start_us + 1e6;
	time_bwd_start_us = time_fwd_end_us + time_wait_us;
	time_bwd_decel_start_us = time_bwd_start_us + 22e6;
	time_bwd_end_us = time_bwd_decel_start_us + 1e6;

	while (1)
	{
		if (w_servo_on == 1)
		{
			if (r_servo_on == 0)
			{
				w_servo_on = 0;
				r_servo_on = 1;
			}
		}
		if (r_servo_on == 1)
		{
			if (time_now_us < time_init_us + time_fwd_start_us) //往路の走行開始前
			{
				PEV_abz_clear(BDN0);
				r_bench_mode = 1;
			}
			else if (time_now_us < time_init_us + time_fwd_decel_start_us) //往路の加速・定速区間
			{
				PEV_pio_clr_bit(BDN0, CN1_41);
				PEV_pio_set_bit(BDN0, CN1_46);
				r_bench_mode = 2;
			}
			else if (time_now_us < time_init_us + time_fwd_end_us + time_wait_us / 2) //往路の減速区間
			{
				PEV_pio_clr_bit(BDN0, CN1_41);
				PEV_pio_clr_bit(BDN0, CN1_46);
				r_bench_mode = 3;
			}
			else if (time_now_us < time_init_us + time_bwd_start_us) //往路の走行開始前
			{
				PEV_pio_clr_bit(BDN0, CN1_41);
				PEV_pio_clr_bit(BDN0, CN1_46);
				r_bench_mode = 4;
			}
			else if (time_now_us < time_init_us + time_bwd_decel_start_us) //往路の加速・定速区間
			{
				PEV_pio_set_bit(BDN0, CN1_41);
				PEV_pio_set_bit(BDN0, CN1_46);
				r_bench_mode = 5;
			}
			else if (time_now_us < time_init_us + time_bwd_end_us + time_wait_us / 2) //往路の減速区間
			{
				PEV_pio_set_bit(BDN0, CN1_41);
				PEV_pio_clr_bit(BDN0, CN1_46);
				r_bench_mode = 6;
			}
			else //動作終了
			{
				PEV_pio_out(BDN0, OFF);
				r_bench_mode = 7;
			}
		}

		if (encoder_count > inverter_start_count && inverter_mode_rx == SLEEP)
		{
			inverter_mode_rx = STANDBY;
		}
	}
}