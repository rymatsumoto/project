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

#define BDN_PEV_TX 1  //送電側PEVボード
#define BDN_PEV_RX 0  //受電側PEVボード
#define BDN_FPGA 0    //FPGAボード

#define TIMER_0_INTERVAL 100
#define TIMER_1_INTERVAL 5
                            //WN WP VN VP UN UP
#define TRANS_MODE 80       //00 00 01 01 00 00
#define SHORT_MODE 187      //00 00 10 11 10 11

#define INV_FREQ 85000
#define DEAD_TIME 600

#define OFF 0
#define CN1_40 0
#define CN1_41 1
#define CN1_45 2
#define CN1_46 3

#define SLEEP 0
#define TRIGGER 1
#define STANDBY 2
#define SHORT 3
#define PULSE 4
#define PAUSE 5
#define RECT 6
#define TRANSMIT 7
#define STOP 8

#define RPI_SIGNAL_THLD 1.5

float time_fwd_start_us = 0;
float time_wait_us = 2e6;
float time_fwd_decel_start_us;
float time_fwd_end_us;
float time_bwd_start_us;
float time_bwd_decel_start_us;
float time_bwd_end_us;

volatile int end_trans_time_us = 3e6;
volatile int end_pulse_time_us = 500;
volatile int end_pause_time_us = 500;
volatile int end_trigger_time_us = 20;
volatile int end_standby_time_us = 1e6;

int bench_mode = 0;
volatile int servo_on = 0;

int run_time_us = 0;
int pause_time_us = 0;
int trans_time_us = 0;
int pulse_time_us = 0;
int trigger_time_us = 0;
int standby_time_us = 0;

volatile int inverter_mode_tx = 0;
volatile int inverter_mode_rx = 0;

float data[] = {0., 0., 0., 0.};
float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float rpi_signal_current = 0;
float rpi_signal_previous = 0;

volatile int inverter_start_count = 5e4; // 82.25 mmps

int encoder_count = 0;
int encoder_count_start_transfer = 0;

INT32 adc_0_data_peak;
INT32 adc_1_data_peak;
float i1_ampl = 0;
float i2_ampl = 0;
volatile float i1_weight = 1.1;
volatile float i2_weight = 1.1;
volatile int overcurrent_protection = 0;
volatile float i1_ampl_max = 50;
volatile float i2_ampl_max = 50;
volatile float i1_ampl_thld_detect = 1;

volatile float duty_trans = 1;
volatile float duty_pulse = 0.1;

//----------------------------------------------------------------------------------------
//　走行開始からの経過時間を計測（timer0）
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
//　トリガー用パルスの印加時間，車両側のパルス印加時間，路面側のPAUSE時間，路面側の給電時間を計測（timer1）
//----------------------------------------------------------------------------------------

void count_trans_time(void)
{
	if (inverter_mode_tx == TRIGGER)
	{
		trigger_time_us += TIMER_1_INTERVAL;
	}
	if (inverter_mode_tx == STANDBY)
	{
		standby_time_us += TIMER_1_INTERVAL;
	}
	if (inverter_mode_rx == PULSE)
	{
		pulse_time_us += TIMER_1_INTERVAL;
	}
	if (inverter_mode_tx == PAUSE)
	{
		pause_time_us += TIMER_1_INTERVAL;
	}
	if (inverter_mode_tx == TRANSMIT)
	{
		trans_time_us += TIMER_1_INTERVAL;
	}
}

//----------------------------------------------------------------------------------------
//　RaspberryPiから走行開始と給電開始の信号を受け取る（timer1）
//----------------------------------------------------------------------------------------

void read_rpi_signal(void)
{
	PEV_ad_in_4ch(BDN_PEV_TX, 0, data);

	rpi_signal_current = data[0];

	if (bench_mode == 0)
	{
		if (rpi_signal_previous < RPI_SIGNAL_THLD && rpi_signal_current > RPI_SIGNAL_THLD)
		{
			servo_on = 1;
		}
	}
	else if (bench_mode == 2 && inverter_mode_rx == SLEEP)
	{
		if (rpi_signal_previous < RPI_SIGNAL_THLD && rpi_signal_current > RPI_SIGNAL_THLD)
		{
			encoder_count_start_transfer = PEV_abz_read(BDN_PEV_TX);
			PEV_inverter_set_uvw(BDN_PEV_RX, duty_pulse-1, 1-duty_pulse, 0, 0);
			PEV_inverter_start_pwm(BDN_PEV_RX);
			inverter_mode_rx = PULSE;
		}
	}

	rpi_signal_previous = rpi_signal_current;
}

//----------------------------------------------------------------------------------------
//　トリガー用パルス，車両側からのパルス，路面側からの給電を停止（timer1）
//----------------------------------------------------------------------------------------

void stop_inverter(void)
{
	if (inverter_mode_tx == TRIGGER)
	{
		if (trigger_time_us > end_trigger_time_us)
		{
			PEV_inverter_stop_pwm(BDN_PEV_TX);
			inverter_mode_tx = STANDBY;
		}
	}
	if (inverter_mode_tx == STANDBY)
	{
		if (standby_time_us > end_standby_time_us)
		{
			PEV_inverter_control_gate(BDN_PEV_TX, SHORT_MODE);
			PEV_inverter_start_pwm(BDN_PEV_TX);
			inverter_mode_tx = SHORT;
		}
	}
	if (inverter_mode_tx == TRANSMIT)
	{
		if (trans_time_us > end_trans_time_us)
		{
			PEV_inverter_stop_pwm(BDN_PEV_TX);
			inverter_mode_tx = STOP;
		}
	}
	if (inverter_mode_rx == PULSE)
	{
		if (pulse_time_us > end_pulse_time_us)
		{
			PEV_inverter_stop_pwm(BDN_PEV_RX);
			inverter_mode_rx = RECT;
		}
	}
}

//----------------------------------------------------------------------------------------
//　路面側からの給電を開始（timer1）
//----------------------------------------------------------------------------------------

void start_power_transfer(void)
{
	if (inverter_mode_tx == SHORT)
	{
		if (i1_ampl > i1_ampl_thld_detect)
		{
			inverter_mode_tx = PAUSE;
		}
	}
	if (inverter_mode_tx == PAUSE)
	{
		if (pause_time_us > end_pause_time_us)
		{
			PEV_inverter_stop_pwm(BDN_PEV_TX);
			PEV_inverter_control_gate(BDN_PEV_TX, TRANS_MODE);
			PEV_inverter_set_uvw(BDN_PEV_TX, duty_trans-1, 1-duty_trans, 0, 0);
			PEV_inverter_start_pwm(BDN_PEV_TX);
			inverter_mode_tx = TRANSMIT;
		}
	}
}

//----------------------------------------------------------------------------------------
//　timer1の割込み関数
//----------------------------------------------------------------------------------------

interrupt void timer_1_int_function(void)
{
	C6657_timer1_clear_eventflag();

	count_trans_time();
	read_rpi_signal();
	stop_inverter();
	start_power_transfer();
}

//----------------------------------------------------------------------------------------
//　電流包絡線検波（キャリア同期割込み）
//----------------------------------------------------------------------------------------

interrupt void read_envelope(void)
{
	int0_ack();

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
			inverter_mode_tx = STOP;
			inverter_mode_rx = STOP;
		}
	}
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
	PEV_ad_set_range(BDN_PEV_TX, range);
	PEV_ad_set_mode(BDN_PEV_TX, 0);

	int_disable();

	C6657_timer0_init(TIMER_0_INTERVAL);
	C6657_timer0_init_vector(count_run_time, (CSL_IntcVectId)5);
	C6657_timer0_start();
	C6657_timer0_enable_int();

	C6657_timer1_init(TIMER_1_INTERVAL);
	C6657_timer1_init_vector(timer_1_int_function, (CSL_IntcVectId)6);
	C6657_timer1_start();
	C6657_timer1_enable_int();

	PEV_init(BDN_PEV_TX);
	PEV_inverter_disable_int(BDN_PEV_TX);
    PEV_inverter_init(BDN_PEV_TX, INV_FREQ, DEAD_TIME);
    PEV_inverter_init_int_timing(BDN_PEV_TX, 1, 0, 0);
    PEV_int_init(BDN_PEV_TX, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(read_envelope, (CSL_IntcVectId)8, FALSE);
    PEV_inverter_control_gate(BDN_PEV_TX, TRANS_MODE);
    PEV_inverter_enable_int(BDN_PEV_TX);
    int0_enable_int();

	PEV_init(BDN_PEV_RX);
	PEV_inverter_disable_int(BDN_PEV_RX);
    PEV_inverter_init(BDN_PEV_RX, INV_FREQ, DEAD_TIME);
    PEV_inverter_control_gate(BDN_PEV_RX, TRANS_MODE);

	int_enable();

	PEV_abz_init_maxcount(BDN_PEV_TX, 100100100);
	PEV_abz_set_mode(BDN_PEV_TX, 0, 1);

	PEV_pio_out(BDN_PEV_TX, OFF);
	PEV_pio_set_bit(BDN_PEV_TX, CN1_40);
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

		if (encoder_count > inverter_start_count && inverter_mode_tx == SLEEP)
		{
			PEV_inverter_set_uvw(BDN_PEV_TX, duty_pulse-1, 1-duty_pulse, 0, 0);
			PEV_inverter_start_pwm(BDN_PEV_TX);
			inverter_mode_tx = TRIGGER;
		}
	}
}
