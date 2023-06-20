//----------------------------------------------------------------------------------------
//title
//　簡易包絡線モデルに基づいた電流包絡線PID制御
//designer
//　ryo matsumoto
//----------------------------------------------------------------------------------------

#include    <mwio4.h>

#define BDN0 1              //PEVボード
#define BDN1 1              //FPGAボード
                            //WN WP VN VP UN UP
#define TRANS_MODE 80       //00 00 01 01 00 00
#define INV_FREQ 85000
#define DEAD_TIME 300

INT32 adc_0_data_peak;
float i1_ampl;
//volatile float i1_weight = 1.18;
volatile float i1_weight_pos = 1.18;
volatile float i1_weight_neg = 1;
volatile float i1_ampl_ref;
float v1_ampl_ref = 0;
volatile int set_pwm_on_trans_w_o_control = 0;
volatile int set_pwm_on_trans_w_control = 0;
int control_on = 0;
volatile float proportionalGain;
volatile float integralGain;
volatile float derivativeGain;
volatile float tau;
float error_crnt = 0;
float error_prvs = 0;
float error_integral = 0;
float error_derivative_crnt = 0;
float error_derivative_prvs = 0;
float period = 1 / (float)INV_FREQ / 2;
volatile float v1dc;
volatile float inv_mod_BDN0 = 0;
const float pi = 3.14;

//----------------------------------------------------------------------------------------
//　arccos関数
//----------------------------------------------------------------------------------------

float arccos(float x)
{
    float y = mwarctan(mwsqrt(1-x*x) / x);
    return y;
}

//----------------------------------------------------------------------------------------
//　電流包絡線検波
//----------------------------------------------------------------------------------------

void read_envelope(void)
{
    adc_0_data_peak = IPFPGA_read(BDN1, 0x17);
//    i1_ampl = mwabs(adc_0_data_peak * 125. / 8000 * i1_weight);
    if (adc_0_data_peak < 0)
    {
        i1_ampl = mwabs(adc_0_data_peak * 125. / 8000 * i1_weight_neg);
    }
    else
    {
        i1_ampl = mwabs(adc_0_data_peak * 125. / 8000 * i1_weight_pos);
    }
}

//----------------------------------------------------------------------------------------
//　電流制御
//----------------------------------------------------------------------------------------

void current_control(void)
{
    if (control_on == 1)
    {
        error_crnt = i1_ampl_ref - i1_ampl;
        error_integral = error_integral + (error_crnt + error_prvs) / 2 * period;
        error_derivative_crnt = 2/(period+2*tau)*error_crnt - 2/(period+2*tau)*error_prvs - (period-2*tau)/(period+2*tau)*error_derivative_prvs;
        v1_ampl_ref = proportionalGain * error_crnt + integralGain * error_integral + derivativeGain * error_derivative_crnt;
        error_prvs = error_crnt;
        error_derivative_prvs = error_derivative_crnt;

        if (v1_ampl_ref < 0)
        {
            v1_ampl_ref = 0;
        }
        else if (v1_ampl_ref > v1dc * 4 / pi)
        {
            v1_ampl_ref = v1dc * 4 / pi;
        }

        inv_mod_BDN0 = 2 / pi * arccos(pi * v1_ampl_ref / (4 * v1dc));

        if (inv_mod_BDN0 < 0)
        {
            inv_mod_BDN0 = 0;
        }
        else if (inv_mod_BDN0 > 1)
        {
            inv_mod_BDN0 = 1;
        }

        PEV_inverter_set_uvw(BDN0, -inv_mod_BDN0, inv_mod_BDN0, 0, 0);
    }
    else
    {
        PEV_inverter_set_uvw(BDN0, -inv_mod_BDN0, inv_mod_BDN0, 0, 0);
    }
}

//----------------------------------------------------------------------------------------
//　割り込み関数
//----------------------------------------------------------------------------------------

interrupt void sync_interrupt(void)
{
    int0_ack();
    read_envelope();
    current_control();
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
    PEV_init(BDN0);

    int_disable();

    PEV_inverter_disable_int(BDN0);

    PEV_inverter_init(BDN0, INV_FREQ, DEAD_TIME);
    PEV_inverter_init_int_timing(BDN0, 3, 0, 0);
    PEV_int_init(BDN0, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(sync_interrupt, (CSL_IntcVectId)7, FALSE);
    PEV_inverter_control_gate(BDN0, TRANS_MODE);
    PEV_inverter_set_uvw(BDN0, 0, 0, 0, 0);

    PEV_inverter_enable_int(BDN0);

    int0_enable_int();

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
        if (set_pwm_on_trans_w_control == 1)
        {
            control_on = 1;
            PEV_inverter_start_pwm(BDN0);
            set_pwm_on_trans_w_control = -1;
        }
        if (set_pwm_on_trans_w_o_control == 1)
        {
            PEV_inverter_start_pwm(BDN0);
            set_pwm_on_trans_w_o_control = -1;
        }
    }
}