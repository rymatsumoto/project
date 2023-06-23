//----------------------------------------------------------------------------------------
//title
//　簡易包絡線モデルに基づいた電流包絡線FF制御
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
volatile float i1_weight_pos = 1.18;
volatile float i1_weight_neg = 1;

float v1_ampl_ref = 0;
float v1_ampl_ref_prvs = 0;
float v1_ampl_ref_prvs_prvs = 0;
volatile float i1_ampl_ref = 1.5;
float i1_ampl_ref_prvs = 0;
float i1_ampl_ref_prvs_prvs = 0;

volatile int set_pwm_on_trans_w_o_control = 0;
volatile int set_pwm_on_trans_w_control = 0;
int control_on = 0;

const float pi = 3.14;
float period = 1 / (float)INV_FREQ / 2;
volatile float v1dc;
float inv_mod_BDN0 = 1;

float T;
float f;
float w;
float L1;
float L2;
float M;
float RL;

float alpha1;
float alpha0;
float beta1;
float beta0;

float lpf_T;

float p0;
float p1;
float p2;
float q0;
float q1;
float q2;

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
        v1_ampl_ref = q2/p0*i1_ampl_ref_prvs_prvs + q1/p0*i1_ampl_ref_prvs + q0/p0*i1_ampl_ref - p2/p0*v1_ampl_ref_prvs_prvs - p1/p0*v1_ampl_ref_prvs;

        v1_ampl_ref_prvs_prvs = v1_ampl_ref_prvs;
        v1_ampl_ref_prvs = v1_ampl_ref;
        
        i1_ampl_ref_prvs_prvs = i1_ampl_ref_prvs;
        i1_ampl_ref_prvs = i1_ampl_ref;
        
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
    T = 11.76e-6;
    f = 1 / T;
    w = f * 2 * pi;
    L1 = 246e-6;
    L2 = 106e-6;
    M = 19.5e-6;
    RL = 5.7;

    lpf_T = 100e-6;

    alpha1 = RL / (2*L2);
    alpha0 = (w*M)*(w*M) / (4*L1*L2);
    beta1 = 1 / (2*L1);
    beta0 = RL / (4*L1*L2);

    p0 = 4*lpf_T*beta1 + 2*period*(beta1+lpf_T*beta0) + period*period*beta0;
    p1 = -8*lpf_T*beta1 + 2*period*period*beta0;
    p2 = 4*lpf_T*beta1 - 2*period*(beta1+lpf_T*beta0) + period*period*beta0;
    q0 = 4 + 2*period*alpha1 + period*period*alpha0;
    q1 = -8 + 2*period*period*alpha0;
    q2 = 4 - 2*period*alpha1 + period*period*alpha0;

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