//----------------------------------------------------------------------------------------
//title
// 1対2コイル給電（クロスカップリングなし）
//designer
// ryo matsumoto
//----------------------------------------------------------------------------------------

#include    <mwio4.h>

#define BDN0 0              //Rx1可変C制御&Tx電流包絡線検波用FPGAボード
#define BDN1 1              //Rx2可変C制御用FPGAボード
#define BDN2 0              //PEVボード

                            //WN WP VN VP UN UP
#define TRANS_MODE 80       //00 00 01 01 00 00
#define INV_FREQ 85000
#define DEAD_TIME 600
#define REF_PWMC 260
#define SMA_LEN 50
#define TIMER0_INTERVAL 5
#define TIMER1_INTERVAL 2000
#define TIMER2_INTERVAL 2000

volatile int LPF_CUTTOFF = 2000;   // LPFカットオフ周波数 [Hz]
volatile float P_O_STEP_SIZE = 0.005;

INT32 adc_0_data_peak;
float itx_ampl;
volatile float itx_weight = 1.1;
volatile float irx1_weight = 1;
volatile float irx2_weight = 1;
volatile float itx_ampl_ref;
float vtx_ampl_ref = 0;
volatile int set_pwm_on_trans_w_o_control = 0;
volatile int set_pwm_on_trans_w_control = 0;
int control_on = 0;
volatile float proportionalGain;
volatile float integralGain;
volatile float derivativeGain;
float error_crnt = 0;
float error_prvs = 0;
float error_integral = 0;
float period = 1 / (float)INV_FREQ;
volatile float vtxdc;
volatile float inv_mod_BDN2 = 0;
const float pi = 3.14;

float range[] = {5., 5., 5., 5., 5., 5., 5., 5.};
float data[] = {0., 0., 0., 0.};
float irx1dc_crnt = 0;
float irx1dc_prvs = 0;
float irx2dc_crnt = 0;
float irx2dc_prvs = 0;
float irx1dc_lpf_crnt = 0;
float irx1dc_lpf_prvs = 0;
float irx2dc_lpf_crnt = 0;
float irx2dc_lpf_prvs = 0;
float lpf_T;
float lpf_A;
float lpf_B;

float irx1dc_p_o_crnt = 0;
float irx1dc_p_o_prvs = 0;
float irx2dc_p_o_crnt = 0;
float irx2dc_p_o_prvs = 0;
int up_down_rx1 = 1;
int up_down_rx2 = 1;

volatile float pwm_out_BDN0 = 0;
volatile float pwm_out_BDN1 = 0;
volatile float wref_BDN0 = 0;
volatile float wref_BDN1 = 0;
volatile int carrier_cnt_max;
volatile int carrier_cnt_hlf;
volatile int pwmc_rx1_control_on = 0;
volatile int pwmc_rx2_control_on = 0;

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
    adc_0_data_peak = IPFPGA_read(BDN0, 0x17);
    itx_ampl = adc_0_data_peak * 125. / 8000 * itx_weight;
}

//----------------------------------------------------------------------------------------
//　Tx電流制御
//----------------------------------------------------------------------------------------

void current_control(void)
{
    if (control_on == 1)
    {
        error_crnt = itx_ampl_ref - itx_ampl;
        error_integral = error_integral + (error_crnt + error_prvs) / 2 * period;
        vtx_ampl_ref = proportionalGain * error_crnt + integralGain * error_integral;
        error_prvs = error_crnt;

        if (vtx_ampl_ref < 0)
        {
            vtx_ampl_ref = 0;
        }
        else if (vtx_ampl_ref > vtxdc * 4 / pi)
        {
            vtx_ampl_ref = vtxdc * 4 / pi;
        }

        inv_mod_BDN2 = 2 / pi * arccos(pi * vtx_ampl_ref / (4 * vtxdc));

        if (inv_mod_BDN2 < 0)
        {
            inv_mod_BDN2 = 0;
        }
        else if (inv_mod_BDN2 > 1)
        {
            inv_mod_BDN2 = 1;
        }

        PEV_inverter_set_uvw(BDN2, -inv_mod_BDN2, inv_mod_BDN2, 0, 0);
    }
    else
    {
        PEV_inverter_set_uvw(BDN2, -inv_mod_BDN2, inv_mod_BDN2, 0, 0);
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
// Rx1，Rx2のdc電流を取得
//----------------------------------------------------------------------------------------

interrupt void read_irx1dc_irx2dc(void)
{
    C6657_timer0_clear_eventflag();

    lpf_T = 1 / (2 * 3.14 * LPF_CUTTOFF);
    lpf_A = TIMER0_INTERVAL * 1e-6 / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T);
    lpf_B = (TIMER0_INTERVAL * 1e-6 - 2 * lpf_T) / (TIMER0_INTERVAL * 1e-6 + 2 * lpf_T);

    PEV_ad_in_4ch(BDN2, 0, data);
    irx1dc_crnt = data[0] * 25 * irx1_weight;                           // 2V 50A
    irx2dc_crnt = data[1] * 25 * irx2_weight;                           // 2V 50A
    irx1dc_lpf_crnt = lpf_A * irx1dc_crnt + lpf_A * irx1dc_prvs - lpf_B * irx1dc_lpf_prvs;
    irx2dc_lpf_crnt = lpf_A * irx2dc_crnt + lpf_A * irx2dc_prvs - lpf_B * irx2dc_lpf_prvs;

    irx1dc_prvs = irx1dc_crnt;
    irx2dc_prvs = irx2dc_crnt;
    irx1dc_lpf_prvs = irx1dc_lpf_crnt;
    irx2dc_lpf_prvs = irx2dc_lpf_crnt;
}

//----------------------------------------------------------------------------------------
// Rx1可変C制御
//----------------------------------------------------------------------------------------

interrupt void pwmc_rx1_control(void)
{
    C6657_timer1_clear_eventflag();

    if (pwmc_rx1_control_on == 1) {
        irx1dc_p_o_crnt = irx1dc_lpf_crnt;

        if (irx1dc_p_o_crnt < irx1dc_p_o_prvs && up_down_rx1 == 1)
        {
            pwm_out_BDN0 = pwm_out_BDN0 - P_O_STEP_SIZE;
            up_down_rx1 = -1;
        }
        else if (irx1dc_p_o_crnt < irx1dc_p_o_prvs && up_down_rx1 == -1)
        {
            pwm_out_BDN0 = pwm_out_BDN0 + P_O_STEP_SIZE;
            up_down_rx1 = 1;
        }
        else if (irx1dc_p_o_crnt >= irx1dc_p_o_prvs && up_down_rx1 == 1)
        {
            pwm_out_BDN0 = pwm_out_BDN0 + P_O_STEP_SIZE;
            up_down_rx1 = 1;
        }
        else if (irx1dc_p_o_crnt >= irx1dc_p_o_prvs && up_down_rx1 == -1)
        {
            pwm_out_BDN0 = pwm_out_BDN0 - P_O_STEP_SIZE;
            up_down_rx1 = -1;
        }

        if (pwm_out_BDN0 > 1) {
            pwm_out_BDN0 = 1;
        }
        else if (pwm_out_BDN0 < 0) {
            pwm_out_BDN0 = 0;
        }

        wref_BDN0 = pwm_out_BDN0 * carrier_cnt_max;
        IPFPGA_write(BDN0, 0x04, wref_BDN0);

        irx1dc_p_o_prvs = irx1dc_p_o_crnt;
    }

    else {
        wref_BDN0 = pwm_out_BDN0 * carrier_cnt_max;
        IPFPGA_write(BDN0, 0x04, wref_BDN0);
    }
}

//----------------------------------------------------------------------------------------
// Rx2可変C制御
//----------------------------------------------------------------------------------------

interrupt void pwmc_rx2_control(void)
{
    C6657_timer2_clear_eventflag();

    if (pwmc_rx2_control_on == 1) {
        irx2dc_p_o_crnt = irx2dc_lpf_crnt;

        if (irx2dc_p_o_crnt < irx2dc_p_o_prvs && up_down_rx2 == 1)
        {
            pwm_out_BDN1 = pwm_out_BDN1 - P_O_STEP_SIZE;
            up_down_rx2 = -1;
        }
        else if (irx2dc_p_o_crnt < irx2dc_p_o_prvs && up_down_rx2 == -1)
        {
            pwm_out_BDN1 = pwm_out_BDN1 + P_O_STEP_SIZE;
            up_down_rx2 = 1;
        }
        else if (irx2dc_p_o_crnt >= irx2dc_p_o_prvs && up_down_rx2 == 1)
        {
            pwm_out_BDN1 = pwm_out_BDN1 + P_O_STEP_SIZE;
            up_down_rx2 = 1;
        }
        else if (irx2dc_p_o_crnt >= irx2dc_p_o_prvs && up_down_rx2 == -1)
        {
            pwm_out_BDN1 = pwm_out_BDN1 - P_O_STEP_SIZE;
            up_down_rx2 = -1;
        }

        if (pwm_out_BDN1 > 1) {
            pwm_out_BDN1 = 1;
        }
        else if (pwm_out_BDN1 < 0) {
            pwm_out_BDN1 = 0;
        }

        wref_BDN1 = pwm_out_BDN1 * carrier_cnt_max;
        IPFPGA_write(BDN1, 0x04, wref_BDN1);

        irx2dc_p_o_prvs = irx2dc_p_o_crnt;
    }

    else {
        wref_BDN1 = pwm_out_BDN1 * carrier_cnt_max;
        IPFPGA_write(BDN1, 0x04, wref_BDN1);
    }
}

//----------------------------------------------------------------------------------------
//　初期化
//----------------------------------------------------------------------------------------

void initialize(void)
{
    PEV_ad_set_range(BDN2, range);
    PEV_ad_set_mode(BDN2, 0);

    carrier_cnt_max = 50000000. / INV_FREQ;
    carrier_cnt_hlf = 25000000. / INV_FREQ;
    
    IPFPGA_write(BDN0, 0x01, carrier_cnt_max);
    IPFPGA_write(BDN0, 0x02, carrier_cnt_hlf);
    IPFPGA_write(BDN0, 0x03, carrier_cnt_hlf);
    IPFPGA_write(BDN0, 0x04, wref_BDN0);
    IPFPGA_write(BDN0, 0x05, DEAD_TIME / 10);
    IPFPGA_write(BDN0, 0x09, REF_PWMC);
    IPFPGA_write(BDN0, 0x07, 1);

    IPFPGA_write(BDN1, 0x01, carrier_cnt_max);
    IPFPGA_write(BDN1, 0x02, carrier_cnt_hlf);
    IPFPGA_write(BDN1, 0x03, carrier_cnt_hlf);
    IPFPGA_write(BDN1, 0x04, wref_BDN1);
    IPFPGA_write(BDN1, 0x05, DEAD_TIME / 10);
    IPFPGA_write(BDN1, 0x09, REF_PWMC);
    IPFPGA_write(BDN1, 0x07, 1);

    PEV_init(BDN2);

    int_disable();

    PEV_inverter_disable_int(BDN2);

    PEV_inverter_init(BDN2, INV_FREQ, DEAD_TIME);
    PEV_inverter_init_int_timing(BDN2, 1, 0, 0);
    PEV_int_init(BDN2, 2, 0, 0, 0, 0, 0, 0, 0);
    int0_init_vector(sync_interrupt, (CSL_IntcVectId)6, FALSE);
    PEV_inverter_control_gate(BDN2, TRANS_MODE);
    PEV_inverter_set_uvw(BDN2, 0, 0, 0, 0);

    C6657_timer0_init(TIMER0_INTERVAL);
    C6657_timer0_init_vector(read_irx1dc_irx2dc, (CSL_IntcVectId)5);
    C6657_timer0_start();

    C6657_timer1_init(TIMER1_INTERVAL);
    C6657_timer1_init_vector(pwmc_rx1_control, (CSL_IntcVectId)7);
    C6657_timer1_start();

    C6657_timer2_init(TIMER2_INTERVAL);
    C6657_timer2_init_vector(pwmc_rx2_control, (CSL_IntcVectId)8);
    C6657_timer2_start();

    PEV_inverter_enable_int(BDN2);

    int0_enable_int();

    C6657_timer0_enable_int();
    C6657_timer1_enable_int();
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
        if (set_pwm_on_trans_w_control == 1)
        {
            control_on = 1;
            PEV_inverter_start_pwm(BDN2);
            set_pwm_on_trans_w_control = -1;
        }
        if (set_pwm_on_trans_w_o_control == 1)
        {
            PEV_inverter_start_pwm(BDN2);
            set_pwm_on_trans_w_o_control = -1;
        }
    }
}
