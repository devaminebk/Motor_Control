/******************************************************************************
 * SPWM HARDWARE-DRIVEN — TMS320F28379D LaunchPad
 * V3 : f_carrier + f_sinus + amplitude tous modifiables via Watch Window
 ******************************************************************************/

#include "driverlib.h"
#include "device.h"
#include <math.h>

/*===========================================================================
 * PARAMÈTRES — tous modifiables en live via Watch Window CCS
 *
 *   FREQ_CARRIER  : 1 000 … 200 000 Hz  → TBPRD = 100 000 … 500
 *   FREQ_SINUS    : > 0  … FREQ_CARRIER/9 Hz
 *   AMPLITUDE     : 0.0 … 1.0
 *===========================================================================*/
 /* Hz — porteuse triangulaire     */
volatile float FREQ_CARRIER = 4000.0f;
volatile float FREQ_SINUS   =  50.0f;   /* Hz — modulante sinusoïdale     */
volatile float AMPLITUDE    =  0.80f;     /* 0.0 … 1.0 — indice modulation  */

/*===========================================================================
 * ÉTAT INTERNE — calculé à chaque changement de paramètre
 *===========================================================================*/
volatile uint16_t TBPRD      = 5000U;     /* déduit de FREQ_CARRIER         */
volatile uint32_t increm     = 0U;
volatile uint32_t accum      = 0U;
volatile float    isr_amp    = 0.80f;     /* snapshot atomique pour l'ISR   */
volatile float    isr_half   = 2500.0f;   /* TBPRD/2 pour l'ISR             */

/*===========================================================================
 * DEBUG — Watch Window CCS
 *===========================================================================*/
volatile uint32_t isr_count  = 0U;
volatile uint16_t dbg_cmpa   = 0U;
volatile uint16_t dbg_dac    = 0U;
volatile float    dbg_phase  = 0.0f;
volatile float    dbg_sin    = 0.0f;

__interrupt void epwm1ISR(void);

/*===========================================================================
 * UTILITAIRES
 *===========================================================================*/

/* TBPRD à partir de f_carrier — SYSCLK = 200 MHz, mode up-down             */
static uint16_t calc_tbprd(float f_carrier)
{
    float tbprd_f = 100000000.0f / f_carrier;   /* 200e6 / 2 / f            */
    if(tbprd_f < 500.0f)    tbprd_f = 500.0f;   /* max ~200 kHz             */
    if(tbprd_f > 100000.0f) tbprd_f = 100000.0f;/* min ~1 kHz               */
    return (uint16_t)tbprd_f;
}

/* Incrément DDS                                                              */
static uint32_t calc_increm(float f_sin, float f_carrier)
{
    if(f_sin <= 0.0f)               f_sin = 0.1f;
    if(f_sin >= f_carrier / 2.0f)   f_sin = f_carrier / 2.0f - 1.0f;
    return (uint32_t)(4294967296.0f * (f_sin / f_carrier));
}

/* Recalcul complet — appelé depuis la boucle principale                      */
/* fc     : fréquence effective (fc_raw * 4) pour TBPRD                      */
/* fc_raw : fréquence saisie par l'utilisateur pour le ratio DDS             */
static void apply_params(float fc, float fc_raw, float fs, float amp)
{
    uint16_t tbprd_new  = calc_tbprd(fc);
    uint32_t increm_new = calc_increm(fs, fc_raw);
    float    half_new   = (float)tbprd_new * 0.5f;

    /* Section critique : on coupe les IRQ le temps de tout mettre à jour    */
    DINT;

    /* 1. Arrêter la synchro des horloges ePWM                               */
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /* 2. Mettre à jour TBPRD dans le registre hardware                      */
    EPWM_setTimeBasePeriod(EPWM1_BASE, tbprd_new);

    /* 3. Remettre CMPA au milieu pour éviter une impulsion aberrante        */
    HWREG(EPWM1_BASE + EPWM_O_CMPA) = ((uint32_t)(tbprd_new / 2U) << 16U);

    /* 4. Redémarrer la synchro                                              */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /* 5. Mettre à jour les variables de l'ISR                               */
    TBPRD    = tbprd_new;
    isr_half = half_new;
    isr_amp  = amp;
    increm   = increm_new;
    accum    = 0U;   /* reset phase pour éviter un saut de phase             */

    EINT;
}

/*===========================================================================
 * MAIN
 *===========================================================================*/
void main(void)
{
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();

    /* Init avec les valeurs par défaut */
    /* eff_carrier : x4 pour que f_oscillo = f_saisie                       */
    float eff_carrier   = FREQ_CARRIER * 4.0f;
    uint16_t init_tbprd = calc_tbprd(eff_carrier);
    TBPRD    = init_tbprd;
    isr_half = (float)init_tbprd * 0.5f;
    isr_amp  = AMPLITUDE;
    increm   = calc_increm(FREQ_SINUS, FREQ_CARRIER);
    accum    = 0U;

    /* GPIO0 → ePWM1A */
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(0, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    /* LED debug GPIO31 */
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(31, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_writePin(31, 0);

    /* DAC-A */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);
    DAC_setLoadMode(DACA_BASE, DAC_LOAD_SYSCLK);
    DAC_enableOutput(DACA_BASE);
    DAC_setShadowValue(DACA_BASE, 0U);
    DEVICE_DELAY_US(200U);

    /* ePWM1 */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EPWM_setTimeBasePeriod(EPWM1_BASE, init_tbprd);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    EPWM_disableCounterCompareShadowLoadMode(EPWM1_BASE,
                                             EPWM_COUNTER_COMPARE_A);
    EPWM_setCounterCompareValue(EPWM1_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                init_tbprd / 2U);

    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U);
    EPWM_enableInterrupt(EPWM1_BASE);

    Interrupt_register(INT_EPWM1, &epwm1ISR);
    Interrupt_enable(INT_EPWM1);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EINT;
    ERTM;

    /*-----------------------------------------------------------------------
     * Boucle principale — détection de changement sur les 3 paramètres
     *---------------------------------------------------------------------*/
    float last_fc  = FREQ_CARRIER;
    float last_fs  = FREQ_SINUS;
    float last_amp = AMPLITUDE;

    while(1)
    {
        float cur_fc  = FREQ_CARRIER;
        float cur_fs  = FREQ_SINUS;
        float cur_amp = AMPLITUDE;

        if(cur_fc != last_fc || cur_fs != last_fs || cur_amp != last_amp)
        {
            /* eff_carrier : x4 pour que f_oscillo = f_saisie */
            float eff_carrier = cur_fc * 4.0f;
            apply_params(eff_carrier, cur_fc, cur_fs, cur_amp);
            last_fc  = cur_fc;
            last_fs  = cur_fs;
            last_amp = cur_amp;
        }
    }
}

/*===========================================================================
 * ISR ePWM1 — déclenchée à f_carrier (variable)
 *===========================================================================*/
__interrupt void epwm1ISR(void)
{
    isr_count++;

    float amp  = isr_amp;
    float half = isr_half;

    /* DDS */
    accum += increm;

    /* Phase normalisée [0, 1[ → TMU SINPUF32 */
    float phase = (float)accum * (1.0f / 4294967296.0f);
    dbg_phase = phase;

    float sin_val = __sinpuf32(phase);
    dbg_sin = sin_val;

    /* Mise à l'échelle CMPA */
    float cmpa_f = half * (1.0f + amp * sin_val);
    if(cmpa_f < 0.0f)                cmpa_f = 0.0f;
    if(cmpa_f > (float)(TBPRD))      cmpa_f = (float)(TBPRD);

    uint16_t cmpa_val = (uint16_t)cmpa_f;
    dbg_cmpa = cmpa_val;

    HWREG(EPWM1_BASE + EPWM_O_CMPA) = ((uint32_t)cmpa_val << 16U);

    /* DAC-A */
    uint16_t dac_val = (uint16_t)((uint32_t)cmpa_val * 4095U / (uint32_t)TBPRD);
    DAC_setShadowValue(DACA_BASE, dac_val);
    dbg_dac = dac_val;

    /* LED */
    if((isr_count & 0x1FFFU) == 0U)
        GPIO_togglePin(31);

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}