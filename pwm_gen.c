/******************************************************************************
 * SPWM HARDWARE-DRIVEN — TMS320F28379D LaunchPad
 * V4 : AJOUT DU CONTRÔLE TRIPHASÉ
 *      - 3 bras (ePWM1/2/3) avec porteuse triangulaire COMMUNE et synchronisée
 *      - modulantes sinusoïdales déphasées de 0° / 120° / 240°
 *      - f_carrier + f_sinus + amplitude restent modifiables via Watch Window
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
 *
 * Ces 3 paramètres sont COMMUNS aux 3 phases (système triphasé équilibré).
 * Seul le déphasage (0° / 120° / 240°) diffère entre les bras.
 *===========================================================================*/
 /* Hz — porteuse triangulaire     */
volatile float FREQ_CARRIER = 4000.0f;
volatile float FREQ_SINUS   =  50.0f;   /* Hz — modulante sinusoïdale     */
volatile float AMPLITUDE    =  0.80f;     /* 0.0 … 1.0 — indice modulation  */

/*===========================================================================
 * AJOUT TRIPHASÉ — déphasages fixes en unités de phase normalisée 32 bits
 *   2^32 / 3      ≈ 120°
 *   2 × 2^32 / 3  ≈ 240°
 *===========================================================================*/
#define PHASE_OFFSET_120   (1431655765UL)
#define PHASE_OFFSET_240   (2863311531UL)

/*===========================================================================
 * ÉTAT INTERNE — calculé à chaque changement de paramètre
 *===========================================================================*/
volatile uint16_t TBPRD      = 5000U;     /* déduit de FREQ_CARRIER — commun aux 3 bras */
volatile uint32_t increm     = 0U;
volatile uint32_t accum      = 0U;        /* accumulateur de phase — référence Phase A  */
volatile float    isr_amp    = 0.80f;     /* snapshot atomique pour l'ISR   */
volatile float    isr_half   = 2500.0f;   /* TBPRD/2 pour l'ISR             */

/*===========================================================================
 * DEBUG — Watch Window CCS
 *===========================================================================*/
volatile uint32_t isr_count  = 0U;
volatile uint16_t dbg_cmpa   = 0U;        /* Phase A */
volatile uint16_t dbg_cmpa_b = 0U;        /* Phase B — AJOUT TRIPHASÉ */
volatile uint16_t dbg_cmpa_c = 0U;        /* Phase C — AJOUT TRIPHASÉ */
volatile uint16_t dbg_dac    = 0U;
volatile float    dbg_phase  = 0.0f;
volatile float    dbg_sin    = 0.0f;      /* Phase A */
volatile float    dbg_sin_b  = 0.0f;      /* Phase B — AJOUT TRIPHASÉ */
volatile float    dbg_sin_c  = 0.0f;      /* Phase C — AJOUT TRIPHASÉ */

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

    /* 1. Arrêter la synchro des horloges ePWM (affecte les 3 bras)          */
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /* 2. Mettre à jour TBPRD dans les 3 registres matériels                 */
    /*    AJOUT TRIPHASÉ : la porteuse est commune aux 3 bras ePWM1/2/3      */
    EPWM_setTimeBasePeriod(EPWM1_BASE, tbprd_new);
    EPWM_setTimeBasePeriod(EPWM2_BASE, tbprd_new);
    EPWM_setTimeBasePeriod(EPWM3_BASE, tbprd_new);

    /* 3. Remettre les 3 CMPA au milieu pour éviter une impulsion aberrante  */
    HWREG(EPWM1_BASE + EPWM_O_CMPA) = ((uint32_t)(tbprd_new / 2U) << 16U);
    HWREG(EPWM2_BASE + EPWM_O_CMPA) = ((uint32_t)(tbprd_new / 2U) << 16U);
    HWREG(EPWM3_BASE + EPWM_O_CMPA) = ((uint32_t)(tbprd_new / 2U) << 16U);

    /* 4. Redémarrer la synchro (les 3 bras repartent alignés)               */
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

    /* GPIO0 → ePWM1A (Phase A) */
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(0, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    /* GPIO2 → ePWM2A (Phase B) — AJOUT TRIPHASÉ */
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(2, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_2_EPWM2A);

    /* GPIO4 → ePWM3A (Phase C) — AJOUT TRIPHASÉ */
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(4, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_4_EPWM3A);

    /* LED debug GPIO31 */
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(31, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_writePin(31, 0);

    /* DAC-A / DAC-B / DAC-C — une sortie analogique par phase pour visu     */
    /* AJOUT TRIPHASÉ : DAC-B et DAC-C en plus de DAC-A                      */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACC);

    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);
    DAC_setReferenceVoltage(DACC_BASE, DAC_REF_ADC_VREFHI);

    DAC_setLoadMode(DACA_BASE, DAC_LOAD_SYSCLK);
    DAC_setLoadMode(DACB_BASE, DAC_LOAD_SYSCLK);
    DAC_setLoadMode(DACC_BASE, DAC_LOAD_SYSCLK);

    DAC_enableOutput(DACA_BASE);
    DAC_enableOutput(DACB_BASE);
    DAC_enableOutput(DACC_BASE);

    DAC_setShadowValue(DACA_BASE, 0U);
    DAC_setShadowValue(DACB_BASE, 0U);
    DAC_setShadowValue(DACC_BASE, 0U);
    DEVICE_DELAY_US(200U);

    /* ePWM1 / ePWM2 / ePWM3 */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);   /* AJOUT TRIPHASÉ */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);   /* AJOUT TRIPHASÉ */
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /* --- EPWM1 : Phase A, MAÎTRE de la synchro et de l'interruption --- */
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

    /* AJOUT TRIPHASÉ : EPWM1 émet un SYNCO à chaque passage à zéro du
     * compteur — c'est ce pulse qui garde EPWM2/EPWM3 alignés sur EPWM1.   */
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);

    /* --- EPWM2 : Phase B, esclave synchronisé — AJOUT TRIPHASÉ --- */
    EPWM_setTimeBasePeriod(EPWM2_BASE, init_tbprd);
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0U);
    EPWM_setClockPrescaler(EPWM2_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    EPWM_disableCounterCompareShadowLoadMode(EPWM2_BASE,
                                             EPWM_COUNTER_COMPARE_A);
    EPWM_setCounterCompareValue(EPWM2_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                init_tbprd / 2U);

    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    /* Aucun déphasage du COMPTEUR : la porteuse triangulaire reste
     * identique sur les 3 bras, seule la sinusoïde de référence (calculée
     * dans l'ISR) est déphasée de 120°/240°.                               */
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setPhaseShift(EPWM2_BASE, 0U);
    /* Relaie le SYNCO reçu d'EPWM1 vers EPWM3 (chaînage de la synchro) */
    EPWM_setSyncOutPulseMode(EPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);

    /* --- EPWM3 : Phase C, esclave synchronisé — AJOUT TRIPHASÉ --- */
    EPWM_setTimeBasePeriod(EPWM3_BASE, init_tbprd);
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    EPWM_setClockPrescaler(EPWM3_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    EPWM_disableCounterCompareShadowLoadMode(EPWM3_BASE,
                                             EPWM_COUNTER_COMPARE_A);
    EPWM_setCounterCompareValue(EPWM3_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                init_tbprd / 2U);

    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_enablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setPhaseShift(EPWM3_BASE, 0U);

    /* Seul EPWM1 déclenche l'interruption : EPWM2 et EPWM3 sont pilotés
     * directement depuis l'ISR EPWM1 (un seul accumulateur de phase,
     * pas de risque de dérive/jitter entre les 3 bras).                    */
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
 * AJOUT TRIPHASÉ : calcule et applique les 3 sinusoïdes (0°/120°/240°)
 * à partir d'un unique accumulateur de phase DDS.
 *===========================================================================*/
__interrupt void epwm1ISR(void)
{
    isr_count++;

    float amp  = isr_amp;
    float half = isr_half;

    /* DDS — accumulateur de phase commun aux 3 bras */
    accum += increm;

    /* Phase A = accum, Phase B = accum + 120°, Phase C = accum + 240°
     * (le débordement uint32_t fait naturellement le modulo 360°)         */
    float phaseA = (float)accum * (1.0f / 4294967296.0f);
    float phaseB = (float)(accum + PHASE_OFFSET_120) * (1.0f / 4294967296.0f);
    float phaseC = (float)(accum + PHASE_OFFSET_240) * (1.0f / 4294967296.0f);
    dbg_phase = phaseA;

    float sinA = __sinpuf32(phaseA);
    float sinB = __sinpuf32(phaseB);
    float sinC = __sinpuf32(phaseC);
    dbg_sin   = sinA;
    dbg_sin_b = sinB;
    dbg_sin_c = sinC;

    /* Mise à l'échelle CMPA — un calcul indépendant par bras */
    float cmpaA_f = half * (1.0f + amp * sinA);
    float cmpaB_f = half * (1.0f + amp * sinB);
    float cmpaC_f = half * (1.0f + amp * sinC);

    if(cmpaA_f < 0.0f)           cmpaA_f = 0.0f;
    if(cmpaA_f > (float)TBPRD)   cmpaA_f = (float)TBPRD;
    if(cmpaB_f < 0.0f)           cmpaB_f = 0.0f;
    if(cmpaB_f > (float)TBPRD)   cmpaB_f = (float)TBPRD;
    if(cmpaC_f < 0.0f)           cmpaC_f = 0.0f;
    if(cmpaC_f > (float)TBPRD)   cmpaC_f = (float)TBPRD;

    uint16_t cmpaA_val = (uint16_t)cmpaA_f;
    uint16_t cmpaB_val = (uint16_t)cmpaB_f;
    uint16_t cmpaC_val = (uint16_t)cmpaC_f;
    dbg_cmpa   = cmpaA_val;
    dbg_cmpa_b = cmpaB_val;
    dbg_cmpa_c = cmpaC_val;

    HWREG(EPWM1_BASE + EPWM_O_CMPA) = ((uint32_t)cmpaA_val << 16U);
    HWREG(EPWM2_BASE + EPWM_O_CMPA) = ((uint32_t)cmpaB_val << 16U);
    HWREG(EPWM3_BASE + EPWM_O_CMPA) = ((uint32_t)cmpaC_val << 16U);

    /* DAC-A / DAC-B / DAC-C — une sortie analogique par phase */
    uint16_t dacA_val = (uint16_t)((uint32_t)cmpaA_val * 4095U / (uint32_t)TBPRD);
    uint16_t dacB_val = (uint16_t)((uint32_t)cmpaB_val * 4095U / (uint32_t)TBPRD);
    uint16_t dacC_val = (uint16_t)((uint32_t)cmpaC_val * 4095U / (uint32_t)TBPRD);
    DAC_setShadowValue(DACA_BASE, dacA_val);
    DAC_setShadowValue(DACB_BASE, dacB_val);
    DAC_setShadowValue(DACC_BASE, dacC_val);
    dbg_dac = dacA_val;

    /* LED */
    if((isr_count & 0x1FFFU) == 0U)
        GPIO_togglePin(31);

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}