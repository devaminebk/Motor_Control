/******************************************************************************
 * SPWM HARDWARE-DRIVEN — TMS320F28379D LaunchPad
 * V4 : AJOUT DU CONTRÔLE TRIPHASÉ
 *      - 3 bras (ePWM1/2/3) avec porteuse triangulaire COMMUNE et synchronisée
 *      - modulantes sinusoïdales déphasées de 0° / 120° / 240°
 *      - f_carrier + f_sinus + amplitude restent modifiables via Watch Window
 * V5 : AJOUT DU DEAD-BAND (temps mort)
 *      - sous-module DB activé sur EPWM1/2/3, mode AHC (Active High Complementary)
 *      - EPWMxA = signal SPWM original, EPWMxB = complémentaire inversé + retard
 *      - DEADBAND_NS modifiable en live (ns), RED = FED, commun aux 3 bras
 *      - GPIO1/3/5 ajoutés (EPWM1B/2B/3B) pour sortir le bras complémentaire
 *
 * FIX (deadband) :
 *      - DB_RISE_FALL était initialisé à 11 (DECIMAL onze), ce qui ne
 *        correspond à AUCUN des cas testés par apply_deadband() (0,1,2,
 *        sinon "both"). Toute valeur != 0,1,2 retombe dans le "else" =
 *        RED+FED actifs => explique pourquoi 01/10 "agissaient comme 11".
 *        -> valeur par défaut corrigée à 3 (0b11 = RED+FED), et il faut
 *           écrire des valeurs DÉCIMALES (0, 1, 2, 3) dans la Watch
 *           Window, PAS des littéraux "0b01"/"0b10" (non supportés par
 *           CCS Watch Window).
 *      - EPWM_DB_COUNTER_CLOCK_FULL_CYCLE faisait tourner le compteur DB
 *        à TBCLK/2 : chaque compte valait donc 10 ns et non 5 ns, et
 *        comme RED+FED sont tous deux affectés, l'écart mesuré entre les
 *        deux signaux complémentaires ressortait à 4x le réglage voulu
 *        au lieu de 2x. -> corrigé en EPWM_DB_COUNTER_CLOCK_HALF_CYCLE
 *        (1 compte = 1 période TBCLK = 5 ns @ 200 MHz), sur les 3 bras.
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
 *   DEADBAND_NS   : 0 … 5 000 ns — temps mort RED = FED, commun aux 3 bras
 *
 * Ces 4 paramètres sont COMMUNS aux 3 phases (système triphasé équilibré).
 * Seul le déphasage (0° / 120° / 240°) diffère entre les bras.
 *===========================================================================*/
 /* Hz — porteuse triangulaire     */
volatile float FREQ_CARRIER = 4000.0f;
volatile float FREQ_SINUS   =  50.0f;   /* Hz — modulante sinusoïdale     */
volatile float AMPLITUDE    =  0.80f;     /* 0.0 … 1.0 — indice modulation  */

/*===========================================================================
 * AJOUT DEAD-BAND — temps mort commun aux 3 bras (RED = FED = DEADBAND_NS)
 *   Modifiable en live via Watch Window, comme les 3 paramètres ci-dessus.
 *   SYSCLK = 200 MHz, pas de prescaler sur EPWMCLK, mode HALF_CYCLE
 *   => 1 compte DB = 5 ns (FIX : voir note en tête de fichier).
 *   Plage raisonnable : 0 … 5000 ns (0 … 1000 comptes).
 *===========================================================================*/
volatile float DEADBAND_NS = 1000.0f;   /* ns — temps mort RED = FED       */

/* FIX : DB_RISE_FALL doit être une valeur DÉCIMALE parmi :
 *   0 -> aucun deadband (bypass RED et FED)
 *   1 -> FED seul (0b01)
 *   2 -> RED seul (0b10)
 *   3 -> RED + FED (0b11) -- ou toute autre valeur, cf. "else" de
 *        apply_deadband()
 * Écrire ces valeurs en DÉCIMAL dans la Watch Window (1, 2, 3...),
 * jamais en notation binaire "0b01"/"0b10" : CCS ne la supporte pas
 * et l'ancienne valeur par défaut 11 (= onze en décimal) ne correspond à
 * aucun cas testé explicitement, ce qui la faisait retomber dans le
 * "else" (RED+FED actifs) -- d'où le bug "01/10 se comportent comme 11".
 */
volatile int DB_RISE_FALL = 3 ;


volatile int APPLY_THIRD_H = 1;
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
volatile uint16_t db_count   = 200U;      /* AJOUT DEAD-BAND : DEADBAND_NS en comptes TBCLK (5 ns/compte, mode HALF_CYCLE) */

/*===========================================================================
 * DEBUG — Watch Window CCS
 *===========================================================================*/
volatile uint32_t isr_count  = 0U;
volatile uint16_t dbg_cmpa   = 0U;        /* Phase A */
volatile uint16_t dbg_cmpa_b = 0U;        /* Phase B — AJOUT TRIPHASÉ */
volatile uint16_t dbg_cmpa_c = 0U;        /* Phase C — AJOUT TRIPHASÉ */
volatile uint16_t dbg_dac    = 0U;
volatile uint16_t dbg_db_count = 0U;      /* AJOUT DEAD-BAND : comptes RED/FED réellement chargés */
volatile float    dbg_phase  = 0.0f;
volatile float    dbg_sin    = 0.0f;      /* Phase A */
volatile float    dbg_sin_b  = 0.0f;      /* Phase B — AJOUT TRIPHASÉ */
volatile float    dbg_sin_c  = 0.0f;      /* Phase C — AJOUT TRIPHASÉ */

volatile uint32_t dbctl;
volatile uint32_t dbred;
volatile uint32_t dbfed;

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

/* AJOUT DEAD-BAND : ns → comptes TBCLK                                       */
/* SYSCLK = 200 MHz, EPWMCLK = SYSCLK (pas de prescaler), mode HALF_CYCLE     */
/* => 1 compte = 5 ns (FIX : anciennement FULL_CYCLE => 10 ns/compte réel,    */
/*    ce qui doublait chaque retard et faisait apparaître un écart x4)       */
/* Registre RED/FED sur 10 bits utiles → max 1023 comptes ≈ 5115 ns          */
static uint16_t calc_db_count(float ns)
{
    float counts_f = ns / 5.0f;             /* 1 compte TBCLK = 5 ns @200MHz (HALF_CYCLE) */
    if(counts_f < 0.0f)    counts_f = 0.0f;
    if(counts_f > 1000.0f) counts_f = 1000.0f; /* garde-fou ~5 µs max        */
    return (uint16_t)counts_f;
}

/* AJOUT DEAD-BAND : applique RED=FED=db sur les 3 bras (mode AHC)            */
/* RISE_FALL attendu en DÉCIMAL : 0 = bypass, 1 = FED seul, 2 = RED seul,     */
/* 3 (ou autre) = RED+FED. Ne PAS utiliser de notation binaire "0bxx".        */
static void apply_deadband(uint16_t db, uint8_t RISE_FALL)
{
    uint32_t epwm_bases[] = {EPWM1_BASE, EPWM2_BASE, EPWM3_BASE};

    int i = 0;

    for ( i = 0; i < 3; i++) 
    {
        uint32_t base = epwm_bases[i];

        if (RISE_FALL == 0) // 0: No Deadband (Bypass)
        {
            EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, false); // Disable Rising Edge Delay
            EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, false); // Disable Falling Edge Delay
        } 
        else if (RISE_FALL == 1) // 1 (0b01): Falling Edge Delay (FED) Only
        {
            EPWM_setFallingEdgeDelayCount(base, db);
            EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, false); // Disable RED
            EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);  // Enable FED
        } 
        else if (RISE_FALL == 2) // 2 (0b10): Rising Edge Delay (RED) Only
        {
            EPWM_setRisingEdgeDelayCount(base, db);
            EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);  // Enable RED
            EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, false); // Disable FED
        } 
        else // 3 (0b11) or anything else: Both RED and FED Enabled
        {
            EPWM_setRisingEdgeDelayCount(base, db);
            EPWM_setFallingEdgeDelayCount(base, db);
            EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, true);  // Enable RED
            EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, true);  // Enable FED
        }
    }
}

/* Recalcul complet — appelé depuis la boucle principale                      */
/* fc     : fréquence effective (fc_raw * 4) pour TBPRD                      */
/* fc_raw : fréquence saisie par l'utilisateur pour le ratio DDS             */
/* db_ns  : AJOUT DEAD-BAND — temps mort RED=FED en nanosecondes             */
static void apply_params(float fc, float fc_raw, float fs, float amp, float db_ns, int RISE_FALL)
{
    uint16_t tbprd_new  = calc_tbprd(fc);
    uint32_t increm_new = calc_increm(fs, fc_raw);
    float    half_new   = (float)tbprd_new * 0.5f;
    uint16_t db_new      = calc_db_count(db_ns);   /* AJOUT DEAD-BAND */

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

    /* 3bis. AJOUT DEAD-BAND : recharger RED/FED sur les 3 bras              */
    apply_deadband(db_new, RISE_FALL);

    /* 4. Redémarrer la synchro (les 3 bras repartent alignés)               */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);


    /* 5. Mettre à jour les variables de l'ISR                               */
    TBPRD    = tbprd_new;
    isr_half = half_new;
    isr_amp  = amp;
    increm   = increm_new;
    accum    = 0U;   /* reset phase pour éviter un saut de phase             */
    db_count = db_new;   /* AJOUT DEAD-BAND */
    dbg_db_count = db_new;   /* AJOUT DEAD-BAND */

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
    db_count = calc_db_count(DEADBAND_NS);   /* AJOUT DEAD-BAND */
    dbg_db_count = db_count;                 /* AJOUT DEAD-BAND */

    /* GPIO0 → ePWM1A (Phase A) */
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(0, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    /* GPIO1 → ePWM1B (Phase A, bras complémentaire) — AJOUT DEAD-BAND */
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(1, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

    /* GPIO2 → ePWM2A (Phase B) — AJOUT TRIPHASÉ */
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(2, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_2_EPWM2A);

    /* GPIO3 → ePWM2B (Phase B, bras complémentaire) — AJOUT DEAD-BAND */
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(3, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_3_EPWM2B);

    /* GPIO4 → ePWM3A (Phase C) — AJOUT TRIPHASÉ */
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(4, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_4_EPWM3A);

    /* GPIO5 → ePWM3B (Phase C, bras complémentaire) — AJOUT DEAD-BAND */
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(5, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_5_EPWM3B);

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

    /* AJOUT DEAD-BAND : sous-module DB d'EPWM1 — mode AHC (Active High
     * Complementary). EPWMB est généré à partir d'EPWMA (inversé), avec
     * un retard RED à la montée et FED à la descente, pour créer le temps
     * mort entre les deux interrupteurs complémentaires du bras A.        */
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    /* FIX : HALF_CYCLE (pas FULL_CYCLE) => 1 compte DB = 1 période TBCLK
     * = 5 ns @200MHz. FULL_CYCLE faisait tourner le compteur DB 2x plus
     * lentement (1 compte = 2 périodes TBCLK = 10 ns), ce qui doublait
     * silencieusement chaque retard RED/FED et donnait un écart x4 au lieu
     * de x2 entre EPWMxA et EPWMxB une fois RED+FED combinés. */
    EPWM_setDeadBandCounterClock(EPWM1_BASE, EPWM_DB_COUNTER_CLOCK_HALF_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM1_BASE, db_count);
    EPWM_setFallingEdgeDelayCount(EPWM1_BASE, db_count);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);

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

    /* AJOUT DEAD-BAND : sous-module DB d'EPWM2 — mode AHC, bras B           */
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    /* FIX : voir commentaire sur EPWM1_BASE ci-dessus */
    EPWM_setDeadBandCounterClock(EPWM2_BASE, EPWM_DB_COUNTER_CLOCK_HALF_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM2_BASE, db_count);
    EPWM_setFallingEdgeDelayCount(EPWM2_BASE, db_count);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);

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

    /* AJOUT DEAD-BAND : sous-module DB d'EPWM3 — mode AHC, bras C           */
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED,
                                  EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED,
                                  EPWM_DB_POLARITY_ACTIVE_LOW);
    /* FIX : voir commentaire sur EPWM1_BASE ci-dessus */
    EPWM_setDeadBandCounterClock(EPWM3_BASE, EPWM_DB_COUNTER_CLOCK_HALF_CYCLE);
    EPWM_setRisingEdgeDelayCount(EPWM3_BASE, db_count);
    EPWM_setFallingEdgeDelayCount(EPWM3_BASE, db_count);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);

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
    float last_fc      = FREQ_CARRIER;
    float last_fs      = FREQ_SINUS;
    float last_amp     = AMPLITUDE;
    float last_db      = DEADBAND_NS;   /* AJOUT DEAD-BAND */
    int   last_RISE_FALL = DB_RISE_FALL;

    while(1)
    {
        float cur_fc  = FREQ_CARRIER;
        float cur_fs  = FREQ_SINUS;
        float cur_amp = AMPLITUDE;
        float cur_db  = DEADBAND_NS;   /* AJOUT DEAD-BAND */
        int   curr_RISE_FALL = DB_RISE_FALL;

        if(cur_fc != last_fc || cur_fs != last_fs || cur_amp != last_amp
           || cur_db != last_db || curr_RISE_FALL != last_RISE_FALL )
        { 
            /* eff_carrier : x4 pour que f_oscillo = f_saisie */
            float eff_carrier = cur_fc * 4.0f;
            apply_params(eff_carrier, cur_fc, cur_fs, cur_amp, cur_db, curr_RISE_FALL);
            last_fc  = cur_fc;
            last_fs  = cur_fs;
            last_amp = cur_amp;
            last_db  = cur_db;   /* AJOUT DEAD-BAND */
            last_RISE_FALL = curr_RISE_FALL;
        }

        dbctl = HWREG(EPWM1_BASE + EPWM_O_DBCTL);
        dbred = HWREG(EPWM1_BASE + EPWM_O_DBRED);
        dbfed = HWREG(EPWM1_BASE + EPWM_O_DBFED);
        /* Export these to Watch Window or send over UART for inspection */
    }
}

/*===========================================================================
 * ISR ePWM1 — déclenchée à f_carrier (variable)
 * V6 : AVEC INJECTION DE TROISIÈME HARMONIQUE (THIPWM)
 *===========================================================================*/
__interrupt void epwm1ISR(void)
{
    isr_count++;

    float amp  = isr_amp;
    float half = isr_half;

    /* DDS — accumulateur de phase commun aux 3 bras */
    accum += increm;

    /* Phase A = accum, Phase B = accum + 120°, Phase C = accum + 240° */
    float phaseA = (float)accum * (1.0f / 4294967296.0f);
    float phaseB = (float)(accum + PHASE_OFFSET_120) * (1.0f / 4294967296.0f);
    float phaseC = (float)(accum + PHASE_OFFSET_240) * (1.0f / 4294967296.0f);
    dbg_phase = phaseA;

    /* Calcul des fondamentales */
    float sinA = __sinpuf32(phaseA);
    float sinB = __sinpuf32(phaseB);
    float sinC = __sinpuf32(phaseC);

    /* --- AJOUT INJECTION 3ÈME HARMONIQUE --- */
    /* On multiplie l'accumulateur par 3 dans le domaine entier 
       pour bénéficier du rebouclage automatique à 360° (overflow 32-bit) */
    
    uint32_t accum3 = 3U * accum;
    float phase3 = (float)accum3 * (1.0f / 4294967296.0f);
    float sin3 = __sinpuf32(phase3);
    
    /* V_inj = 1/6 * sin(3*theta). À 90° (sinA=1), sin3 vaut -1, 
       ce qui donne 1 + 1/6*(-1) = 5/6, écrasant ainsi la bosse du signal. */
    volatile float injection ;

    if (amp<0)  amp = 0;

    float tab[3] = {sinA, sinB, sinC};

    float min = sinA ;
    float max = sinA ;

    int i = 0;

    for ( i = 0 ; i < 3 ; i++)
    {
        if (tab[i] < min) min = tab[i] ;
        if (tab[i] > max) max = tab[i] ;
    }

    switch  (APPLY_THIRD_H) {
        case 0 :
        injection = 0 ;
        break;
        case 1 :
        injection = 0.1666667 * sin3 ;
        break;
        case 2 :
        injection = - 0.5f * (max + min) ;
        break;
    }

    /* Application du signal modulant modifié aux 3 phases */
    float modA = amp * sinA + injection;
    float modB = amp * sinB + injection;
    float modC = amp * sinC + injection;

    /* Mise à jour des variables de debug pour la Watch Window */
    dbg_sin   = modA;
    dbg_sin_b = modB;
    dbg_sin_c = modC;

    

    /* Mise à l'échelle CMPA — calcul basé sur la modulante modifiée */
    float cmpaA_f = half * (1.0f + modA);
    float cmpaB_f = half * (1.0f + modB);
    float cmpaC_f = half * (1.0f + modC);


    /* Saturation de sécurité (anti-overmodulation matérielle) */
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

    /* Sorties DAC de visualisation (reflètent désormais la forme "selle de cheval") */
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