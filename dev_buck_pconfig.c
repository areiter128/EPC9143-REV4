/*
 * File:   drv_cuck_pconfig.c
 * Author: M91406
 *
 * Created on March 12, 2020, 4:31 PM
 */

#include "pwr_control/devices/dev_buck_pconfig.h"
#include "pwr_control/devices/dev_buck_typedef.h"

/* PRIVATE VARIABLES */
volatile uint16_t adcore_mask=0;
volatile uint16_t adcore_diff_mask=0;


volatile uint16_t buckPWM_ModuleInitialize(volatile BUCK_POWER_CONTROLLER_t* buckInstance)
{
    volatile uint16_t fres=1;
    volatile P33C_PWM_MODULE_t* pwm;
    
    pwm = (volatile P33C_PWM_MODULE_t*) ((volatile uint8_t*) &PCLKCON);
    
    // Make sure power to the peripheral is enabled
    PMD1bits.PWMMD = 0; // PWM Module Disable: PWM module is enabled
    
    // PWM GENERATOR ENABLE
    PG1CONLbits.ON = 0; // PWM Generator #1 Enable: PWM Generator is not enabled
    PG2CONLbits.ON = 0; // PWM Generator #2 Enable: PWM Generator is not enabled
    PG3CONLbits.ON = 0; // PWM Generator #3 Enable: PWM Generator is not enabled
    PG4CONLbits.ON = 0; // PWM Generator #4 Enable: PWM Generator is not enabled
    PG5CONLbits.ON = 0; // PWM Generator #5 Enable: PWM Generator is not enabled
    PG6CONLbits.ON = 0; // PWM Generator #6 Enable: PWM Generator is not enabled
    PG7CONLbits.ON = 0; // PWM Generator #7 Enable: PWM Generator is not enabled
    PG8CONLbits.ON = 0; // PWM Generator #8 Enable: PWM Generator is not enabled
    
    // PWM CLOCK CONTROL REGISTER
    pwm->PCLKCON = 0b0000000000000011;
    
//    PCLKCONbits.LOCK = 0;       // Lock bit: Write-protected registers and bits are unlocked
//    PCLKCONbits.DIVSEL = 0b00;  // PWM Clock Divider Selection: Divide ratio is 1:2
//    PCLKCONbits.MCLKSEL = 0b11; // PWM Master Clock Selection: Auxiliary PLL post-divider output
    
    // FREQUENCY SCALE REGISTER & FREQUENCY SCALING MINIMUM PERIOD REGISTER
    pwm->FSCL = 0x0000;      // Reset frequency scaling register
    pwm->FSMINPER = 0x0000;  // Reset frequency scaling minimum register
    
    // MASTER PHASE, DUTY CYCLE AND PERIOD REGISTERS
    pwm->MPHASE = 0;    // Reset master phase
    pwm->MDC = 0x0000;  // Reset master duty cycle
    pwm->MPER = 0x0000; // Reset Master period 
    
    // If buck converter has been configured in MASTER PERIOD mode
    if (buckInstance->sw_node.master_period)
        pwm->MPER = buckInstance->sw_node.period; // Set Period
    
    // LINEAR FEEDBACK SHIFT REGISTER
    pwm->LFSR = 0x0000;      // Reset linear feedback shift register
    
    // COMBINATIONAL TRIGGER REGISTERS
    pwm->CMBTRIGL = 0x0000;
    pwm->CMBTRIGH = 0x0000;

    // COMBINATORIAL PWM LOGIC A CONTROL REGISTERS A-F
    pwm->LOGCONA = 0x0000;
    pwm->LOGCONB = 0x0000;
    pwm->LOGCONC = 0x0000;
    pwm->LOGCOND = 0x0000;
    pwm->LOGCONE = 0x0000;
    pwm->LOGCONF = 0x0000;
    
    // PWM EVENT OUTPUT CONTROL REGISTERS A-F
    pwm->PWMEVTA = 0x0000;
    pwm->PWMEVTB = 0x0000;
    pwm->PWMEVTC = 0x0000;
    pwm->PWMEVTD = 0x0000;
    pwm->PWMEVTE = 0x0000;
    pwm->PWMEVTF = 0x0000;

    return(fres);    
}

volatile uint16_t buckPWM_VMC_Initialize(volatile BUCK_POWER_CONTROLLER_t* buckInstance)
{
    volatile uint16_t fres=1;


    volatile P33C_PWM_INSTANCE_t* pg;
    volatile P33C_GPIO_INSTANCE_t* gpio;
    volatile uint16_t pwm_Instance;
    volatile uint16_t gpio_Instance;

    // LOAD PERIPHERAL INSTANCES FROM BUCK CONVERTER OBJECT
    pwm_Instance = buckInstance->sw_node.pwm_instance;
    gpio_Instance = buckInstance->sw_node.gpio_instance;
    
    // CAPTURE MEMORY ADDRESS OF GIVEN PWM GENERATOR INSTANCE
    gpio = (volatile P33C_GPIO_INSTANCE_t*)((volatile struct P33C_GPIO_INSTANCE_t*) 
        ((volatile uint8_t*) &ANSELA + ((gpio_Instance - 1) * P33C_GPIO_SFR_OFFSET)));
    
    pg   = (volatile P33C_PWM_INSTANCE_t*) ((volatile struct P33C_GPIO_INSTANCE_t*) 
        ((volatile uint8_t*) &PG1CONL + ((pwm_Instance - 1) * P33C_PWM_SFR_OFFSET)));

    // WRITE GPIO CONFIGURATION OF PWM OUTPUT(S)
    gpio->LATx  &= ~(0x0001 << buckInstance->sw_node.gpio_high);  // Clear PWMxH output LOW
    gpio->LATx  &= ~(0x0001 << buckInstance->sw_node.gpio_low);   // Clear PWMxL output LOW
    gpio->TRISx &= ~(0x0001 << buckInstance->sw_node.gpio_high);  // Clear PWMxH output to OUTPUT
    gpio->TRISx &= ~(0x0001 << buckInstance->sw_node.gpio_low);   // Clear PWMxL output to OUTPUT
    gpio->CNPDx |= (0x0001 << buckInstance->sw_node.gpio_high); // Enable intern pull down register (PWM1H)
    gpio->CNPDx |= (0x0001 << buckInstance->sw_node.gpio_low); // Enable intern pull down register (PWM1L)
    
    // COPY CONFIGURATION FROM TEMPLATE TO PWM GENERATOR x CONTROL REGISTERS
    pg->PGxCONL = REG_PGxCONL; // PGxCONL: PWM GENERATOR x CONTROL REGISTER LOW
    pg->PGxCONH = REG_PGxCONH; // PGxCONH: PWM GENERATOR x CONTROL REGISTER HIGH
    pg->PGxIOCONL = REG_PGxIOCONL; // PGxIOCONL: PWM GENERATOR x I/O CONTROL REGISTER LOW
    pg->PGxIOCONH = REG_PGxIOCONH; // PGxIOCONL: PWM GENERATOR x I/O CONTROL REGISTER HIGH
    pg->PGxEVTL = REG_PGxEVTL; // PGxEVTL: PWM GENERATOR x EVENT REGISTER LOW
    pg->PGxEVTH = REG_PGxEVTH; // PGxEVTH: PWM GENERATOR x EVENT REGISTER HIGH
    pg->PGxCLPCIL = REG_PGxCLPCIL; // PGxLCPCIL: PWM GENERATOR x CURRENT LIMIT PCI REGISTER LOW
    pg->PGxCLPCIH = REG_PGxCLPCIH; // PGxLCPCIL: PWM GENERATOR x CURRENT LIMIT PCI REGISTER HIGH
    pg->PGxFPCIL = REG_PGxFPCIL; // PGxFPCIL: PWM GENERATOR x FAULT PCI REGISTER LOW
    pg->PGxFPCIH = REG_PGxFPCIH; // PGxFPCIL: PWM GENERATOR x FAULT PCI REGISTER HIGH
    pg->PGxFFPCIL = REG_PGxFFPCIL; // PGxFFPCIL: PWM GENERATOR x FEED FORWARD PCI REGISTER LOW
    pg->PGxFFPCIH = REG_PGxFFPCIH; // PGxFFPCIL: PWM GENERATOR x FEED FORWARD PCI REGISTER HIGH
    pg->PGxSPCIL = REG_PGxSPCIL; // PGxSPCIL: PWM GENERATOR x SOFTWARE PCI REGISTER LOW
    pg->PGxSPCIH = REG_PGxSPCIH; // PGxSPCIL: PWM GENERATOR x SOFTWARE PCI REGISTER HIGH
    pg->PGxLEBL = REG_PGxLEBL; // PGxLEBL: PWM GENERATOR x LEADING-EDGE BLANKING REGISTER LOW
    pg->PGxLEBH = REG_PGxLEBH; // PGxLEBL: PWM GENERATOR x LEADING-EDGE BLANKING REGISTER HIGH

    // LOAD PWM GENERATOR TIMING SETTINGS FROM BUCK CONVERTER OBJECT
    pg->PGxPHASE = buckInstance->sw_node.phase; // PGxPHASE: PWM GENERATOR x PHASE REGISTER
    pg->PGxDC = buckInstance->sw_node.duty_ratio_min; // PGxDC: PWM GENERATOR x DUTY CYCLE REGISTER
    pg->PGxDCA = REG_PGxDCA; // PGxDCA: PWM GENERATOR x DUTY CYCLE ADJUSTMENT REGISTER (not used)
    pg->PGxPER = buckInstance->sw_node.period; // PGxPER: PWM GENERATOR x PERIOD REGISTER
    pg->PGxTRIGA = 0; // PGxTRIGA: PWM GENERATOR x TRIGGER A REGISTER
    pg->PGxTRIGB = 0; // PGxTRIGB: PWM GENERATOR x TRIGGER B REGISTER
    pg->PGxTRIGC = 0; // PGxTRIGC: PWM GENERATOR x TRIGGER C REGISTER
    pg->PGxDTL = buckInstance->sw_node.dead_time_falling; // PGxDTL: PWM GENERATOR x DEAD-TIME REGISTER LOW
    pg->PGxDTH = buckInstance->sw_node.dead_time_rising; // PGxDTH: PWM GENERATOR x DEAD-TIME REGISTER HIGH
    pg->PGxLEBL = buckInstance->sw_node.leb_period; // PWM GENERATOR x LEADING-EDGE BLANKING REGISTER LOW 

    return(fres);    
}

volatile uint16_t buckPWM_Start(volatile uint16_t pwmInstance) 
{
    volatile uint16_t fres=1;
    volatile uint16_t timeout=0;
    volatile P33C_PWM_INSTANCE_t* pg;

    // CAPTURE MEMORY ADDRESS OF GIVEN PWM GENERATOR INSTANCE
    pg = (volatile P33C_PWM_INSTANCE_t*)((volatile struct P33C_PWM_INSTANCE_t*) 
        ((volatile uint8_t*) & PG1CONL + ((pwmInstance - 1) * P33C_PWM_SFR_OFFSET)));

    pg->PGxIOCONL |= P33C_PGxIOCONL_OVREN; // PWMxH/L Output Override Enable: PWM generator controls the PWMxH output pin
    pg->PGxIOCONH &= ~(P33C_PGxIOCONH_PEN); // PWMxH/L Output Port Disable: PWM generator controls the PWMxH output pin
    
    pg->PGxCONL |= P33C_PGxCONL_PWM_ON; // PWM Generator Enable: PWM Generator is enabled
    pg->PGxSTAT |= P33C_PGxSTAT_UPDREQ; // Update all PWM registers

    if(pg->PGxCONL & P33C_PGxCONL_HRES_EN) { // If high resolution is enabled
        while((!PCLKCONbits.HRRDY) && (timeout++ < 5000));  // wait for high resolution to get ready
        if ((timeout >= 5000) || (PCLKCONbits.HRERR))       // if there is an error
            return(0);  // return ERROR
    }
    
    pg->PGxIOCONH |= P33C_PGxIOCONH_PEN; // PWMxH/L Output Port Enable: PWM generator controls the PWMxH output pin

    return(fres);    
}

volatile uint16_t buckPWM_Stop(volatile uint16_t pwmInstance) 
{
    volatile uint16_t fres=1;
    volatile P33C_PWM_INSTANCE_t* pg;

    // CAPTURE MEMORY ADDRESS OF GIVEN PWM GENERATOR INSTANCE
    pg = (volatile P33C_PWM_INSTANCE_t*)((volatile struct P33C_PWM_INSTANCE_t*) 
            ((volatile uint8_t*) & PG1CONL + ((pwmInstance - 1) * P33C_PWM_SFR_OFFSET)));

    pg->PGxIOCONL |= P33C_PGxIOCONL_OVREN;  // PWMxH/L Output Override Enable
    pg->PGxIOCONH &= ~(P33C_PGxIOCONH_PEN); // PWMxH/L Output Pint Control Disable
    pg->PGxCONL &= ~(P33C_PGxCONL_PWM_ON);  // PWM Generator Disable
    pg->PGxDC = 0; // Reset Duty Cycle
    pg->PGxSTAT |= P33C_PGxSTAT_UPDREQ;     // Set the Update Request bit to update PWM timing

    return(fres);    
}

volatile uint16_t buckPWM_Suspend(volatile uint16_t pwmInstance) 
{
    volatile uint16_t fres=1;
    volatile P33C_PWM_INSTANCE_t* pg;

    // CAPTURE MEMORY ADDRESS OF GIVEN PWM GENERATOR INSTANCE
    pg = (volatile P33C_PWM_INSTANCE_t*)((volatile struct P33C_PWM_INSTANCE_t*) 
        ((volatile uint8_t*) & PG1CONL + ((pwmInstance - 1) * P33C_PWM_SFR_OFFSET)));
    pg->PGxIOCONL |= P33C_PGxIOCONL_OVREN; // PWMxH/L Output Override Enable
    pg->PGxDC = 0;  // Reset Duty Cycle
    pg->PGxSTAT |= P33C_PGxSTAT_UPDREQ; // Set the Update Request bit to update PWM timing

    fres &= (bool)(pg->PGxIOCONL & P33C_PGxIOCONL_OVREN);

    return(fres);    
}


volatile uint16_t buckPWM_Resume(volatile uint16_t pwmInstance) 
{
    volatile uint16_t fres=1;
    volatile P33C_PWM_INSTANCE_t* pg;

    // CAPTURE MEMORY ADDRESS OF GIVEN PWM GENERATOR INSTANCE
    pg = (volatile P33C_PWM_INSTANCE_t*)((volatile struct P33C_PWM_INSTANCE_t*) 
        ((volatile uint8_t*) & PG1CONL + ((pwmInstance - 1) * P33C_PWM_SFR_OFFSET)));
    
    pg->PGxSTAT |= P33C_PGxSTAT_UPDREQ; // Set the Update Request bit to update PWM timing
    pg->PGxIOCONL &= ~(P33C_PGxIOCONL_OVREN); // PWMxH/L Output Override Disable

    fres = (bool)(pg->PGxIOCONL & P33C_PGxIOCONL_OVREN);

    return(fres);    
}


volatile uint16_t buckADC_ModuleInitialize(void) 
{
    volatile uint16_t fres=1;
    
    // Make sure power to peripheral is enabled
    PMD1bits.ADC1MD = 0; // ADC Module Power Disable: ADC module power is enabled
    
    // ADCON1L: ADC CONTROL REGISTER 1 LOW
    ADCON1Lbits.ADON = 0; // ADC Enable: ADC module is off during configuration
    ADCON1Lbits.ADSIDL = 0; // ADC Stop in Idle Mode: Continues module operation in Idle mode
    
    // ADCON1H: ADC CONTROL REGISTER 1 HIGH
    ADCON1Hbits.SHRRES = 0b11; // Shared ADC Core Resolution Selection: 12-bit resolution ADC resolution = 12-bit (0...4095 ticks)
    ADCON1Hbits.FORM = 0; // Fractional Data Output Format: Integer

    // ADCON2L: ADC CONTROL REGISTER 2 LOW
    ADCON2Lbits.REFCIE = 0;; // Band Gap and Reference Voltage Ready Common Interrupt Enable: Common interrupt is disabled for the band gap ready event
    ADCON2Lbits.REFERCIE = 0; // Band Gap or Reference Voltage Error Common Interrupt Enable: Disabled
    ADCON2Lbits.EIEN = 1; // Early Interrupts Enable: The early interrupt feature is enabled
    ADCON2Lbits.PTGEN = 0; // External Conversion Request Interface: Disabled
    ADCON2Lbits.SHREISEL = 0b111; // Shared Core Early Interrupt Time Selection: Early interrupt is set and interrupt is generated 8 TADCORE clocks prior to when the data are ready
    ADCON2Lbits.SHRADCS = 0b0000001; // Shared ADC Core Input Clock Divider: 2:1 (minimum)

    // ADCON2H: ADC CONTROL REGISTER 2 HIGH
    ADCON2Hbits.SHRSAMC = 8; // Shared ADC Core Sample Time Selection: 8x TADs sampling time 
    ADCON2Hbits.REFERR = 0; // reset error flag
    ADCON2Hbits.REFRDY = 0; // reset bandgap status bit

    // ADCON3L: ADC CONTROL REGISTER 3 LOW
    ADCON3Lbits.REFSEL = 0b000; // ADC Reference Voltage Selection: AVDD-toAVSS
    ADCON3Lbits.SUSPEND = 0; // All ADC Core Triggers Disable: All ADC cores can be triggered
    ADCON3Lbits.SUSPCIE = 0; // Suspend All ADC Cores Common Interrupt Enable: Common interrupt is not generated for suspend ADC cores
    ADCON3Lbits.SUSPRDY = 0; // All ADC Cores Suspended Flag: ADC cores have previous conversions in progress
    ADCON3Lbits.SHRSAMP = 0; // Shared ADC Core Sampling Direct Control: use hardware trigger
    ADCON3Lbits.CNVRTCH = 0; // Software Individual Channel Conversion Trigger: Next individual channel conversion trigger can be generated (not used)
    ADCON3Lbits.SWLCTRG = 0; // Software Level-Sensitive Common Trigger: No software, level-sensitive common triggers are generated (not used)
    ADCON3Lbits.SWCTRG = 0; // Software Common Trigger: Ready to generate the next software common trigger (not used)
    ADCON3Lbits.CNVCHSEL = 0; // Channel Number Selection for Software Individual Channel Conversion Trigger: AN0 (not used)
    
    // ADCON3H: ADC CONTROL REGISTER 3 HIGH
    ADCON3Hbits.CLKSEL = 0b01; // ADC Module Clock Source Selection: AVCODIV
    ADCON3Hbits.CLKDIV = 0b000000; // ADC Module Clock Source Divider: 1 Source Clock Period
    ADCON3Hbits.SHREN = 0; // Shared ADC Core Enable: Shared ADC core is disabled
    ADCON3Hbits.C0EN = 0; // Dedicated ADC Core 0 Enable: Dedicated ADC Core 0 is disabled
    ADCON3Hbits.C1EN = 0; // Dedicated ADC Core 1 Enable: Dedicated ADC Core 1 is disabled
    
    // ADCON4L: ADC CONTROL REGISTER 4 LOW
    ADCON4Lbits.SAMC0EN = 0;  // Dedicated ADC Core 0 Conversion Delay Enable: Immediate conversion
    ADCON4Lbits.SAMC1EN = 0;  // Dedicated ADC Core 1 Conversion Delay Enable: Immediate conversion
    
    // ADCON4H: ADC CONTROL REGISTER 4 HIGH
    ADCON4Hbits.C0CHS = 0b00; // Dedicated ADC Core 0 Input Channel Selection: AN0
    ADCON4Hbits.C1CHS = 0b01; // Dedicated ADC Core 1 Input Channel Selection: ANA1

    // ADCON5L: ADC CONTROL REGISTER 5 LOW
    // ADCON5Lbits.SHRRDY: Shared ADC Core Ready Flag (read only)
    // ADCON5Lbits.C0RDY: Dedicated ADC Core 0 Ready Flag (read only)
    // ADCON5Lbits.C1RDY: Dedicated ADC Core 1 Ready Flag (read only)
    ADCON5Lbits.SHRPWR = 0; // Shared ADC Core Power Enable: ADC core is off
    ADCON5Lbits.C0PWR = 0; // Dedicated ADC Core 0 Power Enable: ADC core is off
    ADCON5Lbits.C1PWR = 0; // Dedicated ADC Core 1 Power Enable: ADC core is off
  
    // ADCON5H: ADC CONTROL REGISTER 5 HIGH
    ADCON5Hbits.WARMTIME = 0b1111; // ADC Dedicated Core x Power-up Delay: 32768 Source Clock Periods
    ADCON5Hbits.SHRCIE = 0; // Shared ADC Core Ready Common Interrupt Enable: Common interrupt is disabled for an ADC core ready event
    ADCON5Hbits.C0CIE = 0; // C1CIE: Dedicated ADC Core 0 Ready Common Interrupt Enable: Common interrupt is disabled
    ADCON5Hbits.C1CIE = 0; // C1CIE: Dedicated ADC Core 1 Ready Common Interrupt Enable: Common interrupt is disabled
    
    // ADCORExL: DEDICATED ADC CORE x CONTROL REGISTER LOW
    ADCORE1Lbits.SAMC = 0b0000000000;   // Dedicated ADC Core 1 Conversion Delay Selection: 2 TADCORE (minimum)
    ADCORE0Lbits.SAMC = 0b0000000000;   // Dedicated ADC Core 0 Conversion Delay Selection: 2 TADCORE (minimum)

    // ADCORExH: DEDICATED ADC CORE x CONTROL REGISTER HIGH
    ADCORE0Hbits.RES = 0b11; // ADC Core x Resolution Selection: 12 bit
    ADCORE0Hbits.ADCS = 0b0000000; // ADC Core x Input Clock Divider: 2 Source Clock Periods
    ADCORE0Hbits.EISEL = 0b111; // Early interrupt is set and an interrupt is generated 8 TADCORE clocks prior

    ADCORE1Hbits.RES = 0b11; // ADC Core x Resolution Selection: 12 bit
    ADCORE1Hbits.ADCS = 0b0000000; // ADC Core x Input Clock Divider: 2 Source Clock Periods
    ADCORE1Hbits.EISEL = 0b111; // Early interrupt is set and an interrupt is generated 8 TADCORE clocks prior
    
    return(fres);    
}

volatile uint16_t buckADC_Channel_Initialize(volatile BUCK_ADC_INPUT_SETTINGS_t* adcInstance) 
{
    volatile uint16_t fres=1;
    volatile uint8_t* ptrADCRegister;
    volatile uint8_t bit_offset;
    
    // Initialize ADC input registers
    if (adcInstance->enabled) {

        // Write level trigger setting
        if (adcInstance->adc_input < 16) {
            ADLVLTRGL |= ((uint16_t)(adcInstance->level_trigger) << adcInstance->adc_input);
            ADEIEL |= ((uint16_t)(adcInstance->early_interrupt_enable) << adcInstance->adc_input);
            ADIEL |= ((uint16_t)(adcInstance->interrupt_enable) << adcInstance->adc_input);
        }
        else if (adcInstance->adc_input < 32) {
            ADLVLTRGH |= ((uint16_t)(adcInstance->level_trigger) << (adcInstance->adc_input - 16));
            ADEIEH |= ((uint16_t)(adcInstance->early_interrupt_enable) << (adcInstance->adc_input - 16));
            ADIEH |= ((uint16_t)(adcInstance->interrupt_enable) << (adcInstance->adc_input - 16));
        }
        else {
            return(0); // ADC input number out of range
        }

        // write input mode setting
        ptrADCRegister = (volatile uint8_t *)((volatile uint8_t *)&ADMOD0L + (volatile uint8_t)(adcInstance->adc_input >> 8));
        if (adcInstance->adc_input < 8)
            bit_offset = (2 * adcInstance->adc_input);
        else if (adcInstance->adc_input < 16)
            bit_offset = (2 * (adcInstance->adc_input-8));
        else if (adcInstance->adc_input < 24)
            bit_offset = (2 * (adcInstance->adc_input-16));
        else if (adcInstance->adc_input < 32)
            bit_offset = (2 * (adcInstance->adc_input-24));
        else
            return(0); // ADC input number out of range
        
        *ptrADCRegister |= ((unsigned int)adcInstance->signed_result << bit_offset);
        *ptrADCRegister |= ((unsigned int)adcInstance->differential_input << (bit_offset + 1));
       
        // Write ADC trigger source setting
        ptrADCRegister = (volatile uint8_t *)((volatile uint8_t *)&ADTRIG0L + adcInstance->adc_input);
        *ptrADCRegister = adcInstance->trigger_source;
        
        // Register ADC core to be active
        switch (adcInstance->adc_core) {
            case 0:
                adcore_mask |= ADC_CORE0_MASK_INDEX;
                if (adcInstance->differential_input)
                    adcore_diff_mask |= ADC_CORE0_MASK_INDEX;
                break;
            case 1:
                adcore_mask |= ADC_CORE1_MASK_INDEX;
                if (adcInstance->differential_input)
                    adcore_diff_mask |= ADC_CORE1_MASK_INDEX;
                break;
            case 2:
                adcore_mask |= ADC_CORE2_MASK_INDEX;
                if (adcInstance->differential_input)
                    adcore_diff_mask |= ADC_CORE2_MASK_INDEX;
                break;
            case 3:
                adcore_mask |= ADC_CORE3_MASK_INDEX;
                if (adcInstance->differential_input)
                    adcore_diff_mask |= ADC_CORE3_MASK_INDEX;
                break;
            default:
                adcore_mask |= ADC_SHRCORE_MASK_INDEX;
                if (adcInstance->differential_input)
                    adcore_diff_mask |= ADC_SHRCORE_MASK_INDEX;
                break;
        }
        
    }
    
    return(fres);
}

volatile uint16_t buckADC_Start(void) 
{
    volatile uint16_t fres=1;
    volatile uint16_t timeout=0;
    volatile uint16_t adcore_mask_compare=0;
    
    // Turn on ADC module
    ADCON1Lbits.ADON = 1;

    ADCON5L = adcore_mask;    // Enable power to all used ADC cores
    adcore_mask_compare = ((adcore_mask << 8) | adcore_mask); // Set ADC Core Ready Bit Mask
    
    while ((ADCON5L != adcore_mask_compare) & (timeout++ < ADC_POWRUP_TIMEOUT)); // Wait until ADC cores are ready
    if (timeout >= ADC_POWRUP_TIMEOUT) return(0); // Skip if powering up ADC cores was unsuccessful
    ADCON3H = adcore_mask; // Enable ADC cores


    return(fres);    
}

