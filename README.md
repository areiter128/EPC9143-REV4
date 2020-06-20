<p><a href="https://www.microchip.com" rel="nofollow"><img src="https://www.microchip.com/ResourcePackages/Microchip/assets/dist/images/logo.png" alt="Microchip Technology" style="max-width:100%;"></a></p>

## EPC9143 16th Brick Non-Isolated Step Down Converter Firmware
**2-Phase Synchronous Buck Converter with Advanced Voltage Mode Control and Current Balancing**

**Description:**
This code example demonstrates a closed loop voltage mode control implementation for dsPIC33CK. It has specifically been developed for the EPC9143 Rev4.0 1/16 brick converter.

The board starts up the buck converter automatically when power is applied to the board, providing a regulated output voltage of 12 V at the output of the converter. The startup procedure is controlled and executed by the power controller state machine (PWRLIB_BUCK) and includes an configurable startup procedure with power-on delay, ramp up period and power good delay before dropping into constant regulation mode.
An additional fault handler (FUNLIB_FAULT_HANDLER) shuts down the power supply if the input voltage is outside the defined range of 18V to 72.5V (UVLO/OVLO) or if the output voltage is more than 0.5V out of regulation for more than 10 milliseconds.

The control loop is based on a digital type IV compensator (4P4Z controller).


**Operation Parameters:**
  * Input Voltage: 18 V to 72.5 V
  * Output Voltage: 12.0 V
  * Switching Frequency:	500 kHz
  * Control Frequency:	500 kHz
  * Cross-Over Frequency: ~20 kHz (depends on VIN and if AGC is on/off)
  * Phase Margin: ~ 50°

**Required Hardware:**
  * EPC9143: EPC9143 16th Brick Non-Isolated Step Down Converter
  * Digital Power Libraries System Blocks used:
    * Buck Converter

**Device Support:**
  * [dsPIC33CK32MP102](https://www.microchip.com/dsPIC33CK32MP102)

**Libraries:**
  * [PWRLIB_BUCK](https://bitbucket.microchip.com/projects/MCU16ASMPSL/repos/pwrlib_buck/browse) - Single Phase Buck Converter State Machine & Peripheral Configuration Template
  * [FUNLIB_FAULT_HANDLER](https://bitbucket.microchip.com/projects/MCU16ASMPSL/repos/funlib_faulthandler/browse) - Generic Fault Handler Template

**Software Tools:**
  * DCLD - Digital Control Library Designer, version v0.9.7.99 or higher
  * [Download Digital Control Library SDK for Windows here](https://areiter128.github.io/DCLD/)
    (Using previous versions will result in naming conflicts of labels and function names)

**Development Tools:**
  * MPLAB X v5.35
  * MPLAB X XC16 C-Compiler, v1.50

**History:**
  * | 03/24/2020  | 1.0  |  M91406  | Initial Release
