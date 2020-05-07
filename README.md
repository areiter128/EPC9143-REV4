<p><a href="https://www.microchip.com" rel="nofollow"><img src="https://www.microchip.com/ResourcePackages/Microchip/assets/dist/images/logo.png" alt="Microchip Technology" style="max-width:100%;"></a></p>

## P33C_CE200: Simple Control Loop Implementation
**Synchronous Buck Converter in Voltage Mode Control**

**Description:**
This code example demonstrates a closed loop voltage mode control implementation for dsPIC33CK. It has specifically been developed for the Digital Power Starter Kit 3 (DPSK3) buck converter.

The board starts up the buck converter automatically when power is applied to the board, providing a regulated output voltage of 3.3V at the output of the buck converter. The startup procedure is controlled and executed by the power controller state machine (PWRLIB_BUCK) and includes an configurable startup procedure with power-on delay, ramp up period and power good delay before dropping into constant regulation mode. 
An additional fault handler (FUNLIB_FAULT_HANDLER) shuts down the power supply if the input voltage is outside the defined range of 6V to 13.8V (UVLO/OVLO) or if the output voltage is more than 0.5V out of regulation for more than 10 milliseconds. 
The most recent input and output voltage is continuously shown on the LC display of the board and is updated every 200 ms. 

The control loop is based on a digital type III compensator (3P3Z controller). 


**Operation Parameters:**
  * Input Voltage: 6 V ... 13.8 V
  * Output Voltage: 3.3 V
  * Switching Frequency:	500 kHz
  * Control Frequency:	500 kHz
  * Cross-Over Frequency: ~15 kHz (depends on VIN and if AGC is on/off)
  * Phase Margin: ~ 50°

**Required Hardware:** 
  * DPSK3: dsPIC33C Digital Power Starter Kit 3 (Part-No. [DM330017-3](https://www.microchip.com/dm330017-3))
  * DPSK3 system blocks used:
    * Buck Converter
    * LC Display
    * Debugging LED
    * User Switch Button (turns on/off AGC control algorithm)
    * Debugging Pins TP50, TP52 and TP53

**Device Support:**
  * [dsPIC33CK256MP505](https://www.microchip.com/dsPIC33CK256MP505)
  
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

