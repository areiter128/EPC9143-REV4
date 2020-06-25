<p><a href="https://www.microchip.com" rel="nofollow"><img src="https://www.microchip.com/ResourcePackages/Microchip/assets/dist/images/logo.png" alt="Microchip Technology" style="max-width:100%;"></a></p>

## EPC9143 300W 16th Brick Non-Isolated Step Down Converter Firmware
**2-Phase Synchronous Buck Converter with Advanced Type IV Voltage Mode Control, Adaptive Gain Control and Current Balancing**

**Description:**
This code example demonstrates a closed loop voltage mode control implementation for dsPIC33CK. It has specifically been developed for the Digital Power Starter Kit 3 (DPSK3) buck converter.

The board starts up the buck converter automatically when power is applied to the board, providing a regulated output voltage of 3.3V at the output of the buck converter. The startup procedure is controlled and executed by the power controller state machine (PWRLIB_BUCK) and includes an configurable startup procedure with power-on delay, ramp up period and power good delay before dropping into constant regulation mode. 
An additional fault handler (FUNLIB_FAULT_HANDLER) shuts down the power supply if the input voltage is outside the defined range of 6V to 13.8V (UVLO/OVLO) or if the output voltage is more than 0.5V out of regulation for more than 10 milliseconds. 
The most recent input and output voltage is continuously shown on the LC display of the board and is updated every 200 ms. 

The control loop is based on a digital type III compensator (3P3Z controller). 


**Operation Parameters:**
  * Input Voltage: 6 V ... 13.8 V
  * Output Voltage: 3.0 V
  * Switching Frequency:	500 kHz
  * Control Frequency:	500 kHz
  * Cross-Over Frequency: ~15 kHz (depends on VIN and if AGC is on/off)
  * Phase Margin: ~ 50°

**Required Hardware:** 
  * EPC9143: EPC9143 16th Brick Non-Isolated Step Down Converter, Revision 4.0
  * EPC9531: EPC9531 test fixture for EPC9143 16th brick reference design
  * Digital Power Libraries System Blocks used (see below)
    * Multiphase Buck Converter
	* Fault Handler

**Device Support:**
  * [dsPIC33CK32MP102](https://www.microchip.com/dsPIC33CK32MP102)
  
**Libraries:**
  * [PWRLIB_BUCK](https://bitbucket.microchip.com/projects/MCU16ASMPSL/repos/pwrlib_buck/browse) - Single Phase Buck Converter State Machine & Peripheral Configuration Template
  * [FUNLIB_FAULT_HANDLER](https://bitbucket.microchip.com/projects/MCU16ASMPSL/repos/funlib_faulthandler/browse) - Generic Fault Handler Template

**Software Tools:**
  * DCLD - Digital Control Library Designer, version v0.9.7.104 or higher
  * [Download Digital Control Library SDK for Windows here](https://areiter128.github.io/DCLD/)
    (Using previous versions will result in naming conflicts of labels and function names)

**Development Tools:**
  * MPLAB X v5.35
  * MPLAB X XC16 C-Compiler, v1.50

**History:**
  * | 03/24/2020  | 1.0  |  M91406  | Initial Development Version
  * | 06/25/2020  | 1.3  |  M91406  | Reviewed and tested AGC implementation (first release version)  

