![image](images/microchip.jpg) 

## EPC9143 300W 16th Brick Non-Isolated Step Down Converter Advanced Voltage Mode Control Firmware
**2-Phase Synchronous Buck Converter with Advanced Voltage Mode Control and Current Balancing**

<p>
  <center>
    Top View <br>
    <a href="https://www.microchip.com/EPC9143" rel="nofollow">
      <img src="images/9143_r4.png" alt="EPC9143 Top View" width="300">
	</a>	
  </center>
</p>
<p>
  <center>
    Bottom View <br>
    <a href="https://www.microchip.com/EPC9143" rel="nofollow">
      <img src="images/9143_r4_back.png" alt="EPC9143 Top View" width="300">
    </a>
  </center>
</p>

## Summary
This code example demonstrates a closed loop voltage mode control implementation for dsPIC33CK. It has specifically been developed for the EPC9143 Rev4.0 1/16 brick converter.

The board starts up the buck converter automatically when power is applied to the board, providing a regulated output voltage of 12 V at the output of the converter. The startup procedure is controlled and executed by the power controller state machine (PWRLIB_BUCK) and includes an configurable startup procedure with power-on delay, ramp up period and power good delay before dropping into constant regulation mode.
An additional fault handler (FUNLIB_FAULT_HANDLER) shuts down the power supply if the input voltage is outside the defined range of 18V to 72.5V (UVLO/OVLO) or if the output voltage is more than 0.5V out of regulation for more than 10 milliseconds.

A single, high-speed type IV (4P4Z) voltage mode controller with enforced PWM steering is used to automatically create balanced phase currents in both phases of this interleaved converter. An underlying current balancing scheme compensates component tolerances and deviations over temperature. A built-in adaptive gain control algorithm stabilizes gain variations of the voltage loop controller during input- and output voltage transients, stabilizing cross-over frequency and output impedance, supporting control bandwidths of 25 kHz, for improved transient response, helping to minimize power distribution network (PDN) decoupling capacity.

#### Product Features:
  - Input Voltage: 18 V to 61 V
  - Output Voltage: 12 V DC (programmable from 5 V to 15.0 V)
  - Switching Frequency: 500 kHz
  - Control Frequency: 500 kHz
  - Cross-Over Frequency: ~25 kHz (depends on VIN and if AGC is on/off)
  - Phase Margin: ~ 50°
  - Gain Margin: ~ 12 dB

## Related Documentation
  - [EPC9143 300W 1/16th brick Power Module Reference Design Product Website](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/EPC9143)

## Software Used 
  - [MPLAB® X IDE v5.40](https://www.microchip.com/mplabx-ide-windows-installer)
  - [MPLAB® XC16 Compiler v1.50](https://www.microchip.com/mplabxc16windows)
  - [DCLD - Digital Control Library Designer v0.9.12.642](https://areiter128.github.io/DCLD/)

## Hardware Used
  - EPC9143: EPC9143 16th Brick Non-Isolated Step Down Converter, Revision 4.0
  - EPC9531: EPC9531 test fixture for EPC9143 16th brick reference design

<p>
  <center>
    <a href="https://www.microchip.com/EPC9143" rel="nofollow">
      <img src="images/9531_test_fixture.png" alt="EPC9143 mounted on EPC9531 Test Fixture" width="500">
	</a>
  </center>
</p>

## Device Support:
The following Microchip devices are used on the EPC9143 refrence design:
  - [dsPIC33CK32MP102](https://www.microchip.com/dsPIC33CK32MP102)
  - [MCP6C02 Shunt Amplifier](https://www.microchip.com/MCP6C02)

## Setup
The EPC9143 1/16th brick module is best tested plugged into EPC9531 test fixture. The EPC9531 QSG provides detailed operating procedure instructions. 

Available documents provided by EPC:

  - [EPC9143 Quick Start Guide (QSG)](https://epc-co.com/epc/documents/guides/EPC9143_qsg.pdf)
  - [EPC9143 Schematics](https://epc-co.com/epc/documents/schematics/EPC9143_Schematic.pdf)
  - [EPC9531 Quick Start Guide (QSG)](https://epc-co.com/epc/documents/guides/EPC9531_qsg.pdf)
  - [EPC9531 Test Fixture Schematics](https://epc-co.com/epc/documents/schematics/EPC9531_Schematic.pdf)

The bard comes preogrammed and ready to be used when unpacked. No reprogramming of the target device is required to operate the board unless features or settings such as the nominal output voltage or start-up timimng need to be modified in which case the Microchip dsPIC33CK controller can be re-programmed using the in-circuit serial programming port (ICSP) available on the RJ-11 programming interface as well as the 5-pin header provided by the EPC9531 test fixture. These interfaces support all of Microchip’s in-circuit programmers/debuggers, such as MPLAB® ICD4, MPLAB® REAL ICE or MPLAB® PICkit4 and previous derivatives. See [EPC9531 Quick Start Guide](https://epc-co.com/epc/documents/guides/EPC9531_qsg.pdf) for details.

## Operation
The converter is starting up automatically when more than 10.5 V DC are applied across the input terminals of the EPC9531 test fixture. It is not recommended to operate the EPC9143 refrence design without proper decoupling capacitance at either input or output. The EPC9531 test fixture provides the best test environment for the converter. Pleease read the [EPC9531 Quick Start Guide](https://epc-co.com/epc/documents/guides/EPC9531_qsg.pdf) to get detailed information about the requirements for setup and operation of this refrence design.

## Firmware Qick-Start Guide

##### 1) Buck Converter State Machine

The state machine goes through the following steps in chronological order:

a) Initialization

In this step the control loop parameters are reset to their defaults, PWM outputs are turned off but the PWM is still running, continuously triggering the ADC to keep sampling input and output voltage as well as board temperature.

b) Reset
This is the 'fall-back' state from which the buck converter will be restarted once it has been started successfully and has been shut down due to a fault condition (e.g. input under/over voltage or over temperature condition)

c) Standby
After RESET, the state machine waits for all fault flags to be cleared and the enable and GO bits to be set.

d) Power-On Delay (POD)
Once the buck converter has been cleared the state machine will execute the startup procedure starting with the Power On Delay. This is just a simple delay during which the converter will remain inactive but the fault handler will observe the values generated by the ADC for occurring fault conditions.

e) Launch Voltage Ramp
After the Power-On delay has expired, input and output voltage will be measured. In case the converter output is pre-biased (voltage = non-zero), the power controller will be 'pre-charged' with an artificial control history and PWM output to softly ramp up the output voltage from its most recent level. 

f) Voltage Ramp-Up
Now the digital feedback loop and PWM are enabled and the closed loop system reference value is incremented with every execution of the state machine (100usec interval). The control loop has been adjusted to operate with a cross-over frequency of >10 kHz matching the maximum perturbance frequency allowed to keep the control system stable.  

g) Power Good Delay
After the reference voltage has been increased to the pre-defined nominal level, the state machine switches over into the Power Good Delay period. This is another, simple delay where the control loop is in steady state waiting for the delay period to expire.

h) Online
After the Power Good Delay has expired, the converter drops into nominal operation. In this condition it continuously observes the reference value for changes. Should any other part of the firmware change the controller reference, the state machine will softly tune into the new level instead of hard-switching the reference. 

i) Suspend/Error
If the power controller is shut down and reset by external commands (e.g. fault handler detecting a fault condition or through user-interaction), the state machine is switching into the SUSPEND state, which disables the PWM outputs and control loop execution, clears the control histories and resets the state machine back to RESET

##### 2) Cycle-by-Cycle Voltage Control Loop

This firmware uses a digital type IV controller to close the feedback loop in voltage mode control. This digital loop reads the most recent ADC sample of the output voltage and processes the derived value through a digital type IV (4P4Z) compensation filter. The numeric output is checked and, when necessary, clamped to user-defined minimum/maximum thresholds before being written to the PWM duty cycle register. As EPC9143 is a multiphase converter, a special PWM steering mode has been implemented supporting high-speed current balancing between the two phases. The voltage mode convtroller has been extended with a self-adapting loop gain tuing algorithm (AGC) stabilizing the frequency domain characteristic over input voltage, output voltage and load, always providing the exact same bandwidth of >25 kHz and stability margins across the entire operating range.

This control loop can be turned on/off by using the ENABLE bit in the STATUS word of the cNPNZ_t controller data structure. The adaptive loop gain modulation is permanently active as soon as the control loop is enabled.

##### 3) Digital Controller Design

The control loop source code is configured and generated by using the 
Digital Control Loop Designer (DCLD). 

This additional design software is available for download by following this link:

[https://areiter128.github.io/DCLD](https://areiter128.github.io/DCLD)

Once installed, the controller configuration can be modified. The most recent configuration can be opened from within the MPLAB X® IDE by right-clicking on the file 'DPSK3_VMC.dcld' located in the Important Files folder of the Project Manager. When right-clicked, select 'Open In System' to open the configuration in DCLD. 

Please refer to the user guide of DCLD whcih is included in the software and can be opened from the help menu of the application.

##### 4) User Control

No user control interface has been added to the firmware. Any change to the firmware and fundamental operation of the reference design, including reprogramming of the nominal output voltage can be done by editing the hardware-specific values in the hardware description header file 'epc9143_r40_hwdescr.h' located in 'Project Manager => Header Files/Config'

Converter settings in this file are defined as phasical values such as Volt, Ampere, Ohm, etc. Each defined value is converted into binary numbers by so-called macros, at compile time. Thus, users do not have to convert values manually.

##### Example:
To program the converter to provide a nominal output voltage different from the 12 V DC set by default, follow these steps:

  - Open the project in MPLAB X® IDE
  - Navigate to 'Header Files/Config/epc9143_r40_hwdescr.h' using the Project Manager on the left of the main window
  - Go to line #325 (see below)
  - Change the give settings as desired
  - Build the program
  - Remove power from the input of the EPC9531 test fixture
  - Connect a valid ICSP programming device (e.g. MPLAB ICD4, MPLAB PICkit4) to the PC and the EPC9531 test fixture (see [EPC9531 Quick Start Guide](https://epc-co.com/epc/documents/guides/EPC9531_qsg.pdf) for details)
  - Program the device whith the target devicebeing powered by the debugger/programmer
  - Disconnect the ICSP programming device from the EPC9531 test fixture
  - Apply valid input voltage across the input terminals of EPC9531 and observve the output of the EPC9143 reference design

The setting for the nominal output votage is found in lines #324 through #326.

    #define BUCK_VOUT_NOMINAL           (float)12.000  // Nominal output voltage
    #define BUCK_VOUT_TOLERANCE_MAX     (float)0.500   // Output voltage tolerance [+/-]
    #define BUCK_VOUT_TOLERANCE_MIN     (float)0.100   // Output voltage tolerance [+/-]

###### Please note:
The tolerance settings above include the transient response at a maximum load step. The value for maximum output voltage tolerance 'BUCK_VOUT_TOLERANCE_MAX' is observed by the fault handler. Should the output voltage reading divert from the most recent reference voltage value by more than the given range, the converter will be shut down and a REGULATION ERROR will be indicated. The power supply will automatically recover as soon as the fault condition has been cleared and the recover delay period specified by BUCK_REGERR_RECOVERY_DELAY in line #527 of the EPC9143 hardware description header file has expired.

(line numbers given may be subject to change)

##### 5) Power Plant Measurement Support

This code examples includes an alternative, proportional control loop which is commonly used during measurements of the frequency response of the power plant. When the following define is set to TRUE, the common main control loop is replaced by the proportional controller.

    app_power_control.c, line 33:   #define PLANT_MEASUREMENT   false


###### PLEASE NOTE:
PROPORTIONAL CONTROLLERS ARE BY DEFAULT UNSTABLE AND NOT SUITED TO REGULATE THE OUTPUT OF A POWER SUPPLY UNDER NORMAL OPERATING CONDITIONS. DURING A PLANT MEASUREMENT IT IS MANDATORY THAT INPUT VOLTAGE AND LOAD REMAIN STABLE AND DO NOT CHANGE. 

FOR MORE INFORMATION ABOUT HOW TO CONDUCT A POWER PLANT MEASUREMENT, PLEASE READ THE SECTIONS IN THE DCLD USER GUIDE.

_________________________________________________
(c) 2020, Microchip Technology Inc.

