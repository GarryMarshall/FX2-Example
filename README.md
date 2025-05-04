# FX2-Example

This code is a simple example of programming the FX2 device from Cypress

When the device first boots it toggles LEDs on PA0 and PA1 to show the code is running

Once booted any data sent from EP2 (OUT) is sent back to EP6 (IN) and issuing vendor command 0xB4 will toggle the LED attached to PA0.

This code can be tested using cyControl and Streamer, both of which are C# programs that come as part of the 
[EZ-USB SDK](https://www.infineon.com/cms/en/design-support/tools/sdk/usb-controllers-sdk/ez-usb-fx3-software-development-kit/)

**4th May 2025:** Modified code so that it compiles and runs correctly with 
[SDCC](https://sdcc.sourceforge.net/) v4.5. The original code had two bugs, the first being the ISR table being incorrectly positoned 
because of the way the vector table was set up in USBJumpTable.asm.

The second bug was due to two variables being defined DWORD in main() when they should be defined as WORD.  This last bug was 
in the original code that came as part of the EZ-USB SDK v1.3

![Devboard](fx2-dev-board.jpg)
