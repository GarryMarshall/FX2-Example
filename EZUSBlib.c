//-----------------------------------------------------------------------------
// File: ezusblib.c
// Contents: All the EZUSBxxx functions
//-----------------------------------------------------------------------------
#include "fx2.h"
#include "fx2regs.h"

void EZUSB_Delay(WORD ms)
{
    if (CPUCS & bmCLKSPD)
        ms = ms * 2;            // 48Mhz
    else
        ms = (ms + 1) / 2;      // 12Mhz

    while (ms--)
        EZUSB_Delay1ms();
}

//disconnect function.  used to electrically disconnect from USB for renumeration
void EZUSB_Discon(BOOL renum)
{

    if (renum)                         // If renumerate (i.e. 8051 will handle SETUP commands)
        USBCS |= (bmDISCON | bmRENUM); // disconnect from USB and set the renumerate bit
    else
        USBCS |= bmDISCON; // just disconnect from USB

    EZUSB_Delay(1500); // Wait 1500 ms

    USBIRQ = 0xff; // Clear any pending USB interrupt requests
    EPIRQ = 0xff;
    EZUSB_IRQ_CLEAR();

    USBCS &= ~bmDISCON; // reconnect USB
}

// function to wake-up the host after a resume
void EZUSB_Resume(void)
{
    if (((WAKEUPCS & bmWUEN) && (WAKEUPCS & bmWU)) || // Check status AND Enable
        ((WAKEUPCS & bmWU2EN) && (WAKEUPCS & bmWU2)))
    {
        USBCS |= bmSIGRESUME;
        EZUSB_Delay(20);
        USBCS &= ~bmSIGRESUME;
    }
}


// function to return a pointer to a string descriptor given an index
STRINGDSCR __xdata *EZUSB_GetStringDscr(BYTE StrIdx)
{
    STRINGDSCR __xdata *dscr;

    dscr = (STRINGDSCR __xdata *)pStringDscr;

    while (dscr->type == STRING_DSCR)
    {
        if (!StrIdx--)
            return (dscr);
        dscr = (STRINGDSCR __xdata *)((WORD)dscr + dscr->length);
    }

    return (NULL);
}

