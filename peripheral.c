//-----------------------------------------------------------------------------
//   File:      peripheral.c
//   Contents:  Hooks required to implement USB peripheral function.
//-----------------------------------------------------------------------------
// Registers which require a synchronization delay, see section TRM 15.14
//
//     EPxAUTOINLENH:L     EPxBCH:L            EPxFIFOCFG          EPxFIFOIE
//     EPxFIFOIRQ          EPxFIFOPFH:L        EPxGPIFFLGSEL       EPxGPIFTRIG
//     FIFOPINPOLAR        FIFORESET           GPIFADRH:L          GPIFIE
//     GPIFIRQ             GPIFTCBx            GPIFTRIG            INPKTEND
//     OUTPKTEND           PINFLAGSxx          REVCTL              UDMACRCH:L
//-----------------------------------------------------------------------------

#pragma noiv // Do not generate interrupt vectors

#include "fx2.h"
#include "fx2regs.h"

extern BOOL GotSUD; // Received setup data flag
extern BOOL Sleep;
extern BOOL Rwuen;
extern BOOL Selfpwr;

BYTE Configuration;    // Current configuration
BYTE AlternateSetting; // Alternate settings

#define VR_NAKALL_ON 0xD0
#define VR_NAKALL_OFF 0xD1

#define VR_TOGGLE_LED 0xB4

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
//   The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------

void TD_Init(void) // Called once at startup
{
    CPUCS = ((CPUCS & ~bmCLKSPD) | bmCLKSPD1); // CPU clock to 48MHz

    IFCONFIG |= 0x40; // set the slave FIFO interface to 48MHz

    EP2CFG = 0xA2; // VALID, OUT, BULK, 512 Bytes, DOUBLE BUFF
    SYNCDELAY;
    EP6CFG = 0xE2; // VALID, IN, BULK, 512 Bytes, DOUBLE BUFF
    SYNCDELAY;

    // arm out endpoints, as they are double buffered write dummy byte counts twice
    EP2BCL = 0x80;
    SYNCDELAY;
    EP2BCL = 0x80;
    SYNCDELAY;

    AUTOPTRSETUP |= 0x01; // enable dual autopointer feature
}

void TD_Poll(void) // Called repeatedly while the device is idle
{
    WORD i;
    WORD count;

    if (!(EP2468STAT & bmEP2EMPTY))
    { 
        if (!(EP2468STAT & bmEP6FULL))
        {
            // setup to transfer EP2OUT buffer to EP6IN buffer using AUTOPOINTER(s)
            AUTOPTRH1 = MSB(&EP2FIFOBUF);
            AUTOPTRL1 = LSB(&EP2FIFOBUF);

            AUTOPTRH2 = MSB(&EP6FIFOBUF);
            AUTOPTRL2 = LSB(&EP6FIFOBUF);

            count = (EP2BCH << 8) + EP2BCL;
            for (i = 0; i < count; i++) 
                EXTAUTODAT2 = EXTAUTODAT1;

            EP6BCH = EP2BCH;
            SYNCDELAY;
            EP6BCL = EP2BCL; // arm EP6IN
            SYNCDELAY;
            EP2BCL = 0x80; // re(arm) EP2OUT
            SYNCDELAY;
        }
    }
}

BOOL TD_Suspend(void) // Called before the device goes into suspend mode
{
    return (TRUE);
}

BOOL TD_Resume(void) // Called after the device resumes
{
    return (TRUE);
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------

BOOL DR_GetDescriptor(void)
{
    return (TRUE);
}

BOOL DR_SetConfiguration(void) // Called when a Set Configuration command is received
{
    Configuration = SETUPDAT[2];
    return (TRUE); // Handled by user code
}

BOOL DR_GetConfiguration(void) // Called when a Get Configuration command is received
{
    EP0BUF[0] = Configuration;
    EP0BCH = 0;
    EP0BCL = 1;
    return (TRUE); // Handled by user code
}

BOOL DR_SetInterface(void) // Called when a Set Interface command is received
{
    AlternateSetting = SETUPDAT[2];
    return (TRUE); // Handled by user code
}

BOOL DR_GetInterface(void) // Called when a Set Interface command is received
{
    EP0BUF[0] = AlternateSetting;
    EP0BCH = 0;
    EP0BCL = 1;
    return (TRUE); // Handled by user code
}

BOOL DR_GetStatus(void)
{
    return (TRUE);
}

BOOL DR_ClearFeature(void)
{
    return (TRUE);
}

BOOL DR_SetFeature(void)
{
    return (TRUE);
}

BOOL DR_VendorCmnd(void)
{
    BYTE tmp;

    switch (SETUPDAT[1])
    {
        case VR_NAKALL_ON:
            tmp = FIFORESET;
            tmp |= bmNAKALL;
            SYNCDELAY;
            FIFORESET = tmp;
            break;
        case VR_NAKALL_OFF:
            tmp = FIFORESET;
            tmp &= ~bmNAKALL;
            SYNCDELAY;
            FIFORESET = tmp;
            break;
        case VR_TOGGLE_LED:
            IOA ^= 1; // toggle LED on PA0

            EP0BCH = 0; // Don't send any data back
            EP0BCL = 0;
            EP0CS |= bmHSNAK;
            break;
        default:
            return (TRUE);
    }

    return (FALSE);
}

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler
void ISR_Sudav(void) __interrupt(0)
{
    GotSUD = TRUE; // Set flag
    EZUSB_IRQ_CLEAR();
    USBIRQ = bmSUDAV; // Clear SUDAV IRQ
}

// Setup Token __interrupt Handler
void ISR_Sutok(void) __interrupt(0)
{
    EZUSB_IRQ_CLEAR();
    USBIRQ = bmSUTOK; // Clear SUTOK IRQ
}

void ISR_Sof(void) __interrupt(0)
{
    EZUSB_IRQ_CLEAR();
    USBIRQ = bmSOF; // Clear SOF IRQ
}

void ISR_Ures(void) __interrupt(0)
{
    // whenever we get a USB reset, we should revert to full speed mode
    pConfigDscr = pFullSpeedConfigDscr;
    ((CONFIGDSCR __xdata *)pConfigDscr)->type = CONFIG_DSCR;
    pOtherConfigDscr = pHighSpeedConfigDscr;
    ((CONFIGDSCR __xdata *)pOtherConfigDscr)->type = OTHERSPEED_DSCR;

    EZUSB_IRQ_CLEAR();
    USBIRQ = bmURES; // Clear URES IRQ
}

void ISR_Susp(void) __interrupt(0)
{
    Sleep = TRUE;
    EZUSB_IRQ_CLEAR();
    USBIRQ = bmSUSP;
}

void ISR_Highspeed(void) __interrupt(0)
{
    if (EZUSB_HIGHSPEED())
    {
        pConfigDscr = pHighSpeedConfigDscr;
        ((CONFIGDSCR __xdata *)pConfigDscr)->type = CONFIG_DSCR;
        pOtherConfigDscr = pFullSpeedConfigDscr;
        ((CONFIGDSCR __xdata *)pOtherConfigDscr)->type = OTHERSPEED_DSCR;
    }

    EZUSB_IRQ_CLEAR();
    USBIRQ = bmHSGRANT;
}
void ISR_Ep0ack(void) __interrupt(0)
{
}
void ISR_Stub(void) __interrupt(0)
{
}
void ISR_Ep0in(void) __interrupt(0)
{
}
void ISR_Ep0out(void) __interrupt(0)
{
}
void ISR_Ep1in(void) __interrupt(0)
{
}
void ISR_Ep1out(void) __interrupt(0)
{
}
void ISR_Ep2inout(void) __interrupt(0)
{
}
void ISR_Ep4inout(void) __interrupt(0)
{
}
void ISR_Ep6inout(void) __interrupt(0)
{
}
void ISR_Ep8inout(void) __interrupt(0)
{
}
void ISR_Ibn(void) __interrupt(0)
{
}
void ISR_Ep0pingnak(void) __interrupt(0)
{
}
void ISR_Ep1pingnak(void) __interrupt(0)
{
}
void ISR_Ep2pingnak(void) __interrupt(0)
{
}
void ISR_Ep4pingnak(void) __interrupt(0)
{
}
void ISR_Ep6pingnak(void) __interrupt(0)
{
}
void ISR_Ep8pingnak(void) __interrupt(0)
{
}
void ISR_Errorlimit(void) __interrupt(0)
{
}
void ISR_Ep2piderror(void) __interrupt(0)
{
}
void ISR_Ep4piderror(void) __interrupt(0)
{
}
void ISR_Ep6piderror(void) __interrupt(0)
{
}
void ISR_Ep8piderror(void) __interrupt(0)
{
}
void ISR_Ep2pflag(void) __interrupt(0)
{
}
void ISR_Ep4pflag(void) __interrupt(0)
{
}
void ISR_Ep6pflag(void) __interrupt(0)
{
}
void ISR_Ep8pflag(void) __interrupt(0)
{
}
void ISR_Ep2eflag(void) __interrupt(0)
{
}
void ISR_Ep4eflag(void) __interrupt(0)
{
}
void ISR_Ep6eflag(void) __interrupt(0)
{
}
void ISR_Ep8eflag(void) __interrupt(0)
{
}
void ISR_Ep2fflag(void) __interrupt(0)
{
}
void ISR_Ep4fflag(void) __interrupt(0)
{
}
void ISR_Ep6fflag(void) __interrupt(0)
{
}
void ISR_Ep8fflag(void) __interrupt(0)
{
}
void ISR_GpifComplete(void) __interrupt(0)
{
}
void ISR_GpifWaveform(void) __interrupt(0)
{
}
