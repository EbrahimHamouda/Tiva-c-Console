#include "inc/tm4c123gh6pm.h"
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_can.h"
//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_types.h"

 tCANMsgObject sCANMessage;
 tCANMsgObject rCANMessage;
 volatile bool g_bRXFlag = 0;
 
//*****************************************************************************
//
// A counter that keeps track of the number of times the TX interrupt has
// occurred, which should match the number of TX messages that were sent.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;
volatile uint32_t g_r_ui32MsgCount = 0;
//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;

//*****************************************************************************

void InitConsole()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0

	GPIOPinConfigure(GPIO_PCTL_PA0_U0RX);
	GPIOPinConfigure(GPIO_PCTL_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0|GPIO_PIN_1); 
	UARTStdioConfig(0,9600,16000000);
}

void
CAN0_Handler(void)
{
	
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32MsgCount++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }
    else if(ui32Status == 2)
    {

        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        CANIntClear(CAN0_BASE, 2);
        g_r_ui32MsgCount++;
        g_bRXFlag = 1;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {

        //
        // Spurious interrupt handling can go here.
        //
    }
}

void
SimpleDelay(void)
{
    //
    // Delay cycles for 1 second
    //
    SysCtlDelay(16000000 / 3);
}
int
main(void)
{
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    uint32_t ui32SysClock;
#endif


    uint32_t ui32MsgData;
    uint8_t *pui8MsgData;
    uint8_t r_pui8MsgData[4];

    pui8MsgData = (uint8_t *)&ui32MsgData;

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_OSC)
                                       25000000);
#else
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
#endif

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for CAN operation.
    //
		InitConsole();

    //
    // For this example CAN0 is used with RX and TX pins on port B4 and B5.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.
    // GPIO port B needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using
    //
    

    //
    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using
    //
    GPIOPinConfigure(GPIO_PCTL_PF0_CAN0RX);
    GPIOPinConfigure(GPIO_PCTL_PF3_CAN0TX);

    //
    // Enable the alternate function on the GPIO pins.  The above step selects
    // which alternate function is available.  This step actually enables the
    // alternate function instead of GPIO for these pins.
    // TODO: change this to match the port/pin you are using
    //
    GPIOPinTypeCAN(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_3);

    //
    // The GPIO port and pins have been set up for CAN.  The CAN peripheral
    // must be enabled.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    //
    // Initialize the CAN controller
    //
    CANInit(CAN0_BASE);

    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() or ui32SysClock is used to determine the 
    // clock rate that is used for clocking the CAN peripheral.  This can be 
    // replaced with a  fixed value if you know the value of the system clock, 
    // saving the extra function call.  For some parts, the CAN peripheral is 
    // clocked by a fixed 8 MHz regardless of the system clock in which case 
    // the call to SysCtlClockGet() or ui32SysClock should be replaced with 
    // 8000000.  Consult the data sheet for more information about CAN 
    // peripheral clocking.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    CANBitRateSet(CAN0_BASE, ui32SysClock, 500000);
#else
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
#endif

    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    // CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //
    // Enable the CAN interrupt on the processor (NVIC).
    //
    IntEnable(55);


    /*  Enable the loop back test */
    HWREG(CAN0_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
    HWREG(CAN0_BASE + CAN_O_TST) |= CAN_TST_LBACK;




    //
    // Initialize the message object that will be used for sending CAN
    // messages.  The message will be 4 bytes that will contain an incrementing
    // value.  Initially it will be set to 0.
    //
    ui32MsgData = 0;
    sCANMessage.ui32MsgID = 1;
    sCANMessage.ui32MsgIDMask = 0;
    sCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessage.ui32MsgLen = sizeof(pui8MsgData);
    sCANMessage.pui8MsgData = pui8MsgData;

    rCANMessage.ui32MsgID = 0;
    rCANMessage.ui32MsgIDMask = 0;
    rCANMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    rCANMessage.ui32MsgLen = 4;

    //
    // Now load the message object into the CAN peripheral.  Once loaded the
    // CAN will receive any message on the bus, and an interrupt will occur.
    // Use message object 1 for receiving messages (this is not the same as
    // the CAN ID which can be any value in this example).
    //
    CANMessageSet(CAN0_BASE, 2, &rCANMessage, MSG_OBJ_TYPE_RX);
    //
    // Enter loop to send messages.  A new message will be sent once per
    // second.  The 4 bytes of message content will be treated as an uint32_t
    // and incremented by one each time.
    //

    //
      // Enable the CAN for operation.
      //
      CANEnable(CAN0_BASE);
    while(1)
    {
        //
        // Print a message to the console showing the message count and the
        // contents of the message being sent.
        //
        if(g_bRXFlag == 1)
        {
            uint8_t uIdx = 0;
            //
            // Reuse the same message object that was used earlier to configure
            // the CAN for receiving messages.  A buffer for storing the
            // received data must also be provided, so set the buffer pointer
            // within the message object.
            //
            rCANMessage.pui8MsgData = r_pui8MsgData;

            //
            // Read the message from the CAN.  Message object number 1 is used
            // (which is not the same thing as CAN ID).  The interrupt clearing
            // flag is not set because this interrupt was already cleared in
            // the interrupt handler.
            //
            CANMessageGet(CAN0_BASE, 2, &rCANMessage, 0);

            //
            // Clear the pending message flag so that the interrupt handler can
            // set it again when the next message arrives.
            //
            g_bRXFlag = 0;

            //
            // Check to see if there is an indication that some messages were
            // lost.
            //
            if(sCANMessage.ui32Flags & MSG_OBJ_DATA_LOST)
            {
                UARTprintf("CAN message loss detected\n");
            }

            //
            // Print out the contents of the message that was received.
            //
            UARTprintf("Msg ID=0x%08X len=%u data=0x",
                       sCANMessage.ui32MsgID, sCANMessage.ui32MsgLen);
            for(uIdx = 0; uIdx < sCANMessage.ui32MsgLen; uIdx++)
            {
                UARTprintf("%02X ", r_pui8MsgData[uIdx]);
            }
            UARTprintf("total rx count=%u\n", g_r_ui32MsgCount);
        }
        else

        {
            UARTprintf("Sending msg: 0x%02X %02X %02X %02X",
                               pui8MsgData[0], pui8MsgData[1], pui8MsgData[2],
                               pui8MsgData[3]);

                    //
                    // Send the CAN message using object number 1 (not the same thing as
                    // CAN ID, which is also 1 in this example).  This function will cause
                    // the message to be transmitted right away.
                    //
                    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_TX);

                    //
                    // Now wait 1 second before continuing
                    //
                    SimpleDelay();

                    //
                    // Check the error flag to see if errors occurred
                    //
                    if(g_bErrFlag)
                    {
                        UARTprintf(" error - cable connected?\n");
                    }
                    else
                    {
                        //
                        // If no errors then print the count of message sent
                        //
                        UARTprintf(" total count = %u\n", g_ui32MsgCount);
                    }

                    //
                    // Increment the value in the message data.
                    //
                    ui32MsgData++;
        }

    }

}
