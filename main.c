/*
 * Author: Daniel Kleebinder
 *
 * A quadro-copter controller.
 *
 * Already set build options of this project (no need to change anything):
 * Stack size set to 1024 byte
 * Heap size disabled - No malloc() available
 * No code optimization - Easier debugging
 * Strict floating point interrupt behavior
 * Hardware floating point unit activated
 */

#define TARGET_IS_TM4C129_RA2 /* Tells rom.h the version of the silicon */

#include "libs/sleep.h"
#include "libs/utils.h"
#include "libs/serialio.h"
#include "libs/rn4678.h"

#include <math.h>
#include <string.h>
#include <ctype.h>
#include <cstdio>
#include <stdint.h> /* C99 header for uint*_t types */
#include <stdbool.h> /* Driverlib headers require stdbool.h to be included first */

#include <driverlib/gpio.h> /* Supplies GPIO* functions and GPIO_PIN_x */
#include <driverlib/sysctl.h> /* Supplies SysCtl* functions and SYSCTL_* macros */
#include <driverlib/rom.h> /* Supplies ROM_* variations of functions */
#include <driverlib/adc.h>
#include <driverlib/interrupt.h>
#include <driverlib/timer.h>
#include <driverlib/uart.h>
#include <driverlib/pin_map.h>

#include <inc/hw_memmap.h> /* Supplies GPIO_PORTx_BASE */
#include <inc/hw_ints.h>


/* Controller is initially clocked with 16 MHz (via PIOSC) */
/* !!! Changing this macro does not change clock speed !!! */
#define F_CPU (120000000)       /* 120 MHz */
#define F_TIM (16000000)        /* 16 MHz  */
#define UART_BAUD_RATE (115200)
#define STICK_BUFFER_ZONE (40)

#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define CLAMP(X, I, A) (MIN(MAX((X), (I)), (A)))

#define PORT_L (GPIO_PIN_2 | GPIO_PIN_1)
#define PORT_F (GPIO_PIN_4 | GPIO_PIN_0)
#define PORT_N (GPIO_PIN_1 | GPIO_PIN_0)
#define PORT_C (GPIO_PIN_6)
#define PORT_E (GPIO_PIN_4 | GPIO_PIN_3)

#define ADC_SEQ (2)



// Data Package
struct DataPackage {
    uint32_t stickX;
    uint32_t stickY;
    uint8_t pressedButtonS1;
    uint8_t pressedButtonS2;
    uint8_t dirty;
    uint8_t dirtyCounter;
    int8_t speed;
    bool arm;
} dataBuffer;


volatile uint8_t initializedBluetooth = false;
volatile uint32_t tickCounter = 0;



// Function Prototypes
void switchLights(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
void isrPadButtonCallback(void);
void isrStickCallback(void);
void isrStickButtonCallback(void);

void isrTimerCallback(void);
void isrTimerSendPacketCallback(void);
void isrTimerTickCallback(void);

void isrBluetoothHandler(void);

void rn4678InitializeBluetooth(void);
void rn4678Connect(void);

void sendDataPacket(struct DataPackage data);

void inFlightSerialCommunication(void);



/**
 * Switches the given lights on or off.
 *
 * @param d1 First list.
 * @param d2 Second list.
 * @param d3 Third list.
 * @param d4 Fourth list.
 */
void switchLights(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4) {
    uint8_t v1 = 0, v2 = 0;

    v1 |= d2 ? GPIO_INT_PIN_0 : 0;
    v1 |= d1 ? GPIO_INT_PIN_1 : 0;
    v2 |= d4 ? GPIO_INT_PIN_0 : 0;
    v2 |= d3 ? GPIO_INT_PIN_4 : 0;

    GPIOPinWrite(GPIO_PORTN_BASE, PORT_N, v1);
    GPIOPinWrite(GPIO_PORTF_BASE, PORT_F, v2);
}


/**
 * Callback function for pad buttons J4.33 (S1) and J4.32 (S2) on port L.
 */
void isrPadButtonCallback() {
    uint32_t status = GPIOIntStatus(GPIO_PORTL_BASE, true);
    GPIOIntClear(GPIO_PORTL_BASE, PORT_L);

    // Suppress button bouncing
    static uint32_t lastTickCounter = 0;
    if ((lastTickCounter + 180) >= tickCounter) {
        return;
    }
    lastTickCounter = tickCounter;

    // Status is 2 if button S1 was pressed and 4 if button S2 was pressed.
    // If both buttons are pressed, bit 1 and 2 will be 1 and the value is 6.
    uint8_t pressedButtonS1 = status & (1 << 1);
    uint8_t pressedButtonS2 = status & (1 << 2);

    dataBuffer.pressedButtonS1 = pressedButtonS1;
    dataBuffer.pressedButtonS2 = pressedButtonS2;
    dataBuffer.dirty = true;
}


/**
 * Callback function for the stick control and ADC on port E.
 */
void isrStickCallback() {
    uint32_t status = ADCIntStatus(ADC0_BASE, ADC_SEQ, true);
    ADCIntClear(ADC0_BASE, ADC_SEQ);

    static uint32_t buffer[4];
    ADCSequenceDataGet(ADC0_BASE, ADC_SEQ, buffer);

    static uint32_t boundsX[] = {2048, 2048};   // [29,4014]
    static uint32_t boundsY[] = {2048, 2048};   // [3,4046]

    boundsX[0] = MIN(boundsX[0], buffer[0]);
    boundsX[1] = MAX(boundsX[1], buffer[0]);
    boundsY[0] = MIN(boundsY[0], buffer[1]);
    boundsY[1] = MAX(boundsY[1], buffer[1]);

    if ((buffer[0] < (dataBuffer.stickX - STICK_BUFFER_ZONE))
            || (buffer[0] > (dataBuffer.stickX + STICK_BUFFER_ZONE))) {
        dataBuffer.stickX = buffer[0];
        dataBuffer.dirty = true;
    }

    if ((buffer[1] < (dataBuffer.stickY - STICK_BUFFER_ZONE))
            || (buffer[1] > (dataBuffer.stickY + STICK_BUFFER_ZONE))) {
        dataBuffer.stickY = buffer[1];
        dataBuffer.dirty = true;
    }
}


/**
 * Callback function for stick button.
 */
void isrStickButtonCallback() {
    uint32_t status = GPIOIntStatus(GPIO_PORTC_BASE, true);
    GPIOIntClear(GPIO_PORTC_BASE, PORT_C);

    // Suppress button bouncing
    static uint32_t lastTickCounter = 0;
    if ((lastTickCounter + 250) >= tickCounter) {
        return;
    }
    lastTickCounter = tickCounter;

    // Toggle Copter Arm
    dataBuffer.arm = !dataBuffer.arm;

    // Not able to start copter while speed is greater or equal to 3
    if (dataBuffer.speed >= 3) {
        dataBuffer.speed = 0;
    }
}


/**
 * Callback function for the system interrupt timer with 100 ms delay.
 */
void isrTimerCallback() {
    uint32_t status = TimerIntStatus(TIMER1_BASE, true);
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC0_BASE, ADC_SEQ);
}


/**
 * Callback for the send bluetooth packet system.
 */
void isrTimerSendPacketCallback() {
    uint32_t status = TimerIntStatus(TIMER1_BASE, true);
    TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

    static uint8_t internalCounter = 0;
    internalCounter = (internalCounter + 1) % 8;

    if (!initializedBluetooth) {
        switchLights(
                internalCounter == 3 || internalCounter == 4,
                internalCounter == 2 || internalCounter == 5,
                internalCounter == 1 || internalCounter == 6,
                internalCounter == 0 || internalCounter == 7
                );
    }

    // Send package every 100 ms if data has changed
    if (initializedBluetooth) {
        dataBuffer.speed += (dataBuffer.pressedButtonS1 ? 1 : 0);
        dataBuffer.speed -= (dataBuffer.pressedButtonS2 ? 1 : 0);

        dataBuffer.speed = CLAMP(dataBuffer.speed, 0, 15);

        dataBuffer.pressedButtonS1 = false;
        dataBuffer.pressedButtonS2 = false;
        dataBuffer.dirty = false;

        sendDataPacket(dataBuffer);

        if (!dataBuffer.arm) {
            switchLights(
                    internalCounter == 1,
                    internalCounter == 0,
                    internalCounter == 1,
                    internalCounter == 0
                    );
        } else {
            switchLights(
                    dataBuffer.speed & (1 << 0),
                    dataBuffer.speed & (1 << 1),
                    dataBuffer.speed & (1 << 2),
                    dataBuffer.speed & (1 << 3)
                    );
        }
    }
}


/**
 * Callback function for global system ticks and time measurements.
 */
void isrTimerTickCallback() {
    uint32_t status = TimerIntStatus(TIMER2_BASE, true);
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    tickCounter++;
}


/**
 * Bluetooth UART handler.
 */
void isrBluetoothHandler() {
    uint32_t status = UARTIntStatus(UART6_BASE, true);
    UARTIntClear(UART6_BASE, status);

    /* Receive Code Here */
    static uint8_t buffer[32];
    int i = 0;
    while(UARTCharsAvail(UART6_BASE)) {
        if (i >= 32) {
            break;
        }
        uartReadByteNonBlocking(UART6_BASE, &buffer[i++]);
    }

    rn4678RTS(true);
    rn4678RTS(false);
}


/**
 * Initializes the bluetooth connection to the quadro copter.
 */
void rn4678InitializeBluetooth() {
    uartWriteLine(UART0_BASE, "Initializing Bluetooth Connection...");

    rn4678Reset(false);
    sleep(1);
    rn4678Reset(true);

    rn4678SoftwareButton(true);

    sleep(1000);
    rn4678RTS(false);
}


/**
 * Connects the system to the quadro copter.
 */
void rn4678Connect() {
    uartWriteLine(UART0_BASE, "Establishing RN4678 Communication...");

    uartWriteNonBlocking(UART6_BASE, "$$$");                   // Activate configuration mode
    sleep(500);

    // Use \r to complete the command
    uartWriteNonBlocking(UART6_BASE, "SF,1\r");                // Restore factory defaults
    sleep(500);

    uartWriteNonBlocking(UART6_BASE, "SG,2\r");                // Classical bluetooth mode
    sleep(500);

    uartWriteNonBlocking(UART6_BASE, "SA,1\r");                // Simple pairing with pin
    sleep(500);

    uartWriteNonBlocking(UART6_BASE, "R,1\r");                 // Restarts the module
    sleep(2000);

    uartWriteNonBlocking(UART6_BASE, "$$$");                   // Activate command mode
    sleep(500);

    uartWriteNonBlocking(UART6_BASE, "C,0006668CB2E2\r");      // Transmit MAC-Address of Quadrocopter
    sleep(500);

    uartWriteNonBlocking(UART6_BASE, "---\r");
    sleep(500);

    uartWriteLine(UART0_BASE, "Connected to Quadrocopter");
}


/**
 * Sends a data packet to the qudro-copter using bluetooth and MultiWii.
 */
void sendDataPacket(struct DataPackage data) {
    uint8_t packet[16];

    /* All values need to be clamped between [1000; 2000] */
    uint16_t roll = (uint16_t) ((data.stickY / 8 - 250) + 1500);
    uint16_t pitch = (uint16_t) ((data.stickX / 8 - 250) + 1500);
    uint16_t yaw = (uint16_t) 1500;

    uint16_t throttle = (uint16_t) (data.speed * 65 + 1000);

    /* MultiWii Header */
    packet[0] = (uint8_t) 0x24;
    packet[1] = (uint8_t) 0x4D;
    packet[2] = (uint8_t) 0x3C;

    /* Payload Packet Size */
    packet[3] = (uint8_t) 0x0A;

    /* Packet Type */
    packet[4] = (uint8_t) 0xC8;

    /* Quadro Copter Pitch */
    packet[5] = (uint8_t) pitch;
    packet[6] = (uint8_t) (pitch >> 8);

    /* Quadro Copter Roll */
    packet[7] = (uint8_t) roll;
    packet[8] = (uint8_t) (roll >> 8);

    /* Quadro Copter Speed */
    packet[9] = (uint8_t) throttle;
    packet[10] = (uint8_t) (throttle >> 8);

    /* Quadro Copter Yaw */
    packet[11] = (uint8_t) yaw;
    packet[12] = (uint8_t) (yaw >> 8);

    /* Quadro Copter Arm */
    if (data.arm) {
        packet[13] = (uint8_t) 0xD0;
        packet[14] = (uint8_t) 0x07;
    } else {
        packet[13] = (uint8_t) 0xE8;
        packet[14] = (uint8_t) 0x03;
    }

    /* Compute Checksum */
    packet[15] = (uint8_t) checksum(packet + 3, 12);

    /* Write Packet */
    uartWriteBytesNonBlocking(UART6_BASE, packet, 16);
}


/**
 * Communicates with the UART serial port.
 */
void inFlightSerialCommunication() {
    unsigned char data[256];

    uartWriteLine(UART0_BASE, "\r\nWelcome to the Serial UART Interface of the Quadro-Copter!\r\n");
    uartWriteLine(UART0_BASE, "The following commands are available from this CLI:");
    uartWriteLine(UART0_BASE, "  -> +    Increase Speed by one tick");
    uartWriteLine(UART0_BASE, "  -> -    Decrease Speed by one tick");
    uartWriteLine(UART0_BASE, "  -> a    Arm the quadro-copter");
    uartWriteLine(UART0_BASE, "  -> d    Disarm the quadro-copter");
    uartWriteLine(UART0_BASE, "  -> x    Exit the command line interface\r\n");

    while (true) {
        uartWrite(UART0_BASE, "Command: ");
        uartReadLine(UART0_BASE, data);
        uartWriteLine(UART0_BASE, "");

        if (strcmp("+", (const char*) data) == 0) {
            dataBuffer.pressedButtonS1 = true;
        }
        if (strcmp("-", (const char*) data) == 0) {
            dataBuffer.pressedButtonS2 = true;
        }

        if (strcmp("a", (const char*) data) == 0) {
            dataBuffer.arm = true;
        }
        if (strcmp("d", (const char*) data) == 0) {
            dataBuffer.arm = false;
        }

        if (strcmp("x", (const char*) data) == 0) {
            break;
        }
    }
}


/**
 * Main program entrance point.
 *
 * @return Exit code.
 */
int main(void) {
    dataBuffer.stickX = 2048;
    dataBuffer.stickY = 2048;
    dataBuffer.pressedButtonS1 = false;
    dataBuffer.pressedButtonS2 = false;
    dataBuffer.speed = 0;
    dataBuffer.arm = true;
    dataBuffer.dirty = false;
    dataBuffer.dirtyCounter = 0;

    /* Set CPU clock speed */
    uint32_t sysClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, F_CPU);

    /* Activate GPIO ports */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);        // Joystick Button
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);        // Joystick ADC (Analog-Digital-Converter)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);        // Gamepad Buttons

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);        // Lights
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);        // Lights

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);        // Bluetooth Software Button
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);        // Bluetooth Reset
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);        // Bluetooth Status 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);        // Bluetooth Wake-Up

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);       // System Interrupt Timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);       // Global Tick Timer

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);        // UART Ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);        // UART Port 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);        // UART Port 1

    /* Configure Pad Buttons Interrupt */
    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, PORT_L);
    GPIOIntRegister(GPIO_PORTL_BASE, isrPadButtonCallback);
    GPIOIntTypeSet(GPIO_PORTL_BASE, PORT_L, GPIO_FALLING_EDGE);

    /* Configure Joystick Select Button Interrupt*/
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, PORT_C);
    GPIOIntRegister(GPIO_PORTC_BASE, isrStickButtonCallback);
    GPIOIntTypeSet(GPIO_PORTC_BASE, PORT_C, GPIO_FALLING_EDGE);

    /* Set pins 0 & 1 of GPIO port N to digital output */
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, PORT_F);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, PORT_N);

    /* Activate all lights */
    switchLights(true, true, true, true);

    /* ADC (Analog-Digital-Converter) configuration */
    SysCtlPeripheralDisable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    GPIOPinTypeADC(GPIO_PORTE_BASE, PORT_E);
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_EIGHTH, 1);

    ADCSequenceDisable(ADC0_BASE, ADC_SEQ);
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQ, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ, 0, ADC_CTL_CH9);
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQ, 1, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE);
    ADCIntRegister(ADC0_BASE, ADC_SEQ, isrStickCallback);
    ADCSequenceEnable(ADC0_BASE, ADC_SEQ);

    ADCIntClear(ADC0_BASE, ADC_SEQ);
    ADCIntEnable(ADC0_BASE, ADC_SEQ);

    IntEnable(INT_ADC0SS2);
    ADCProcessorTrigger(ADC0_BASE, ADC_SEQ);

    /* UART communication configuration */
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    UARTFlowControlSet(UART0_BASE, UART_FLOWCONTROL_NONE);
    UARTFIFODisable(UART0_BASE);
    UARTConfigSetExpClk(UART0_BASE, F_CPU, UART_BAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE);
    UARTEnable(UART0_BASE);

    uartWriteLine(UART0_BASE, "\r\n\r\nUART Connection Successful");

    /* System Interrupt Timer */
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC);
    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_PIOSC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, F_TIM / 1000);
    TimerLoadSet(TIMER1_BASE, TIMER_B, F_TIM / 1000);
    TimerPrescaleSet(TIMER1_BASE, TIMER_A, 100 - 1);
    TimerIntRegister(TIMER1_BASE, TIMER_A, isrTimerCallback);
    TimerPrescaleSet(TIMER1_BASE, TIMER_B, 100 - 1);
    TimerIntRegister(TIMER1_BASE, TIMER_B, isrTimerSendPacketCallback);

    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_B);

    /* Global Tick Timer */
    TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC);
    TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_PIOSC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, F_TIM / 10000);
    TimerPrescaleSet(TIMER2_BASE, TIMER_A, 10 - 1);
    TimerIntRegister(TIMER2_BASE, TIMER_A, isrTimerTickCallback);

    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER2_BASE, TIMER_A);


    /* Bluetooth Configuration */

    // Software Button
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, ~GPIO_PIN_2);

    // Reset und CTS
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_4, GPIO_PIN_4);
    GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_5);

    // Wake Up
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_7, GPIO_PIN_7);

    // This RTS and peripheral CTS
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PIN_4);

    // Status 2 & Status 1
    GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);

    // Configure UART
    SysCtlPeripheralDisable(SYSCTL_PERIPH_UART6);
    SysCtlPeripheralReset(SYSCTL_PERIPH_UART6);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART6));

    GPIOPinConfigure(GPIO_PP0_U6RX);
    GPIOPinConfigure(GPIO_PP1_U6TX);
    GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART6_BASE, UART_CLOCK_SYSTEM);
    UARTFlowControlSet(UART6_BASE, UART_FLOWCONTROL_NONE);

    UARTFIFOEnable(UART6_BASE);

    uint32_t txLevel, rxLevel;
    UARTFIFOLevelGet(UART6_BASE, &txLevel, &rxLevel);
    UARTFIFOLevelSet(UART6_BASE, txLevel, UART_FIFO_RX6_8);

    UARTIntRegister(UART6_BASE, isrBluetoothHandler);

    UARTConfigSetExpClk(UART6_BASE, F_CPU, UART_BAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE);
    UARTIntClear(UART6_BASE, UART_INT_RT | UART_INT_RX);
    UARTIntEnable(UART6_BASE, UART_INT_RT | UART_INT_RX);
    UARTEnable(UART6_BASE);

    uartWriteLine(UART0_BASE, "Bluetooth Configuration Successful");

    /* Enables CPU interrupts */
    GPIOIntEnable(GPIO_PORTL_BASE, PORT_L);
    GPIOIntEnable(GPIO_PORTC_BASE, PORT_C);
    IntMasterEnable();

    /* Console Communication */
    rn4678InitializeBluetooth();
    rn4678Connect();

    struct DataPackage armBuffer;
    armBuffer.stickX = 2000;
    armBuffer.stickY = 2000;
    armBuffer.pressedButtonS1 = false;
    armBuffer.pressedButtonS2 = false;
    armBuffer.speed = 0;
    armBuffer.arm = false;
    armBuffer.dirty = false;
    armBuffer.dirtyCounter = 0;

    sleep(15000);
    uartWriteLine(UART0_BASE, "Sending Initial Arm Packet");
    sendDataPacket(armBuffer);
    sleep(1000);

    initializedBluetooth = true;
    uartWriteLine(UART0_BASE, "Initialization Finished! Ready For Use!");
    uartWriteLine(UART0_BASE, "\r\nPress # to enter the serial command line interface");

    /* CLI communication and hold micro-controller state consistent */
    unsigned char commandInput[1];
    while(true) {
        ROM_SysCtlDelay(F_CPU / 3 / 100);
        uartReadChar(UART0_BASE, commandInput);

        if (commandInput[0] == '#') {
            inFlightSerialCommunication();
        }
    }
}
