#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/hw_bmp180.h"
#include "sensorlib/bmp180.h"


#define MPU9150_I2C_ADDRESS     0x68
#define BMP180_I2C_ADDRESS      0x77


tI2CMInstance g_sI2CInst; // Global instance structure for the I2C master driver.
tMPU9150 g_sMPU9150Inst; // Global instance structure for the ISL29023 sensor driver.
tBMP180 g_sBMP180Inst; // Global instance structure for the BMP180 sensor driver.
uint32_t g_ui32PrintSkipCounter; // counter being checked against PRINT_SKIP_COUNT

volatile uint_fast8_t g_vui8I2CDoneFlag; // Global flags to alert main that MPU9150 I2C transaction is complete
volatile uint_fast8_t g_vui8ErrorFlag; // Global flags to alert main that MPU9150 I2C transaction error has occurred.
volatile uint_fast8_t g_vui8DataFlag; // Global flags to alert main that MPU9150 data is ready to be retrieved.

uint_fast32_t m_pressureValue;
uint_fast32_t m_tempValue;


//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


//*****************************************************************************
// The Hibernate Interrupt Handler, used to get the RTC values in second
//*****************************************************************************
void HibernateHandler(void)
{
}


//*****************************************************************************
// BMP180 Sensor callback function.  Called at the end of BMP180 sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//*****************************************************************************
void BMP180AppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
        g_vui8DataFlag = 1;
}


//*****************************************************************************
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//*****************************************************************************
void MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    if(ui8Status == I2CM_STATUS_SUCCESS)
        g_vui8I2CDoneFlag = 1;

    // Store the most recent status in case it was an error condition
    g_vui8ErrorFlag = ui8Status;
}


//*****************************************************************************
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//*****************************************************************************
void IntGPIOb(void)
{
    unsigned long ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    // Clear all the pin interrupts that are set
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
        // MPU9150 Data is ready for retrieval and processing.
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
}


//*****************************************************************************
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//*****************************************************************************
void MPU9150I2CIntHandler(void)
{
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    I2CMIntHandler(&g_sI2CInst);
}


//*****************************************************************************
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//*****************************************************************************
void MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    // Set terminal color to red and print error status and locations
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n", g_vui8ErrorFlag, pcFilename, ui32Line);

    // Return terminal color to normal
    UARTprintf("\033[0m");

    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    while(1);
}


//*****************************************************************************
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//*****************************************************************************
void MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0));

    // If an error occurred call the error handler immediately.
    if(g_vui8ErrorFlag)
        MPU9150AppErrorHandler(pcFilename, ui32Line);

    // clear the data flag for next use.
    g_vui8I2CDoneFlag = 0;
}

//*****************************************************************************
// Set up the port for the LED, Port F
//*****************************************************************************
void EnableLED(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // PF1-> RED || PF2-> BLUE || PF3-> GREEN
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
}


//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//*****************************************************************************
void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // change the flow control, must also change it on Putty
    UARTFlowControlSet(UART0_BASE,UART_FLOWCONTROL_RX);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 230400, 16000000);
}


//*****************************************************************************
// Configure the I2C and its pins.  This better be called before MPU9512
//*****************************************************************************
void ConfigureI2C(void)
{
	// The I2C3 peripheral must be enabled before use.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

	// Select the I2C function for these pins.  This function will also
	// configure the GPIO pins pins for I2C operation, setting them to
	// open-drain operation with weak pull-ups.  Consult the data sheet
	// to see which functions are allocated per pin.
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
}


//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//*****************************************************************************
void ConfigureMPU9150Interrupt(void)
{
	// Configure and Enable the GPIO interrupt. Used for INT signal from the
	// MPU9150
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
	ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
	ROM_IntEnable(INT_GPIOB);
}


void SetUpPeripheralsInterrupts(void)
{
	// Keep only some parts of the systems running while in sleep mode.
	// GPIOB is for the MPU9150 interrupt pin.
	// UART0 is the virtual serial port
	// TIMER0, TIMER1 and WTIMER5 are used by the RGB driver
	// I2C3 is the I2C interface to the ISL29023
	ROM_SysCtlPeripheralClockGating(true);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_WTIMER5);

	// Enable interrupts to the processor.
	ROM_IntMasterEnable();
}


//*****************************************************************************
// Initialize all sensors inputs such as filters and settings, very specific
// to the appliation, rather than the generic initalization in the Configure fcn
//*****************************************************************************
void InitializeMPU9150(void)
{
	// Initialize the MPU9150 Driver.
	MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS, MPU9150AppCallback, &g_sMPU9150Inst);
	// Wait for transaction to complete
	MPU9150AppI2CWait(__FILE__, __LINE__);

	// Write application specifice sensor configuration such as filter settings and sensor range settings.
	g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_260_256; // ?? Hz acc/gyro bandwidth
	g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_2000; // Gyro max range of +-2000 dps
	g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ | MPU9150_ACCEL_CONFIG_AFS_SEL_16G); // High Pass filter of 5Hz and max range for +-16g for acc
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);

	// Configure the data ready interrupt pin output of the MPU9150.
	g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL | MPU9150_INT_PIN_CFG_INT_RD_CLEAR | MPU9150_INT_PIN_CFG_LATCH_INT_EN;
	g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
	MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG, g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);

    // Modifying the ranges of the sensors
    // hack because it seems that it doesn't work otherwise
    // done after all possible configurations are done
    // TODO: redundant, FIXIT! (look up MPU9150Init function in mpu9150.C)
    MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_ACCEL_CONFIG, ~MPU9150_ACCEL_CONFIG_AFS_SEL_M, MPU9150_ACCEL_CONFIG_AFS_SEL_16G, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
	MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_GYRO_CONFIG, ~MPU9150_GYRO_CONFIG_FS_SEL_M, MPU9150_GYRO_CONFIG_FS_SEL_2000, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
}


//*****************************************************************************
//*****************************************************************************
void InitializeBMP180(void)
{
	g_vui8DataFlag = 0;
	BMP180Init(&g_sBMP180Inst, &g_sI2CInst, BMP180_I2C_ADDRESS, BMP180AppCallback, &g_sBMP180Inst);
	// Wait for initialization callback to indicate reset request is complete.
	while(g_vui8DataFlag == 0); // Wait for I2C Transactions to complete.
	g_vui8DataFlag = 0;
	BMP180ReadModifyWrite(&g_sBMP180Inst, BMP180_O_CTRL_MEAS, ~BMP180_CTRL_MEAS_OSS_M, BMP180_CTRL_MEAS_OSS_8, BMP180AppCallback, &g_sBMP180Inst);
	while(g_vui8DataFlag == 0);
}


//*****************************************************************************
// Set the Sampling Rate of the sensors to the registers
//*****************************************************************************
void SetMpuSampleRate(int16_t sampleRate)
{
	// Read Current Gyro sampling rate, either 8k or 1k by checking the Low pass filter status
	uint8_t dataRead;
	MPU9150Read(&g_sMPU9150Inst, MPU9150_O_CONFIG, &dataRead, 1, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);

	// isolate to get the last 3 bits to get the LPF status
	dataRead |= 0b111;

	// check if rate == 8k or 1k Hz, which corresponds to (1 or 7 for 8k) and others for 1 k
	// calculate the sampling rate using the formula provided in the data sheet
	// Sample Rate = Gyroscope Output Rate / (1 + sample rate divider)
	uint8_t sampleRateDivider;
	if (dataRead == 0 || dataRead == 7)
		sampleRateDivider = (8000 / sampleRate) - 1;
	else
		sampleRateDivider = (1000 / sampleRate) - 1;

	// write to the sample rate divider register
	MPU9150ReadModifyWrite(&g_sMPU9150Inst, MPU9150_O_SMPLRT_DIV, 0, sampleRateDivider, MPU9150AppCallback, &g_sMPU9150Inst);
	MPU9150AppI2CWait(__FILE__, __LINE__);
}


//*****************************************************************************
// All the work needed to fetch the Pressure sensor and return its data
//*****************************************************************************
uint_fast32_t GetPressureSensorValue(void)
{
	g_vui8DataFlag = 0;
	BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
	while(g_vui8DataFlag == 0); // Wait for the new data set to be available.

	uint_fast32_t pressureValue;
	BMP180DataPressureGetRaw(&g_sBMP180Inst, &pressureValue);
	return pressureValue;
}


//*****************************************************************************
// Get the 16 bytes results from the sensor and split it to two 8 bits to send
// it over UART
//*****************************************************************************
void Divide16BitTo8Bit(uint_fast16_t data16Bit[3], uint_fast8_t data8Bit[6])
{
	int_least8_t i = 0;
	for (; i<3; ++i)
	{
		// get the right most 8 bits from the 16 bits and save it to the
		// odd indices of the array for order consistency
		data8Bit[i*2+1] = (uint_fast8_t) data16Bit[i] & 255;
		data8Bit[i*2] = (uint_fast8_t) (data16Bit[i] >> 8);
	}
}



//*****************************************************************************
// Main application entry point.
//*****************************************************************************
int main(void)
{
    uint_fast16_t data16[3];
    uint_fast8_t data8[6];
    int_least8_t i;
    int_least8_t isDataRead = 0;
    uint8_t ledToggler = 0;

    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enable port B used for motion interrupt.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Initialize the UART.
    ConfigureUART();
    EnableLED();

    ConfigureI2C();
    ConfigureMPU9150Interrupt();
    SetUpPeripheralsInterrupts();

    // Initialize I2C3 peripheral.
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff, ROM_SysCtlClockGet());
    InitializeMPU9150();
    InitializeBMP180();

	// Set the Sampling Rate of the MPU-9150 Sampling and the Interrupt Rate
    SetMpuSampleRate(260);

    // Get an initial value of pressure sensor to use it while 9-axis is updating
    g_vui8DataFlag = 0;
	BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
	while(g_vui8DataFlag == 0); // Wait for the new data set to be available.
	BMP180DataPressureGetRaw(&g_sBMP180Inst, &m_pressureValue);
	g_vui8DataFlag = 0;

	while(1)
	{
		// Go to sleep mode while waiting for data ready.
		while(!g_vui8I2CDoneFlag)
			ROM_SysCtlSleep();

		// Clear the flag
		g_vui8I2CDoneFlag = 0;

		// Toggle the LED whenever there is a new reading: Blue LED is ON/OFF
		// incremement the counter, check odd or even for toggling, then multiply by ON value
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, (((++ledToggler) % 2) == 0) * GPIO_PIN_2);

		// add a Carriage Return '\r' at the end to keep add the timestamp
		UARTCharPut(UART0_BASE, '\r');
		UARTCharPut(UART0_BASE, '\n');

		// Get the Raw Data from the Accelerometer with no performace wasted on calculations
		MPU9150DataAccelGetRaw(&g_sMPU9150Inst, &data16[0], &data16[1], &data16[2]);
		// Divide the 16 bits data into 8 bits array
		Divide16BitTo8Bit(data16, data8);
		// print byte per byte through UART
		for (i=0; i<6; ++i)
			UARTCharPut(UART0_BASE, data8[i]);

		MPU9150DataGyroGetRaw(&g_sMPU9150Inst, &data16[0], &data16[1], &data16[2]);
		Divide16BitTo8Bit(data16, data8);
		for (i=0; i<6; ++i)
			UARTCharPut(UART0_BASE, data8[i]);

		MPU9150DataMagnetoGetRaw(&g_sMPU9150Inst, &data16[0], &data16[1], &data16[2]);
		Divide16BitTo8Bit(data16, data8);
		for (i=0; i<6; ++i)
			UARTCharPut(UART0_BASE, data8[i]);

		// Make one call to get the latest Pressure readings
		// Pressure takes a longer time and has a lower sampling rate than the IMU
		if (g_vui8DataFlag == 0 && isDataRead == 0)
		{
			BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
			isDataRead = 1;
		}
		// Whether there is new data or not, print the existing RAW pressure and Temperature
		else
		{
			BMP180DataPressureGetRaw(&g_sBMP180Inst, &m_pressureValue);
			BMP180DataTemperatureGetRaw(&g_sBMP180Inst, &m_tempValue);
			isDataRead = 0;
			g_vui8DataFlag = 0;
		}

		// print the 24 bits value of the pressure through bitmasking and casting to individual bytes
		for (i=2; i>=0; --i)
		{
			uint_fast8_t val2Print = (uint_fast8_t) (m_pressureValue >> (8*i)) & 0xFF;
			UARTCharPut(UART0_BASE, val2Print);
		}
		// print the 16 bits value of the temperature through bitmasking and casting to individual bytes
		for (i=1; i>=0; --i)
		{
			uint_fast8_t val2Print = (uint_fast8_t) (m_tempValue >> (8*i)) & 0xFF;
			UARTCharPut(UART0_BASE, val2Print);
		}

    }
}


