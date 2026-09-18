/*
 * Devices.cpp
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#include "Devices.h"
#include <RepRapFirmware.h>
#include <AnalogIn.h>
#include <AnalogOut.h>

AsyncSerial lpcUart(UART1, UART1_IRQn, ID_UART1, 256, 256,
	[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });

void UART1_Handler() noexcept
{
	lpcUart.IrqHandler();
}

static void LpcUartInit() noexcept
{
	SetPinFunction(LpcUartRxPin, LpcUartPinFunction);
	SetPinFunction(LpcUartTxPin, LpcUartPinFunction);
	EnablePullup(LpcUartRxPin);
	static_assert(NvicPriorityAuxUart >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	lpcUart.setInterruptPriority(NvicPriorityAuxUart);
	lpcUart.begin(LpcUartBaudRate);
}

SerialCDC serialUSB;

void SdhcInit() noexcept
{
	SetPinFunction(HsmciClockPin, HsmciPinsFunction);
	for (Pin p : HsmciOtherPins)
	{
		SetPinFunction(p, HsmciPinsFunction);
		EnablePullup(p);
	}
}

// Device initialisation
void DeviceInit() noexcept
{
	LegacyAnalogIn::AnalogInInit();
	AnalogOut::Init();
	LpcUartInit();
	SdhcInit();
}

void StopAnalogTask() noexcept
{
}

void StopUsbTask() noexcept
{
}

// End
