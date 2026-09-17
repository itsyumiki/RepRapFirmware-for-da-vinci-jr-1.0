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

#if defined(DA_VINCI_JR) && HAS_WIFI_NETWORKING
static void ParkSharedSpiFlash() noexcept
{
	constexpr uint8_t DeepPowerDownCommand = 0xB9;

	SetPinMode(APIN_ESP_SPI_SS0, OUTPUT_HIGH);
	SetPinMode(APIN_ESP_SPI_SCK, OUTPUT_LOW);
	SetPinMode(APIN_ESP_SPI_MOSI, OUTPUT_LOW);
	delayMicroseconds(1);

	digitalWrite(APIN_ESP_SPI_SS0, false);
	for (uint8_t mask = 0x80; mask != 0; mask >>= 1)
	{
		digitalWrite(APIN_ESP_SPI_MOSI, (DeepPowerDownCommand & mask) != 0);
		delayMicroseconds(1);
		digitalWrite(APIN_ESP_SPI_SCK, true);
		delayMicroseconds(1);
		digitalWrite(APIN_ESP_SPI_SCK, false);
	}
	digitalWrite(APIN_ESP_SPI_SS0, true);
	delayMicroseconds(10);

	SetPinMode(APIN_ESP_SPI_MOSI, INPUT);
	SetPinMode(APIN_ESP_SPI_SCK, INPUT);
	SetPinMode(APIN_ESP_SPI_SS0, INPUT_PULLUP);
}
#endif

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
#if defined(DA_VINCI_JR) && HAS_WIFI_NETWORKING
	SetPinMode(EspResetPin, OUTPUT_LOW);
	SetPinMode(EspEnablePin, OUTPUT_LOW);
	ParkSharedSpiFlash();
#endif
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
