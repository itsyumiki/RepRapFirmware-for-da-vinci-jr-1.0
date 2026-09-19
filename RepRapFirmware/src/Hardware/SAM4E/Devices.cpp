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
#include <Interrupts.h>
#include <Stream.h>
#include <General/RingBuffer.h>

#if defined(DA_VINCI_JR) && HAS_WIFI_NETWORKING
class WiFiSoftwareUart final : public Stream
{
public:
	WiFiSoftwareUart() noexcept
	{
		rxBuffer.Init(64);
	}

	void Begin(uint32_t baud) noexcept
	{
		rxBuffer.Clear();
		bitCycles = (SystemCoreClockFreq + baud/2) / baud;
		enabled = true;
		SetPinMode(EspUartTxPin, OUTPUT_HIGH);
		SetPinMode(EspUartRxPin, INPUT_PULLUP);
		AttachPinInterrupt(EspUartRxPin, RxStart, InterruptMode::falling, CallbackParameter(this));
	}

	void End() noexcept
	{
		if (enabled)
		{
			DetachPinInterrupt(EspUartRxPin);
			enabled = false;
		}
		SetPinMode(EspUartTxPin, INPUT_PULLUP);
		SetPinMode(EspUartRxPin, INPUT_PULLUP);
	}

	int available() noexcept override
	{
		return (int)rxBuffer.ItemsPresent();
	}

	int read() noexcept override
	{
		uint8_t data;
		return rxBuffer.GetItem(data) ? data : -1;
	}

	void flush() noexcept override { }

	size_t canWrite() noexcept override
	{
		return enabled ? 1 : 0;
	}

	size_t write(uint8_t data) noexcept override
	{
		if (!enabled)
		{
			return 0;
		}
		AtomicCriticalSectionLocker lock;
		WriteByte(data);
		return 1;
	}

	size_t write(const uint8_t *_ecv_array data, size_t length) noexcept override
	{
		if (!enabled)
		{
			return 0;
		}

		AtomicCriticalSectionLocker lock;
		for (size_t i = 0; i < length; ++i)
		{
			WriteByte(data[i]);
		}
		return length;
	}

private:
	static void RxStart(CallbackParameter param) noexcept
	{
		static_cast<WiFiSoftwareUart*>(param.vp)->ReceiveByte();
	}

	void ReceiveByte() noexcept
	{
		AtomicCriticalSectionLocker lock;
		if (!enabled || digitalRead(EspUartRxPin))
		{
			return;
		}

		uint8_t data = 0;
		uint32_t sampleTime = DelayCycles(GetCurrentCycles(), bitCycles + bitCycles/2);
		for (unsigned int bit = 0; bit < 8; ++bit)
		{
			if (digitalRead(EspUartRxPin))
			{
				data |= (uint8_t)(1u << bit);
			}
			sampleTime = DelayCycles(sampleTime, bitCycles);
		}

		if (digitalRead(EspUartRxPin))
		{
			(void)rxBuffer.PutItem(data);
		}
	}

	void WriteByte(uint8_t data) noexcept
	{
		uint32_t transitionTime = GetCurrentCycles();
		digitalWrite(EspUartTxPin, false);
		for (unsigned int bit = 0; bit < 8; ++bit)
		{
			transitionTime = DelayCycles(transitionTime, bitCycles);
			digitalWrite(EspUartTxPin, (data & 1u) != 0);
			data >>= 1;
		}
		transitionTime = DelayCycles(transitionTime, bitCycles);
		digitalWrite(EspUartTxPin, true);
		(void)DelayCycles(transitionTime, bitCycles);
	}

	RingBuffer<uint8_t> rxBuffer;
	uint32_t bitCycles = 0;
	bool enabled = false;
};

static WiFiSoftwareUart wifiSoftwareUart;

Stream& GetWiFiUploadSerial() noexcept
{
	return wifiSoftwareUart;
}

void BeginWiFiUploadSerial(uint32_t baud) noexcept
{
	wifiSoftwareUart.Begin(baud);
}

void EndWiFiUploadSerial() noexcept
{
	wifiSoftwareUart.End();
}


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
