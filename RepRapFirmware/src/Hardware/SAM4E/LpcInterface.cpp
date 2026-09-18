#include "LpcInterface.h"

#if defined(DA_VINCI_JR)
#include "Devices.h"
#include <Heating/FOPDT.h>
#include <RepRapFirmware.h>

#include <cstring>

namespace LpcInterface
{

static LpcProtocol::Decoder decoder;
static LpcProtocol::GpioMode pinModes[NumLpcPins];
static bool pinValues[NumLpcPins];
static bool pinStateReceived[NumLpcPins];
static uint16_t pwmValues[NumLpcPins];
static uint16_t pwmFrequencies[NumLpcPins];
static bool online;
static uint32_t lastPingSent;
static uint32_t lastPongReceived;
static uint32_t connectionGeneration;
static Mutex transmitMutex;
static bool firmwareUpdateActive;

static uint8_t thermistorPayload[16];
static uint8_t modelAPayload[16];
static uint8_t modelBPayload[16];
static uint8_t modelCPayload[5];
static uint8_t heaterPayload[11];
static uint8_t feedForwardPayload[12];
static bool thermistorConfigured;
static int thermistorSensorNumber = -1;
static bool modelConfigured;
static bool heaterConfigured;
static bool feedForwardConfigured;
static bool thermalStatusReceived;
static ThermalStatus thermalStatus;
static uint32_t thermalStatusReceivedAt;

static void PutFloat(uint8_t* destination, float value) noexcept
{
	memcpy(destination, &value, sizeof(value));
}

static void PutU16(uint8_t* destination, uint16_t value) noexcept
{
	destination[0] = static_cast<uint8_t>(value);
	destination[1] = static_cast<uint8_t>(value >> 8);
}

static uint16_t ReadU16(const uint8_t* source) noexcept
{
	return static_cast<uint16_t>(source[0]) | (static_cast<uint16_t>(source[1]) << 8);
}

static int16_t ReadI16(const uint8_t* source) noexcept
{
	return static_cast<int16_t>(ReadU16(source));
}

static int16_t ToDeciDegrees(float value) noexcept
{
	const float scaled = value * 10.0f;
	return static_cast<int16_t>((scaled < -32768.0f) ? -32768.0f : (scaled > 32767.0f) ? 32767.0f : scaled);
}

static int16_t ToCentiDegrees(float value) noexcept
{
	const float scaled = value * 100.0f;
	return static_cast<int16_t>((scaled < -32768.0f) ? -32768.0f : (scaled > 32767.0f) ? 32767.0f : scaled);
}

static uint16_t ToUnsignedHundredths(float value) noexcept
{
	const float scaled = value * 100.0f;
	return static_cast<uint16_t>((scaled <= 0.0f) ? 0.0f : (scaled >= 65535.0f) ? 65535.0f : scaled);
}

static uint16_t ToUnsignedTenths(float value) noexcept
{
	const float scaled = value * 10.0f;
	return static_cast<uint16_t>((scaled <= 0.0f) ? 0.0f : (scaled >= 65535.0f) ? 65535.0f : scaled);
}

static void Send(LpcProtocol::MessageType type, const uint8_t* payload, uint8_t payloadLength) noexcept
{
	uint8_t encoded[LpcProtocol::MaxEncodedFrame];
	const size_t length = LpcProtocol::Encode(type, payload, payloadLength, encoded);
	MutexLocker lock(transmitMutex);
	if (!firmwareUpdateActive)
	{
		lpcUart.write(encoded, length);
	}
}

static void SendPing() noexcept
{
	const uint8_t payload[] = { LpcProtocol::Version };
	Send(LpcProtocol::MessageType::ping, payload, sizeof(payload));
	lastPingSent = millis();
}

static void SendGpioConfig(size_t index) noexcept
{
	const uint8_t payload[] = {
		GetLpcPinId(FirstLpcPin + index),
		static_cast<uint8_t>(pinModes[index]),
		static_cast<uint8_t>(pinValues[index])
	};
	Send(LpcProtocol::MessageType::gpioConfig, payload, sizeof(payload));
}

static void SendPwm(size_t index) noexcept
{
	const uint16_t duty = pwmValues[index];
	const uint16_t frequency = pwmFrequencies[index];
	const uint8_t payload[] = {
		GetLpcPinId(FirstLpcPin + index),
		static_cast<uint8_t>(duty),
		static_cast<uint8_t>(duty >> 8),
		static_cast<uint8_t>(frequency),
		static_cast<uint8_t>(frequency >> 8)
	};
	Send(LpcProtocol::MessageType::pwmWrite, payload, sizeof(payload));
}

static void ReplayConfiguration() noexcept
{
	MutexLocker lock(transmitMutex);
	Send(LpcProtocol::MessageType::configurationReset, nullptr, 0);
	for (size_t i = 0; i < NumLpcPins; ++i)
	{
		if (pinModes[i] != LpcProtocol::GpioMode::disabled)
		{
			SendGpioConfig(i);
			if (pinModes[i] == LpcProtocol::GpioMode::pwm && pwmFrequencies[i] != 0)
			{
				SendPwm(i);
			}
		}
	}
	if (thermistorConfigured) { Send(LpcProtocol::MessageType::thermistorConfig, thermistorPayload, sizeof(thermistorPayload)); }
	if (modelConfigured)
	{
		Send(LpcProtocol::MessageType::heaterModelA, modelAPayload, sizeof(modelAPayload));
		Send(LpcProtocol::MessageType::heaterModelB, modelBPayload, sizeof(modelBPayload));
		Send(LpcProtocol::MessageType::heaterModelC, modelCPayload, sizeof(modelCPayload));
	}
	if (heaterConfigured) { Send(LpcProtocol::MessageType::heaterConfig, heaterPayload, sizeof(heaterPayload)); }
	if (feedForwardConfigured) { Send(LpcProtocol::MessageType::heaterFeedForward, feedForwardPayload, sizeof(feedForwardPayload)); }
	Send(LpcProtocol::MessageType::configurationComplete, nullptr, 0);
}

static void SetOffline() noexcept
{
	TaskCriticalSectionLocker lock;
	online = false;
	for (bool& received : pinStateReceived)
	{
		received = false;
	}
	thermalStatusReceived = false;
}

static void HandlePong(const LpcProtocol::Frame& frame) noexcept
{
	if (frame.length != 2 || frame.payload[0] != LpcProtocol::Version)
	{
		SetOffline();
		return;
	}

	const bool reconnected = !IsOnline() || frame.payload[1] != 0;
	if (reconnected)
	{
		SetOffline();
	}
	{
		TaskCriticalSectionLocker lock;
		online = true;
		lastPongReceived = millis();
		if (reconnected)
		{
			++connectionGeneration;
		}
	}
	if (reconnected)
	{
		ReplayConfiguration();
	}
}

static void HandleGpioState(const LpcProtocol::Frame& frame) noexcept
{
	if (frame.length != 2)
	{
		return;
	}
	for (size_t i = 0; i < NumLpcPins; ++i)
	{
		if (GetLpcPinId(FirstLpcPin + i) == frame.payload[0])
		{
			pinValues[i] = frame.payload[1] != 0;
			pinStateReceived[i] = true;
			return;
		}
	}
}

static void HandleThermalStatus(const LpcProtocol::Frame& frame) noexcept
{
	if (frame.length != 8
		|| ReadU16(frame.payload + 2) >= 1024u
		|| frame.payload[6] > static_cast<uint8_t>(LpcProtocol::HeaterState::heating)
		|| frame.payload[7] > static_cast<uint8_t>(LpcProtocol::ThermalError::temperatureExcursion))
	{
		return;
	}
	TaskCriticalSectionLocker lock;
	thermalStatus.temperature = static_cast<float>(ReadI16(frame.payload)) * 0.01f;
	thermalStatus.rawAdc = ReadU16(frame.payload + 2);
	thermalStatus.averagePwm = static_cast<float>(ReadU16(frame.payload + 4)) * (1.0f / 65535.0f);
	thermalStatus.state = static_cast<LpcProtocol::HeaterState>(frame.payload[6]);
	thermalStatus.error = static_cast<LpcProtocol::ThermalError>(frame.payload[7]);
	thermalStatusReceivedAt = millis();
	thermalStatusReceived = true;
}

void Init() noexcept
{
	LpcProtocol::Reset(decoder);
	for (size_t i = 0; i < NumLpcPins; ++i)
	{
		pinModes[i] = LpcProtocol::GpioMode::disabled;
		pinValues[i] = false;
		pinStateReceived[i] = false;
		pwmValues[i] = 0;
		pwmFrequencies[i] = 0;
	}
	transmitMutex.Create("LPC");
	online = false;
	thermalStatusReceived = false;
	lastPongReceived = 0;
	SendPing();
}

void PrepareForFirmwareUpdate() noexcept
{
	MutexLocker lock(transmitMutex);
	firmwareUpdateActive = true;
	SetOffline();
	LpcProtocol::Reset(decoder);
	lpcUart.ClearReceiveBuffer();
}

void FirmwareUpdateFinished() noexcept
{
	MutexLocker lock(transmitMutex);
	SetOffline();
	LpcProtocol::Reset(decoder);
	lpcUart.ClearReceiveBuffer();
	lastPingSent = millis();
	firmwareUpdateActive = false;
}

void Spin() noexcept
{
	LpcProtocol::Frame frame;
	while (lpcUart.available() != 0)
	{
		const int value = lpcUart.read();
		if (value < 0 || !LpcProtocol::Feed(decoder, static_cast<uint8_t>(value), frame))
		{
			continue;
		}

		switch (frame.type)
		{
		case LpcProtocol::MessageType::pong:
			HandlePong(frame);
			break;
		case LpcProtocol::MessageType::gpioState:
			HandleGpioState(frame);
			break;
		case LpcProtocol::MessageType::thermalStatus:
			HandleThermalStatus(frame);
			break;
		default:
			break;
		}
	}

	const uint32_t now = millis();
	if (online && now - lastPongReceived >= 3000)
	{
		SetOffline();
	}
	if (now - lastPingSent >= 1000)
	{
		SendPing();
	}
}

bool IsOnline() noexcept
{
	TaskCriticalSectionLocker lock;
	return online;
}
uint32_t GetConnectionGeneration() noexcept
{
	TaskCriticalSectionLocker lock;
	return connectionGeneration;
}
bool IsPinAvailable(Pin pin) noexcept
{
	if (!IsOnline() || !IsLpcPin(pin))
	{
		return false;
	}
	const size_t index = pin - FirstLpcPin;
	if (pinModes[index] == LpcProtocol::GpioMode::input || pinModes[index] == LpcProtocol::GpioMode::inputPullup)
	{
		return pinStateReceived[index];
	}
	if (pinModes[index] == LpcProtocol::GpioMode::analog)
	{
		ThermalStatus status;
		return GetThermalStatus(status)
			&& status.error != LpcProtocol::ThermalError::notConfigured
			&& status.error != LpcProtocol::ThermalError::adcTimeout;
	}
	return true;
}

bool SetPinMode(Pin pin, PinMode mode) noexcept
{
	MutexLocker lock(transmitMutex);
	if (!IsLpcPin(pin))
	{
		return false;
	}

	const size_t index = pin - FirstLpcPin;
	const uint8_t capabilities = static_cast<uint8_t>(PinTable[pin].GetCapability());
	switch (mode)
	{
	case INPUT:
	case INPUT_PULLUP:
		if ((capabilities & static_cast<uint8_t>(PinCapability::read)) == 0)
		{
			return false;
		}
		pinModes[index] = (mode == INPUT) ? LpcProtocol::GpioMode::input : LpcProtocol::GpioMode::inputPullup;
		pinStateReceived[index] = false;
		break;
	case OUTPUT_LOW:
	case OUTPUT_HIGH:
		if ((capabilities & static_cast<uint8_t>(PinCapability::write)) == 0)
		{
			return false;
		}
		pinModes[index] = LpcProtocol::GpioMode::output;
		pinValues[index] = mode == OUTPUT_HIGH;
		break;
	case OUTPUT_PWM_LOW:
	case OUTPUT_PWM_HIGH:
		if ((capabilities & static_cast<uint8_t>(PinCapability::pwm)) == 0)
		{
			return false;
		}
		pinModes[index] = LpcProtocol::GpioMode::pwm;
		pinValues[index] = mode == OUTPUT_PWM_HIGH;
		break;
	case AIN:
		if ((capabilities & static_cast<uint8_t>(PinCapability::ain)) == 0)
		{
			return false;
		}
		pinModes[index] = LpcProtocol::GpioMode::analog;
		break;
	default:
		return false;
	}

	if (IsOnline())
	{
		SendGpioConfig(index);
	}
	return true;
}

bool ReadPin(Pin pin) noexcept
{
	return IsLpcPin(pin) && pinValues[pin - FirstLpcPin];
}

uint16_t ReadAnalog(Pin pin) noexcept
{
	return (IsLpcPin(pin) && GetLpcPinId(pin) == LpcProtocol::Pins::HotendNtc && thermalStatusReceived) ? thermalStatus.rawAdc : 0;
}

void WritePin(Pin pin, bool high) noexcept
{
	MutexLocker lock(transmitMutex);
	if (!IsLpcPin(pin))
	{
		return;
	}
	const size_t index = pin - FirstLpcPin;
	pinValues[index] = high;
	if (IsOnline())
	{
		const uint8_t payload[] = { GetLpcPinId(pin), static_cast<uint8_t>(high) };
		Send(LpcProtocol::MessageType::gpioWrite, payload, sizeof(payload));
	}
}

void WritePwm(Pin pin, float duty, uint16_t frequency) noexcept
{
	MutexLocker lock(transmitMutex);
	if (!IsLpcPin(pin))
	{
		return;
	}
	const size_t index = pin - FirstLpcPin;
	const float constrainedDuty = (duty <= 0.0f) ? 0.0f : (duty >= 1.0f) ? 1.0f : duty;
	pwmValues[index] = static_cast<uint16_t>(constrainedDuty * 65535.0f + 0.5f);
	pwmFrequencies[index] = frequency;
	if (IsOnline())
	{
		SendPwm(index);
	}
}

void ConfigureThermistor(unsigned int sensorNumber, float r25, float beta, float coefficientC, float seriesResistance) noexcept
{
	MutexLocker lock(transmitMutex);
	PutFloat(thermistorPayload, r25);
	PutFloat(thermistorPayload + 4, beta);
	PutFloat(thermistorPayload + 8, coefficientC);
	PutFloat(thermistorPayload + 12, seriesResistance);
	thermistorSensorNumber = static_cast<int>(sensorNumber);
	thermistorConfigured = true;
	if (IsOnline())
	{
		Send(LpcProtocol::MessageType::thermistorConfig, thermistorPayload, sizeof(thermistorPayload));
	}
}

void UnregisterThermistor(unsigned int sensorNumber) noexcept
{
	MutexLocker lock(transmitMutex);
	if (thermistorSensorNumber == static_cast<int>(sensorNumber))
	{
		CommandHeater(LpcProtocol::HeaterCommand::off, 0.0f);
		thermistorSensorNumber = -1;
		thermistorConfigured = false;
	}
}

bool IsThermistorSensor(unsigned int sensorNumber) noexcept
{
	MutexLocker lock(transmitMutex);
	return thermistorConfigured && thermistorSensorNumber == static_cast<int>(sensorNumber);
}

void ConfigureHeaterModel(const FopDt& model) noexcept
{
	MutexLocker lock(transmitMutex);
	PutFloat(modelAPayload, model.GetHeatingRate());
	PutFloat(modelAPayload + 4, model.GetBasicCoolingRate());
	PutFloat(modelAPayload + 8, model.GetFanCoolingRate());
	PutFloat(modelAPayload + 12, model.GetCoolingRateExponent());
	const PidParameters& pid = model.GetPidParameters(false);
	PutFloat(modelBPayload, model.GetDeadTime());
	PutFloat(modelBPayload + 4, model.GetMaxPwm());
	PutFloat(modelBPayload + 8, pid.kP);
	PutFloat(modelBPayload + 12, pid.recipTi);
	PutFloat(modelCPayload, pid.tD);
	modelCPayload[4] = static_cast<uint8_t>((model.UsePid() ? 0x01u : 0u) | (model.IsInverted() ? 0x02u : 0u) | (model.ArePidParametersOverridden() ? 0x04u : 0u));
	modelConfigured = true;
	if (IsOnline())
	{
		Send(LpcProtocol::MessageType::heaterModelA, modelAPayload, sizeof(modelAPayload));
		Send(LpcProtocol::MessageType::heaterModelB, modelBPayload, sizeof(modelBPayload));
		Send(LpcProtocol::MessageType::heaterModelC, modelCPayload, sizeof(modelCPayload));
	}
}

void ConfigureHeater(uint16_t frequency, float upperLimit, float lowerLimit, float maxExcursion, float maxFaultTime, uint8_t maxBadReadings) noexcept
{
	MutexLocker lock(transmitMutex);
	PutU16(heaterPayload, frequency);
	PutU16(heaterPayload + 2, static_cast<uint16_t>(ToDeciDegrees(upperLimit)));
	PutU16(heaterPayload + 4, static_cast<uint16_t>(ToDeciDegrees(lowerLimit)));
	PutU16(heaterPayload + 6, ToUnsignedHundredths(maxExcursion));
	PutU16(heaterPayload + 8, ToUnsignedTenths(maxFaultTime));
	heaterPayload[10] = maxBadReadings;
	heaterConfigured = true;
	if (IsOnline())
	{
		Send(LpcProtocol::MessageType::heaterConfig, heaterPayload, sizeof(heaterPayload));
	}
}

void CommandHeater(LpcProtocol::HeaterCommand command, float targetTemperature) noexcept
{
	const int16_t target = ToCentiDegrees(targetTemperature);
	const uint8_t payload[] = {
		static_cast<uint8_t>(command),
		static_cast<uint8_t>(target),
		static_cast<uint8_t>(static_cast<uint16_t>(target) >> 8)
	};
	if (command == LpcProtocol::HeaterCommand::off || IsOnline())
	{
		Send(LpcProtocol::MessageType::heaterCommand, payload, sizeof(payload));
	}
}

void ConfigureHeaterFeedForward(float fanPwm, float extrusionPwmBoost, float extrusionTemperatureBoost) noexcept
{
	MutexLocker lock(transmitMutex);
	PutFloat(feedForwardPayload, fanPwm);
	PutFloat(feedForwardPayload + 4, extrusionPwmBoost);
	PutFloat(feedForwardPayload + 8, extrusionTemperatureBoost);
	feedForwardConfigured = true;
	if (IsOnline())
	{
		Send(LpcProtocol::MessageType::heaterFeedForward, feedForwardPayload, sizeof(feedForwardPayload));
	}
}

bool GetThermalStatus(ThermalStatus& status) noexcept
{
	TaskCriticalSectionLocker lock;
	if (!online || !thermalStatusReceived || millis() - thermalStatusReceivedAt >= 1000)
	{
		return false;
	}
	status = thermalStatus;
	return true;
}

}

#endif
