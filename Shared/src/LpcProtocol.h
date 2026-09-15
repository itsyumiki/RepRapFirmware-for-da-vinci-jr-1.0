#ifndef LPC_PROTOCOL_H
#define LPC_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace LpcProtocol
{

constexpr uint8_t Version = 3;
constexpr uint8_t SyncByte = 0xA5;
constexpr size_t MaxPayload = 16;
constexpr size_t MaxEncodedFrame = MaxPayload + 4;

namespace Pins
{
	constexpr uint8_t FilamentRunout = 0x27;
	constexpr uint8_t Rotation = 0x21;
	constexpr uint8_t HotendFilament = 0x06;
	constexpr uint8_t StatusLed = 0x2A;
	constexpr uint8_t Heater = 0x09;
	constexpr uint8_t HotendFan = 0x25;
	constexpr uint8_t ReflowFan = 0x1A;
	constexpr uint8_t HotendNtc = 0x10;
}

enum class MessageType : uint8_t
{
	ping = 1,
	pong = 2,
	gpioConfig = 3,
	gpioWrite = 4,
	gpioState = 5,
	pwmWrite = 6,
	thermistorConfig = 7,
	heaterModelA = 8,
	heaterModelB = 9,
	heaterModelC = 10,
	heaterConfig = 11,
	heaterCommand = 12,
	thermalStatus = 13,
	heaterFeedForward = 14,
	configurationReset = 15,
	configurationComplete = 16,
	heaterTuningCommand = 17,
	heaterTuningReportA = 18,
	heaterTuningReportB = 19
};

// heaterTuningCommand payload (8 bytes), host -> LPC. Starts or cancels M303 autotune on the hotend heater.
// There is only ever one LPC heater, so no heater number is carried.
//   [0]    on:            1 to start tuning, 0 to cancel and return to the off state
//   [1]    pwm:           tuning PWM as a fraction of full scale packed into a byte (0-255 -> 0.0-1.0)
//   [2:3]  lowTemp:       hysteresis low temperature, signed centidegrees C (int16, little-endian)
//   [4:5]  highTemp:      tuning target temperature, signed centidegrees C (int16, little-endian)
//   [6:7]  peakTempDrop:  temperature drop after peak that ends the heating phase, unsigned centidegrees C (uint16, little-endian)

// heaterTuningReportA payload (14 bytes), LPC -> host. First half of one completed tuning cycle's data.
// Always immediately followed by heaterTuningReportB for the same cycle; the two are never reordered
// or interleaved with anything else on this point-to-point link, so no sequence tag is needed.
//   [0:1]   cyclesDone:  number of tuning cycles completed so far (uint16, little-endian)
//   [2:5]   ton:         time the heater was on during this cycle, milliseconds (uint32, little-endian)
//   [6:9]   toff:        time the heater was off during this cycle, milliseconds (uint32, little-endian)
//   [10:13] dlow:        time spent below the low threshold, milliseconds (uint32, little-endian)

// heaterTuningReportB payload (16 bytes), LPC -> host. Second half of one completed tuning cycle's data.
//   [0:3]   dhigh:        time spent above the high threshold, milliseconds (uint32, little-endian)
//   [4:7]   heatingRate:  measured heating rate, degrees C/sec (float, little-endian)
//   [8:11]  coolingRate:  measured cooling rate, degrees C/sec (float, little-endian)
//   [12:15] voltage:      supply voltage sample during this cycle, volts (float, little-endian). Zero if unavailable.

enum class GpioMode : uint8_t
{
	disabled = 0,
	input = 1,
	inputPullup = 2,
	output = 3,
	pwm = 4,
	analog = 5
};

enum class HeaterCommand : uint8_t
{
	off = 0,
	on = 1,
	suspend = 2,
	resetFault = 3
};

enum class HeaterState : uint8_t
{
	fault = 0,
	offline = 1,
	off = 2,
	suspended = 3,
	cooling = 4,
	stable = 5,
	heating = 6
};

enum class ThermalError : uint8_t
{
	none = 0,
	notConfigured = 1,
	adcTimeout = 2,
	shortCircuit = 3,
	openCircuit = 4,
	overTemperature = 5,
	underTemperature = 6,
	linkTimeout = 7,
	controlFault = 8,
	heatingTooSlow = 9,
	temperatureExcursion = 10
};

struct Frame
{
	MessageType type;
	uint8_t length;
	uint8_t payload[MaxPayload];
};

struct Decoder
{
	Frame frame;
	uint8_t state;
	uint8_t index;
	uint8_t crc;
};

void Reset(Decoder& decoder) noexcept;
bool Feed(Decoder& decoder, uint8_t byte, Frame& frame) noexcept;
size_t Encode(MessageType type, const uint8_t* payload, uint8_t length, uint8_t* output) noexcept;

}

#endif
