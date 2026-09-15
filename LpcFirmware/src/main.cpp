#include "Gpio.h"
#include "Pwm.h"
#include "Thermal.h"
#include "Uart.h"
#include <LpcProtocol.h>

static void Send(LpcProtocol::MessageType type, const uint8_t* payload, uint8_t payloadLength) noexcept
{
	uint8_t encoded[LpcProtocol::MaxEncodedFrame];
	const size_t length = LpcProtocol::Encode(type, payload, payloadLength, encoded);
	Uart::Write(encoded, length);
}

extern "C" int main() noexcept
{
	Uart::Init();
	Gpio::Init();
	Thermal::Init();

	LpcProtocol::Decoder decoder{};
	LpcProtocol::Reset(decoder);
	LpcProtocol::Frame frame{};
	bool needsConfiguration = true;

	for (;;)
	{
		uint8_t byte;
		while (Uart::Read(byte))
		{
			Thermal::Spin();
			if (!LpcProtocol::Feed(decoder, byte, frame))
			{
				continue;
			}

			switch (frame.type)
			{
			case LpcProtocol::MessageType::ping:
				if (frame.length == 1 && frame.payload[0] == LpcProtocol::Version)
				{
					Thermal::HostHeartbeat();
					const uint8_t payload[] = { LpcProtocol::Version, static_cast<uint8_t>(needsConfiguration) };
					Send(LpcProtocol::MessageType::pong, payload, sizeof(payload));
				}
				break;

			case LpcProtocol::MessageType::configurationReset:
				if (frame.length == 0)
				{
					needsConfiguration = true;
					Gpio::Init();
					Thermal::ResetConfiguration();
				}
				break;

			case LpcProtocol::MessageType::configurationComplete:
				if (frame.length == 0)
				{
					needsConfiguration = false;
				}
				break;

			case LpcProtocol::MessageType::gpioConfig:
				if (frame.length == 3 && frame.payload[0] != LpcProtocol::Pins::Heater)
				{
					Gpio::Configure(frame.payload[0], static_cast<LpcProtocol::GpioMode>(frame.payload[1]), frame.payload[2] != 0);
				}
				break;

			case LpcProtocol::MessageType::gpioWrite:
				if (frame.length == 2 && frame.payload[0] != LpcProtocol::Pins::Heater)
				{
					Gpio::Write(frame.payload[0], frame.payload[1] != 0);
				}
				break;

			case LpcProtocol::MessageType::pwmWrite:
				if (frame.length == 5 && frame.payload[0] != LpcProtocol::Pins::Heater)
				{
					const uint16_t duty = static_cast<uint16_t>(frame.payload[1] | (static_cast<uint16_t>(frame.payload[2]) << 8));
					const uint16_t frequency = static_cast<uint16_t>(frame.payload[3] | (static_cast<uint16_t>(frame.payload[4]) << 8));
					Pwm::Set(frame.payload[0], duty, frequency);
				}
				break;

			default:
				Thermal::HandleFrame(frame);
				break;
			}
		}

		Thermal::Spin();
		if (!needsConfiguration)
		{
			uint8_t pin;
			bool value;
			while (Gpio::Poll(pin, value))
			{
				const uint8_t payload[] = { pin, static_cast<uint8_t>(value) };
				Send(LpcProtocol::MessageType::gpioState, payload, sizeof(payload));
			}

			uint8_t thermalPayload[LpcProtocol::MaxPayload];
			size_t thermalLength;
			if (Thermal::TakeStatus(thermalPayload, thermalLength))
			{
				Send(LpcProtocol::MessageType::thermalStatus, thermalPayload, static_cast<uint8_t>(thermalLength));
			}

			uint8_t tuningPayloadA[LpcProtocol::MaxPayload];
			uint8_t tuningPayloadB[LpcProtocol::MaxPayload];
			size_t tuningLengthA;
			size_t tuningLengthB;
			if (Thermal::TakeTuningReport(tuningPayloadA, tuningLengthA, tuningPayloadB, tuningLengthB))
			{
				Send(LpcProtocol::MessageType::heaterTuningReportA, tuningPayloadA, static_cast<uint8_t>(tuningLengthA));
				Send(LpcProtocol::MessageType::heaterTuningReportB, tuningPayloadB, static_cast<uint8_t>(tuningLengthB));
			}
		}
	}
}
