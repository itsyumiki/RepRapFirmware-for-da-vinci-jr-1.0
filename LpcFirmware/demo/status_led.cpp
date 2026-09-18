#include "Gpio.h"
#include "Lpc1115.h"

namespace
{

constexpr uint32_t CoreClock = 12000000u;
constexpr uint32_t BlinkIntervalMillis = 500u;
volatile uint32_t millisTicks;

}

extern "C" void SysTick_Handler() noexcept
{
	++millisTicks;
}

extern "C" int main() noexcept
{
	LPC_SYSCON_MAINCLKSEL = 0u;
	LPC_SYSCON_MAINCLKUEN = 0u;
	LPC_SYSCON_MAINCLKUEN = 1u;
	LPC_SYSCON_SYSAHBCLKDIV = 1u;

	Gpio::Init();
	(void)Gpio::Configure(LpcProtocol::Pins::StatusLed, LpcProtocol::GpioMode::output, false);

	SYST_RVR = CoreClock / 1000u - 1u;
	SYST_CVR = 0u;
	SYST_CSR = 0x07u;

	uint32_t lastToggle = 0;
	bool ledOn = false;
	for (;;)
	{
		if (millisTicks - lastToggle >= BlinkIntervalMillis)
		{
			lastToggle = millisTicks;
			ledOn = !ledOn;
			(void)Gpio::Write(LpcProtocol::Pins::StatusLed, ledOn);
		}
	}
}
