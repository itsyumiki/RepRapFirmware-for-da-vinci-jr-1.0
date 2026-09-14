#include "Thermal.h"

#include "Gpio.h"
#include "Lpc1115.h"
#include "Pwm.h"

#include <math.h>
#include <string.h>

namespace Thermal
{

constexpr uint32_t CoreClock = 12000000u;
constexpr uint32_t SampleIntervalMillis = 250u;
constexpr uint32_t LinkTimeoutMillis = 2500u;
constexpr float AbsoluteZero = -273.15f;
constexpr float MinimumConnectedTemperature = -5.0f;
constexpr float NormalAmbientTemperature = 25.0f;
constexpr float TemperatureCloseEnough = 1.5f;
constexpr float MaxAmbientTemperature = 45.0f;
constexpr float FanFeedForwardMultiplier = 0.7f;
constexpr uint8_t HeaterPin = LpcProtocol::Pins::Heater;
constexpr uint16_t AdcRange = 1024u;

struct Model
{
	float heatingRate;
	float basicCoolingRate;
	float fanCoolingRate;
	float coolingRateExponent;
	float deadTime;
	float maxPwm;
	float overrideKp;
	float overrideRecipTi;
	float overrideTd;
	bool usePid;
	bool inverted;
	bool pidOverridden;
};

struct Pid
{
	float kP;
	float recipTi;
	float tD;
};

static volatile uint32_t millisTicks;
static volatile uint32_t lastHostHeartbeat;
static volatile bool statusDirty;
static uint32_t lastSampleTime;
static uint32_t excursionFaultMillis;
static uint32_t heatingFaultMillis;
static uint32_t heatingReferenceMillis;
static uint32_t timeSetHeating;
static uint16_t lastRawAdc;
static uint16_t heaterFrequency = 250;
static uint8_t maxBadReadings = 3;
static uint8_t badReadings;
static float r25;
static float beta;
static float shC;
static float pullupR;
static float shA;
static float shB;
static float temperature;
static float targetTemperature;
static float upperLimit;
static float lowerLimit;
static float maxTempExcursion;
static float maxFaultTime;
static float fanPwm;
static float extrusionPwmBoost;
static float extrusionTemperatureBoost;
static float lastExtrusionTemperatureBoost;
static float integral;
static float averagePwm;
static float lastPwm;
static float heatingReferenceTemperature;
static float previousTemperatures[4];
static uint8_t previousIndex;
static uint8_t goodTemperatureMask;
static bool thermistorConfigured;
static bool heaterConfigured;
static bool modelConfigured;
static Model model;
static Model pendingModel;
static uint8_t pendingModelParts;
static volatile LpcProtocol::HeaterState state;
static volatile LpcProtocol::ThermalError error;
static volatile bool linkTimeoutLatched;

static float ReadFloat(const uint8_t* data) noexcept
{
	float value;
	memcpy(&value, data, sizeof(value));
	return value;
}

static int16_t ReadI16(const uint8_t* data) noexcept
{
	return static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}

static uint16_t ReadU16(const uint8_t* data) noexcept
{
	return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

static float Clamp(float value, float low, float high) noexcept
{
	return (value < low) ? low : (value > high) ? high : value;
}

static bool ModelReady(const Model& candidate) noexcept
{
	return isfinite(candidate.heatingRate) && candidate.heatingRate > 0.0f
		&& isfinite(candidate.basicCoolingRate) && candidate.basicCoolingRate >= 0.0f
		&& isfinite(candidate.fanCoolingRate) && candidate.fanCoolingRate >= 0.0f
		&& isfinite(candidate.coolingRateExponent) && candidate.coolingRateExponent >= 1.0f && candidate.coolingRateExponent <= 1.6f
		&& isfinite(candidate.deadTime) && candidate.deadTime > 0.0f
		&& isfinite(candidate.maxPwm) && candidate.maxPwm > 0.0f && candidate.maxPwm <= 1.0f
		&& (!candidate.pidOverridden || (isfinite(candidate.overrideKp) && isfinite(candidate.overrideRecipTi) && isfinite(candidate.overrideTd)));
}

static uint32_t Millis() noexcept;

static bool LinkAlive() noexcept
{
	return Millis() - lastHostHeartbeat <= LinkTimeoutMillis;
}

static bool Active() noexcept
{
	return state >= LpcProtocol::HeaterState::cooling;
}

static bool InPidMode() noexcept
{
	return state >= LpcProtocol::HeaterState::cooling && state <= LpcProtocol::HeaterState::heating;
}

static void UpdateHeaterState(float target) noexcept
{
	const LpcProtocol::HeaterState newState = (temperature + TemperatureCloseEnough < target)
		? LpcProtocol::HeaterState::heating
		: (temperature > target + TemperatureCloseEnough)
			? LpcProtocol::HeaterState::cooling
			: LpcProtocol::HeaterState::stable;
	if (newState != state)
	{
		if (newState == LpcProtocol::HeaterState::heating)
		{
			heatingReferenceTemperature = temperature;
			heatingReferenceMillis = timeSetHeating = Millis();
		}
		heatingFaultMillis = 0;
		excursionFaultMillis = 0;
		state = newState;
	}
}

static void ApplyHeater(float pwm) noexcept
{
	const bool timeoutBeforeWrite = linkTimeoutLatched;
	const float limited = timeoutBeforeWrite ? 0.0f : Clamp(pwm, 0.0f, 1.0f);
	const uint16_t duty = static_cast<uint16_t>(limited * 65535.0f + 0.5f);
	(void)Pwm::Set(HeaterPin, duty, heaterFrequency);
	if (!timeoutBeforeWrite && linkTimeoutLatched)
	{
		(void)Pwm::Set(HeaterPin, 0, heaterFrequency);
	}
}

static void SetFault(LpcProtocol::ThermalError newError) noexcept
{
	ApplyHeater(0.0f);
	state = LpcProtocol::HeaterState::fault;
	error = newError;
	statusDirty = true;
}

static bool ReadAdc(uint16_t& value) noexcept
{
	constexpr uint32_t control = (1u << 1) | (2u << 8); // AD1, 12MHz/(2+1) = 4MHz ADC clock
	LPC_ADC_CR = control | (1u << 24);
	for (uint32_t timeout = 0; timeout < 10000; ++timeout)
	{
		const uint32_t reading = LPC_ADC_DR1;
		if ((reading & (1u << 31)) != 0)
		{
			value = static_cast<uint16_t>((reading >> 6) & 0x03FFu);
			LPC_ADC_CR = control;
			return true;
		}
	}
	LPC_ADC_CR = control;
	return false;
}

static bool SampleTemperature() noexcept
{
	if (!thermistorConfigured)
	{
		error = LpcProtocol::ThermalError::notConfigured;
		return false;
	}

	uint32_t sum = 0;
	for (unsigned int i = 0; i < 8; ++i)
	{
		uint16_t reading;
		if (!ReadAdc(reading))
		{
			error = LpcProtocol::ThermalError::adcTimeout;
			return false;
		}
		sum += reading;
	}
	lastRawAdc = static_cast<uint16_t>((sum + 4u) / 8u);

	if (lastRawAdc == 0)
	{
		error = LpcProtocol::ThermalError::shortCircuit;
		return false;
	}

	// The hotend NTC is connected from AD1 to ground, with the MCU-side ADC node pulled up.
	// Therefore Rntc = Rpullup * ADC / (fullScale - ADC).
	const float resistance = pullupR * static_cast<float>(lastRawAdc) / static_cast<float>(AdcRange - lastRawAdc);
	const float logResistance = logf(resistance);
	const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;
	if (!(recipT > 0.0f))
	{
		error = LpcProtocol::ThermalError::openCircuit;
		return false;
	}

	temperature = (1.0f / recipT) + AbsoluteZero;
	if (!isfinite(temperature) || (temperature < MinimumConnectedTemperature && resistance > pullupR * 100.0f))
	{
		error = LpcProtocol::ThermalError::openCircuit;
		return false;
	}

	error = LpcProtocol::ThermalError::none;
	return true;
}

static float CoolingRate(float temperatureRise, float pwmFan) noexcept
{
	const float scaledRise = temperatureRise * 0.01f;
	const float base = (scaledRise < 0.0f)
		? -powf(-scaledRise, model.coolingRateExponent)
		: powf(scaledRise, model.coolingRateExponent);
	return model.basicCoolingRate * base + scaledRise * model.fanCoolingRate * pwmFan;
}

static Pid PidForTarget(bool loadMode, float target) noexcept
{
	if (model.pidOverridden)
	{
		return { model.overrideKp, model.overrideRecipTi, model.overrideTd };
	}

	const float rise = (target - NormalAmbientTemperature > 1.0f) ? target - NormalAmbientTemperature : 1.0f;
	const float coolingPerDegree = CoolingRate(rise, 0.2f) / rise;
	const float kP = 0.7f / (model.heatingRate * model.deadTime);
	const float recipTi = loadMode
		? powf(coolingPerDegree, 0.25f) / (1.14f * powf(model.deadTime, 0.75f))
		: sqrtf(coolingPerDegree / model.deadTime);
	return { kP, recipTi, model.deadTime * 0.7f };
}

static void Control() noexcept
{
	const bool faultWasLatched = state == LpcProtocol::HeaterState::fault;
	const LpcProtocol::ThermalError latchedError = error;
	const bool goodTemperature = SampleTemperature();
	if (faultWasLatched)
	{
		error = latchedError;
	}
	goodTemperatureMask = static_cast<uint8_t>(goodTemperatureMask << 1);

	float derivative = 0.0f;
	bool gotDerivative = false;
	if (goodTemperature)
	{
		badReadings = 0;
		if ((goodTemperatureMask & 0x08u) != 0)
		{
			derivative = (temperature - previousTemperatures[previousIndex]);
			gotDerivative = fabsf(derivative) <= 10.0f;
		}
		previousTemperatures[previousIndex] = temperature;
		previousIndex = static_cast<uint8_t>((previousIndex + 1u) & 3u);
		goodTemperatureMask |= 1u;
	}
	else if (badReadings < 0xFFu)
	{
		++badReadings;
	}

	if (!Active())
	{
		ApplyHeater(0.0f);
		averagePwm *= 0.95f;
		statusDirty = true;
		return;
	}

	if (!LinkAlive())
	{
		SetFault(LpcProtocol::ThermalError::linkTimeout);
		return;
	}
	if (!goodTemperature && badReadings > maxBadReadings)
	{
		SetFault(error);
		return;
	}
	if (!goodTemperature)
	{
		statusDirty = true;
		return;
	}
	if (temperature > upperLimit)
	{
		SetFault(LpcProtocol::ThermalError::overTemperature);
		return;
	}
	if (lowerLimit > AbsoluteZero + 1.0f && temperature < lowerLimit)
	{
		SetFault(LpcProtocol::ThermalError::underTemperature);
		return;
	}

	const float boostedTarget = targetTemperature + extrusionTemperatureBoost;
	const float adjustedTarget = (boostedTarget < upperLimit) ? boostedTarget : upperLimit;
	if (InPidMode() && extrusionTemperatureBoost != lastExtrusionTemperatureBoost)
	{
		UpdateHeaterState(adjustedTarget);
		lastExtrusionTemperatureBoost = extrusionTemperatureBoost;
	}
	const float tempError = adjustedTarget - temperature;
	const uint32_t now = Millis();
	switch (state)
	{
	case LpcProtocol::HeaterState::heating:
		if (tempError <= TemperatureCloseEnough)
		{
			state = LpcProtocol::HeaterState::stable;
			heatingFaultMillis = 0;
		}
		else if (static_cast<float>(now - timeSetHeating) < model.deadTime * 2000.0f)
		{
			heatingReferenceTemperature = temperature;
			heatingReferenceMillis = now;
			heatingFaultMillis = 0;
		}
		else if (gotDerivative)
		{
			const float temperatureRise = (temperature > 15.0f) ? temperature - 15.0f : 0.0f;
			const float heatingPwm = (averagePwm < lastPwm) ? averagePwm : lastPwm;
			const float expectedRate = model.heatingRate * heatingPwm - CoolingRate(temperatureRise, 1.0f);
			const uint32_t actualInterval = now - heatingReferenceMillis;
			if (expectedRate <= 0.0f || static_cast<float>(actualInterval) * expectedRate >= 3000.0f)
			{
				const float expectedRise = expectedRate * static_cast<float>(actualInterval) * 0.001f;
				const float actualRise = temperature - heatingReferenceTemperature;
				if (expectedRate > 0.0f && actualRise < expectedRise * 0.6f)
				{
					heatingFaultMillis += SampleIntervalMillis;
					if (heatingFaultMillis > static_cast<uint32_t>(maxFaultTime * 1000.0f))
					{
						SetFault(LpcProtocol::ThermalError::heatingTooSlow);
						return;
					}
				}
				else
				{
					heatingReferenceTemperature = temperature;
					heatingReferenceMillis = now;
					if (heatingFaultMillis >= SampleIntervalMillis)
					{
						heatingFaultMillis -= SampleIntervalMillis;
					}
				}
			}
		}
		break;

	case LpcProtocol::HeaterState::stable:
		if (fabsf(tempError) > maxTempExcursion && temperature > MaxAmbientTemperature)
		{
			excursionFaultMillis += SampleIntervalMillis;
			if (excursionFaultMillis > static_cast<uint32_t>(maxFaultTime * 1000.0f))
			{
				SetFault(LpcProtocol::ThermalError::temperatureExcursion);
				return;
			}
		}
		else if (excursionFaultMillis >= SampleIntervalMillis)
		{
			excursionFaultMillis -= SampleIntervalMillis;
		}
		break;

	case LpcProtocol::HeaterState::cooling:
		if (-tempError <= TemperatureCloseEnough && adjustedTarget > MaxAmbientTemperature)
		{
			state = LpcProtocol::HeaterState::stable;
			heatingFaultMillis = 0;
			excursionFaultMillis = 0;
		}
		break;

	default:
		break;
	}

	float pwm;
	if (model.usePid)
	{
		const bool loadMode = state == LpcProtocol::HeaterState::stable || fabsf(tempError) < 3.0f;
		const Pid pid = PidForTarget(loadMode, adjustedTarget);
		const float errorMinusD = tempError - (gotDerivative ? pid.tD * derivative : 0.0f);
		const float pPlusD = pid.kP * errorMinusD;
		const float expected = CoolingRate(temperature - NormalAmbientTemperature, fanPwm) / model.heatingRate;
		if (pPlusD + expected > model.maxPwm)
		{
			pwm = model.maxPwm;
			if (state == LpcProtocol::HeaterState::heating && tempError > 0.0f && derivative > 0.0f)
			{
				integral = expected;
			}
		}
		else if (pPlusD + expected < 0.0f)
		{
			pwm = 0.0f;
		}
		else
		{
			integral = Clamp(integral + tempError * pid.kP * pid.recipTi * 0.25f, 0.0f, model.maxPwm);
			pwm = Clamp(pPlusD + integral, 0.0f, model.maxPwm);
		}
	}
	else
	{
		pwm = (tempError > 0.0f) ? model.maxPwm : 0.0f;
	}

	if (model.inverted)
	{
		pwm = model.maxPwm - pwm;
	}
	if (linkTimeoutLatched)
	{
		SetFault(LpcProtocol::ThermalError::linkTimeout);
		return;
	}
	ApplyHeater(pwm);
	lastPwm = pwm;
	averagePwm = averagePwm * 0.95f + pwm * 0.05f;
	statusDirty = true;
}

void ResetConfiguration() noexcept
{
	heaterFrequency = 250;
	maxBadReadings = 3;
	thermistorConfigured = false;
	heaterConfigured = false;
	modelConfigured = false;
	pendingModelParts = 0;
	badReadings = 0;
	lastRawAdc = 0;
	temperature = AbsoluteZero;
	targetTemperature = 0.0f;
	upperLimit = 2000.0f;
	lowerLimit = AbsoluteZero;
	maxTempExcursion = 15.0f;
	maxFaultTime = 5.0f;
	fanPwm = 0.0f;
	extrusionPwmBoost = 0.0f;
	extrusionTemperatureBoost = 0.0f;
	lastExtrusionTemperatureBoost = 0.0f;
	integral = 0.0f;
	averagePwm = 0.0f;
	lastPwm = 0.0f;
	heatingReferenceTemperature = 0.0f;
	heatingReferenceMillis = 0;
	timeSetHeating = 0;
	excursionFaultMillis = 0;
	heatingFaultMillis = 0;
	previousIndex = 0;
	goodTemperatureMask = 0;
	state = LpcProtocol::HeaterState::off;
	error = LpcProtocol::ThermalError::notConfigured;
	linkTimeoutLatched = false;
	ApplyHeater(0.0f);
	statusDirty = true;
}

void Init() noexcept
{

	LPC_SYSCON_SYSAHBCLKCTRL |= (1u << 13) | (1u << 16);
	LPC_SYSCON_PDRUNCFG &= ~(1u << 4);
	LPC_IOCON_PIO1_0 = (LPC_IOCON_PIO1_0 & ~0x9Fu) | 0x02u; // AD1, analog mode, no pulls
	LPC_ADC_CR = (1u << 1) | (2u << 8);
	ResetConfiguration();

	SYST_RVR = CoreClock / 1000u - 1u;
	SYST_CVR = 0;
	SYST_CSR = 0x07u;
}

static void Tick() noexcept
{
	++millisTicks;
	if (Active() && millisTicks - lastHostHeartbeat > LinkTimeoutMillis)
	{
		linkTimeoutLatched = true;
		SetFault(LpcProtocol::ThermalError::linkTimeout);
	}
}

static uint32_t Millis() noexcept
{
	return millisTicks;
}

void HostHeartbeat() noexcept
{
	lastHostHeartbeat = Millis();
}

static void ConfigureThermistor(const uint8_t* payload, size_t length) noexcept
{
	if (length != 16)
	{
		return;
	}
	r25 = ReadFloat(payload);
	beta = ReadFloat(payload + 4);
	shC = ReadFloat(payload + 8);
	pullupR = ReadFloat(payload + 12);
	if (!isfinite(r25) || !isfinite(beta) || !isfinite(shC) || !isfinite(pullupR)
		|| !(r25 > 0.0f) || !(beta > 0.0f) || !(pullupR > 0.0f))
	{
		thermistorConfigured = false;
		if (Active())
		{
			SetFault(LpcProtocol::ThermalError::notConfigured);
		}
		else
		{
			error = LpcProtocol::ThermalError::notConfigured;
			statusDirty = true;
		}
		return;
	}
	shB = 1.0f / beta;
	const float lnR25 = logf(r25);
	shA = 1.0f / (25.0f - AbsoluteZero) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;
	thermistorConfigured = true;
	badReadings = 0;
	statusDirty = true;
}

static void RejectModelUpdate() noexcept
{
	pendingModelParts = 0;
	if (Active())
	{
		SetFault(LpcProtocol::ThermalError::controlFault);
	}
}

static void ConfigureModelA(const uint8_t* payload, size_t length) noexcept
{
	if (length != 16)
	{
		RejectModelUpdate();
		return;
	}
	pendingModel = model;
	pendingModel.heatingRate = ReadFloat(payload);
	pendingModel.basicCoolingRate = ReadFloat(payload + 4);
	pendingModel.fanCoolingRate = ReadFloat(payload + 8);
	pendingModel.coolingRateExponent = ReadFloat(payload + 12);
	pendingModelParts = 0x01u;
}

static void ConfigureModelB(const uint8_t* payload, size_t length) noexcept
{
	if (length != 16)
	{
		RejectModelUpdate();
		return;
	}
	if (pendingModelParts != 0x01u)
	{
		RejectModelUpdate();
		return;
	}
	pendingModel.deadTime = ReadFloat(payload);
	pendingModel.maxPwm = ReadFloat(payload + 4);
	pendingModel.overrideKp = ReadFloat(payload + 8);
	pendingModel.overrideRecipTi = ReadFloat(payload + 12);
	pendingModelParts = 0x03u;
}

static void ConfigureModelC(const uint8_t* payload, size_t length) noexcept
{
	if (length != 5)
	{
		RejectModelUpdate();
		return;
	}
	if (pendingModelParts != 0x03u)
	{
		RejectModelUpdate();
		return;
	}
	pendingModel.overrideTd = ReadFloat(payload);
	const uint8_t flags = payload[4];
	if ((flags & ~0x07u) != 0)
	{
		RejectModelUpdate();
		return;
	}
	pendingModel.usePid = (flags & 0x01u) != 0;
	pendingModel.inverted = (flags & 0x02u) != 0;
	pendingModel.pidOverridden = (flags & 0x04u) != 0;
	if (ModelReady(pendingModel))
	{
		model = pendingModel;
		modelConfigured = true;
	}
	else
	{
		RejectModelUpdate();
		return;
	}
	pendingModelParts = 0;
}

static void ConfigureHeater(const uint8_t* payload, size_t length) noexcept
{
	if (length != 11)
	{
		return;
	}
	const uint16_t newFrequency = ReadU16(payload);
	const float newUpperLimit = static_cast<float>(ReadI16(payload + 2)) * 0.1f;
	const float newLowerLimit = static_cast<float>(ReadI16(payload + 4)) * 0.1f;
	if (newFrequency == 0 || newUpperLimit <= newLowerLimit)
	{
		heaterConfigured = false;
		if (Active())
		{
			SetFault(LpcProtocol::ThermalError::controlFault);
		}
		else
		{
			error = LpcProtocol::ThermalError::notConfigured;
			statusDirty = true;
		}
		return;
	}
	heaterFrequency = newFrequency;
	upperLimit = newUpperLimit;
	lowerLimit = newLowerLimit;
	maxTempExcursion = static_cast<float>(ReadU16(payload + 6)) * 0.01f;
	maxFaultTime = static_cast<float>(ReadU16(payload + 8)) * 0.1f;
	maxBadReadings = payload[10];
	heaterConfigured = true;
	statusDirty = true;
}

static void Command(const uint8_t* payload, size_t length) noexcept
{
	if (length != 3)
	{
		return;
	}
	const auto command = static_cast<LpcProtocol::HeaterCommand>(payload[0]);
	const float requestedTarget = static_cast<float>(ReadI16(payload + 1)) * 0.01f;
	switch (command)
	{
	case LpcProtocol::HeaterCommand::off:
		ApplyHeater(0.0f);
		if (state != LpcProtocol::HeaterState::fault)
		{
			state = LpcProtocol::HeaterState::off;
		}
		integral = 0.0f;
		extrusionPwmBoost = 0.0f;
		extrusionTemperatureBoost = 0.0f;
		lastExtrusionTemperatureBoost = 0.0f;
		excursionFaultMillis = 0;
		heatingFaultMillis = 0;
		lastPwm = 0.0f;
		break;

	case LpcProtocol::HeaterCommand::on:
		if (!LinkAlive() || !thermistorConfigured || !heaterConfigured || !modelConfigured || state == LpcProtocol::HeaterState::fault || !SampleTemperature())
		{
			if (state != LpcProtocol::HeaterState::fault)
			{
				SetFault(!LinkAlive() ? LpcProtocol::ThermalError::linkTimeout
					: !thermistorConfigured || !heaterConfigured || !modelConfigured ? LpcProtocol::ThermalError::notConfigured
					: error);
			}
			return;
		}
		if (upperLimit >= 1000.0f || requestedTarget > upperLimit
			|| (lowerLimit > AbsoluteZero + 1.0f && requestedTarget < lowerLimit))
		{
			SetFault(LpcProtocol::ThermalError::controlFault);
			return;
		}
		targetTemperature = requestedTarget;
		UpdateHeaterState((targetTemperature + extrusionTemperatureBoost < upperLimit)
			? targetTemperature + extrusionTemperatureBoost : upperLimit);
		break;

	case LpcProtocol::HeaterCommand::suspend:
		ApplyHeater(0.0f);
		if (state != LpcProtocol::HeaterState::fault)
		{
			state = LpcProtocol::HeaterState::suspended;
		}
		lastPwm = 0.0f;
		break;

	case LpcProtocol::HeaterCommand::resetFault:
		if (LinkAlive() && SampleTemperature())
		{
			linkTimeoutLatched = false;
			ApplyHeater(0.0f);
			state = LpcProtocol::HeaterState::off;
			error = LpcProtocol::ThermalError::none;
			badReadings = 0;
			integral = 0.0f;
			extrusionPwmBoost = 0.0f;
			extrusionTemperatureBoost = 0.0f;
			lastExtrusionTemperatureBoost = 0.0f;
			lastPwm = 0.0f;
			heatingFaultMillis = 0;
			excursionFaultMillis = 0;
		}
		break;
	}
	statusDirty = true;
}

static void ConfigureFeedForward(const uint8_t* payload, size_t length) noexcept
{
	if (length != 12)
	{
		return;
	}
	const float newFanPwm = ReadFloat(payload);
	const float newExtrusionPwmBoost = ReadFloat(payload + 4);
	const float newExtrusionTemperatureBoost = ReadFloat(payload + 8);
	if (!isfinite(newFanPwm) || !isfinite(newExtrusionPwmBoost) || !isfinite(newExtrusionTemperatureBoost))
	{
		SetFault(LpcProtocol::ThermalError::controlFault);
		return;
	}
	const float clampedFanPwm = Clamp(newFanPwm, 0.0f, 1.0f);
	if (state == LpcProtocol::HeaterState::stable)
	{
		float pwmBoost = newExtrusionPwmBoost - extrusionPwmBoost;
		extrusionPwmBoost = newExtrusionPwmBoost;
		if (clampedFanPwm != fanPwm)
		{
			const float pwmChange = clampedFanPwm - fanPwm;
			fanPwm = clampedFanPwm;
			pwmBoost += (targetTemperature - NormalAmbientTemperature) * 0.01f
				* model.fanCoolingRate * pwmChange / model.heatingRate * FanFeedForwardMultiplier;
		}
		integral += pwmBoost;
	}
	extrusionTemperatureBoost = newExtrusionTemperatureBoost;
}

void HandleFrame(const LpcProtocol::Frame& frame) noexcept
{
	switch (frame.type)
	{
	case LpcProtocol::MessageType::thermistorConfig:
		ConfigureThermistor(frame.payload, frame.length);
		break;
	case LpcProtocol::MessageType::heaterModelA:
		ConfigureModelA(frame.payload, frame.length);
		break;
	case LpcProtocol::MessageType::heaterModelB:
		ConfigureModelB(frame.payload, frame.length);
		break;
	case LpcProtocol::MessageType::heaterModelC:
		ConfigureModelC(frame.payload, frame.length);
		break;
	case LpcProtocol::MessageType::heaterConfig:
		ConfigureHeater(frame.payload, frame.length);
		break;
	case LpcProtocol::MessageType::heaterCommand:
		Command(frame.payload, frame.length);
		break;
	case LpcProtocol::MessageType::heaterFeedForward:
		ConfigureFeedForward(frame.payload, frame.length);
		break;
	default:
		break;
	}
}

void Spin() noexcept
{
	const uint32_t now = Millis();
	if (now - lastSampleTime >= SampleIntervalMillis)
	{
		lastSampleTime = now;
		Control();
	}
}

bool TakeStatus(uint8_t* payload, size_t& length) noexcept
{
	if (!statusDirty)
	{
		return false;
	}
	statusDirty = false;
	const float boundedTemperature = Clamp(temperature, -327.68f, 327.67f);
	const int16_t temperatureCenti = static_cast<int16_t>(boundedTemperature * 100.0f);
	const uint16_t pwm = static_cast<uint16_t>(Clamp(averagePwm, 0.0f, 1.0f) * 65535.0f + 0.5f);
	payload[0] = static_cast<uint8_t>(temperatureCenti);
	payload[1] = static_cast<uint8_t>(static_cast<uint16_t>(temperatureCenti) >> 8);
	payload[2] = static_cast<uint8_t>(lastRawAdc);
	payload[3] = static_cast<uint8_t>(lastRawAdc >> 8);
	payload[4] = static_cast<uint8_t>(pwm);
	payload[5] = static_cast<uint8_t>(pwm >> 8);
	payload[6] = static_cast<uint8_t>(state);
	payload[7] = static_cast<uint8_t>(error);
	length = 8;
	return true;
}

}

extern "C" void SysTick_Handler() noexcept
{
	Thermal::Tick();
}
