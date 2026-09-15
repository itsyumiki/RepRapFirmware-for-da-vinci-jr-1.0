#ifndef SRC_HARDWARE_SAM4E_LPCINTERFACE_H_
#define SRC_HARDWARE_SAM4E_LPCINTERFACE_H_

#include <CoreIO.h>
#include <LpcProtocol.h>

class FopDt;

namespace LpcInterface
{

struct ThermalStatus
{
	float temperature;
	uint16_t rawAdc;
	float averagePwm;
	LpcProtocol::HeaterState state;
	LpcProtocol::ThermalError error;
};

struct TuningReport
{
	uint16_t cyclesDone;
	uint32_t ton;
	uint32_t toff;
	uint32_t dlow;
	uint32_t dhigh;
	float heatingRate;
	float coolingRate;
	float voltage;
};

void Init() noexcept;
void Spin() noexcept;
bool IsOnline() noexcept;
uint32_t GetConnectionGeneration() noexcept;
bool IsPinAvailable(Pin pin) noexcept;
bool SetPinMode(Pin pin, PinMode mode) noexcept;
bool ReadPin(Pin pin) noexcept;
uint16_t ReadAnalog(Pin pin) noexcept;
void WritePin(Pin pin, bool high) noexcept;
void WritePwm(Pin pin, float duty, uint16_t frequency) noexcept;

void ConfigureThermistor(unsigned int sensorNumber, float r25, float beta, float coefficientC, float seriesResistance) noexcept;
void UnregisterThermistor(unsigned int sensorNumber) noexcept;
bool IsThermistorSensor(unsigned int sensorNumber) noexcept;
void ConfigureHeaterModel(const FopDt& model) noexcept;
void ConfigureHeater(uint16_t frequency, float upperLimit, float lowerLimit, float maxExcursion, float maxFaultTime, uint8_t maxBadReadings) noexcept;
void CommandHeater(LpcProtocol::HeaterCommand command, float targetTemperature) noexcept;
void ConfigureHeaterFeedForward(float fanPwm, float extrusionPwmBoost, float extrusionTemperatureBoost) noexcept;
bool GetThermalStatus(ThermalStatus& status) noexcept;

// Send a heaterTuningCommand frame. 'on' == false cancels tuning and returns the heater to the off state.
void StartHeaterTuning(bool on, float pwm, float lowTemp, float highTemp, float peakTempDrop) noexcept;

// Returns true once for each newly-completed tuning cycle (i.e. once both heaterTuningReportA and
// heaterTuningReportB for that cycle have been received), and clears the pending flag on read.
bool GetTuningReport(TuningReport& report) noexcept;

}

#endif
