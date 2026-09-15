#ifndef SRC_HEATING_LPCHEATER_H_
#define SRC_HEATING_LPCHEATER_H_

#include "Heater.h"

class LpcHeater final : public Heater
{
public:
	explicit LpcHeater(unsigned int heaterNum) noexcept;
	~LpcHeater() noexcept override;

	GCodeResult ConfigurePortAndSensor(const char *_ecv_array portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) override;
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept override;
	GCodeResult ReportDetails(const StringRef& reply) const noexcept override;
	void Spin() noexcept override;
	void SwitchOff() noexcept override;
	GCodeResult ResetFault(const StringRef& reply) noexcept override;
	float GetTemperature() const noexcept override;
	float GetAveragePWM() const noexcept override;
	float GetAccumulator() const noexcept override { return 0.0f; }
	void Suspend(bool sus) noexcept override;
	void SetFanFeedForwardPwm(float pwm) noexcept override;

protected:
	HeaterMode GetMode() const noexcept override { return (tuning) ? HeaterMode::tuning0 : mode; }
	GCodeResult SwitchOn(const StringRef& reply) noexcept override;
	GCodeResult UpdateModel(const StringRef& reply) noexcept override;
	GCodeResult UpdateFaultDetectionParameters(const StringRef& reply) noexcept override;
	GCodeResult UpdateHeaterMonitors(const StringRef& reply) noexcept override;
	GCodeResult StartAutoTune(const StringRef& reply, bool seenA, float ambientTemp) noexcept override;
	void ApplyExtrusionFeedForward() noexcept override;

private:
	void SendConfiguration() noexcept;
	void SendFeedForward() noexcept;
	void RaiseFault(LpcProtocol::ThermalError error) noexcept;
	GCodeResult ValidateMonitors(const StringRef& reply) const noexcept;
	void PollTuning() noexcept;
	void StopTuning() noexcept;
	void CancelTuning(const char *reason) noexcept;

	PwmFrequency frequency;
	HeaterMode mode;
	uint32_t connectionGeneration;
	bool tuning = false;
	uint32_t tuningBeginTime;			// when the current tuning run started, for the not-increasing/overall timeout checks
	float tuningStartTemperature;		// temperature when tuning started, for the "not increasing" check
};

#endif
