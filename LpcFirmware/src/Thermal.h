#ifndef LPC_THERMAL_H
#define LPC_THERMAL_H

#include <LpcProtocol.h>
#include <stddef.h>
#include <stdint.h>

namespace Thermal
{

void Init() noexcept;
void ResetConfiguration() noexcept;
void HostHeartbeat() noexcept;
void HandleFrame(const LpcProtocol::Frame& frame) noexcept;

void Spin() noexcept;
bool TakeStatus(uint8_t* payload, size_t& length) noexcept;

// Returns true once per completed tuning cycle (immediately clearing the pending state), filling in
// both payloads for the caller to send as heaterTuningReportA followed by heaterTuningReportB.
bool TakeTuningReport(uint8_t* payloadA, size_t& lengthA, uint8_t* payloadB, size_t& lengthB) noexcept;

}

#endif
