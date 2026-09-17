/*
 * Devices.h
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAM4E_DEVICES_H_
#define SRC_HARDWARE_SAM4E_DEVICES_H_

#include <AsyncSerial.h>

class Stream;

extern AsyncSerial lpcUart;

#define SUPPORT_USB		1		// needed by SerialCDC.h
#include <SerialCDC.h>

extern SerialCDC serialUSB;

#if defined(DA_VINCI_JR)
Stream& GetWiFiUploadSerial() noexcept;
void BeginWiFiUploadSerial(uint32_t baud) noexcept;
void EndWiFiUploadSerial() noexcept;
void PrepareWiFiUploadSerial(bool external) noexcept;
#endif

void DeviceInit() noexcept;
void StopAnalogTask() noexcept;
void StopUsbTask() noexcept;

#endif /* SRC_HARDWARE_SAM4E_DEVICES_H_ */
