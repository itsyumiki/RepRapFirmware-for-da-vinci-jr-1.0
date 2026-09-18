#ifndef SRC_HARDWARE_SAM4E_LPCISPUPDATER_H_
#define SRC_HARDWARE_SAM4E_LPCISPUPDATER_H_

#include <RepRapFirmware.h>

namespace LpcIspUpdater
{

GCodeResult CheckFirmwareFile(const StringRef& filenameRef, const StringRef& reply) noexcept;
void Update(const StringRef& filenameRef) noexcept;

}

#endif
