/*
 * FirmwareUpdater.cpp
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#include "FirmwareUpdater.h"

#if HAS_WIFI_NETWORKING || HAS_AUX_DEVICES || HAS_MASS_STORAGE || HAS_SBC_INTERFACE

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>

#if defined(DA_VINCI_JR)
# include <Hardware/SAM4E/LpcIspUpdater.h>
#endif

#if HAS_WIFI_NETWORKING
# include <Networking/Network.h>
# include <Networking/ESP8266WiFi/WifiFirmwareUploader.h>
#endif

#if HAS_AUX_DEVICES
# include <Comms/PanelDueUpdater.h>
#endif

namespace FirmwareUpdater
{
	// Check that the prerequisites are satisfied.
	// Return true if yes, else print a message and return false.
	GCodeResult CheckFirmwareUpdatePrerequisites(
			Bitmap<uint8_t> moduleMap,
			GCodeBuffer& gb,
			const StringRef& reply,
			const size_t serialChannel,
			const StringRef& filenameRef) noexcept
	{
#if defined(DA_VINCI_JR)
		if (moduleMap.IsBitSet(LpcFirmwareModule))
		{
			const GCodeResult result = LpcIspUpdater::CheckFirmwareFile(filenameRef, reply);
			if (result != GCodeResult::ok)
			{
				return result;
			}
		}
#endif
#if HAS_WIFI_NETWORKING && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		if (moduleMap.IsBitSet(WifiExternalFirmwareModule) || moduleMap.IsBitSet(WifiFirmwareModule))
		{
			GCodeResult result;
			if (!reprap.GetGCodes().CheckNetworkCommandAllowed(gb, reply, result))
			{
				return result;
			}
			if (moduleMap.IsBitSet(WifiExternalFirmwareModule) && moduleMap.IsBitSet(WifiFirmwareModule))
			{
				reply.copy("Invalid combination of firmware update modules");
				return GCodeResult::error;
			}
#if !WIFI_USES_UART
			if (moduleMap.IsBitSet(WifiFirmwareModule))
			{
				reply.copy("WiFi firmware upload is not supported on this board");
				return GCodeResult::error;
			}
#endif
			if (moduleMap.IsBitSet(WifiFirmwareModule))
			{
				String<MaxFilenameLength> location;
				if (!MassStorage::CombineName(location.GetRef(), FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? reprap.GetPlatform().GetDefaultWiFiFirmwareName() : filenameRef.c_str())
						|| !MassStorage::FileExists(location.c_str()))
				{
					reply.printf("File %s not found", location.c_str());
					return GCodeResult::error;
				}
			}
		}
#endif
#if SUPPORT_PANELDUE_FLASH && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		if (moduleMap.IsBitSet(PanelDueFirmwareModule))
		{
			if (!reprap.GetPlatform().IsChanEnabled(serialChannel) || reprap.GetPlatform().IsChanRaw(serialChannel))
			{
				reply.printf("Aux port %d is not enabled or not in PanelDue mode", serialChannel-1);
				return GCodeResult::error;
			}
			String<MaxFilenameLength> location;
			if (!MassStorage::CombineName(location.GetRef(), FIRMWARE_DIRECTORY, filenameRef.IsEmpty() ? PANEL_DUE_FIRMWARE_FILE : filenameRef.c_str())
					|| !MassStorage::FileExists(location.c_str()))
			{
				reply.printf("File %s not found", location.c_str());
				return GCodeResult::error;
			}
		}
#endif
		return GCodeResult::ok;
	}

	bool IsReady() noexcept
	{
#if HAS_WIFI_NETWORKING && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		WifiFirmwareUploader *_ecv_null const uploader = reprap.GetNetwork().GetWifiUploader();
		if (uploader != nullptr && !uploader->IsReady())
		{
			return false;
		}
#endif
#if SUPPORT_PANELDUE_FLASH
		PanelDueUpdater *_ecv_null const panelDueUpdater = reprap.GetPlatform().GetPanelDueUpdater();
		if (panelDueUpdater != nullptr && !panelDueUpdater->Idle())
		{
			return false;
		}
#endif
		return true;
	}

	void UpdateModule(unsigned int module, const size_t serialChannel, const StringRef& filenameRef) noexcept
	{
#if (HAS_WIFI_NETWORKING || SUPPORT_PANELDUE_FLASH || defined(DA_VINCI_JR)) && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)
		switch(module)
		{
# if defined(DA_VINCI_JR)
		case LpcFirmwareModule:
			LpcIspUpdater::Update(filenameRef);
			break;
# endif
# if HAS_WIFI_NETWORKING
		case WifiExternalFirmwareModule:
			{
				reprap.GetNetwork().ResetWiFiForUpload(true);
			}
			break;

		case WifiFirmwareModule:
			{
				WifiFirmwareUploader *_ecv_null const uploader = reprap.GetNetwork().GetWifiUploader();
				if (uploader != nullptr)
				{
					const char *_ecv_array binaryFilename = filenameRef.IsEmpty() ? reprap.GetPlatform().GetDefaultWiFiFirmwareName() : filenameRef.c_str();
					uploader->SendUpdateFile(binaryFilename, WifiFirmwareUploader::FirmwareAddress);
				}
			}
			break;
# endif
# if SUPPORT_PANELDUE_FLASH
		case PanelDueFirmwareModule:
			{
				Platform& platform = reprap.GetPlatform();
				if (platform.GetPanelDueUpdater() == nullptr)
				{
					platform.InitPanelDueUpdater();
				}
				platform.GetPanelDueUpdater()->Start(filenameRef, serialChannel);
			}
# endif
		}
#endif
	}
}

#endif

// End
