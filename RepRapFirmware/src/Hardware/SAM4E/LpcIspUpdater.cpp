#include "LpcIspUpdater.h"

#if defined(DA_VINCI_JR)

#include "Devices.h"
#include "LpcInterface.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Storage/FileStore.h>

#include <Core.h>
#include <General/SafeVsnprintf.h>
#include <cstring>

namespace LpcIspUpdater
{

namespace
{
constexpr const char *DefaultFirmwareFile = "Lpc1115Firmware.bin";

constexpr uint32_t FlashSize = 64u * 1024u;
constexpr uint32_t RamTop = 0x10002000u;
constexpr uint32_t RamBuffer = 0x10000300u;
constexpr uint32_t PartId = 0x00050080u;
constexpr uint32_t SectorSize = 4096u;
constexpr uint32_t NumSectors = FlashSize / SectorSize;
constexpr size_t TransferSize = 512u;
constexpr size_t UuencodeLineBytes = 45u;
constexpr uint32_t CommandTimeout = 5000u;
constexpr uint32_t SyncTimeout = 1000u;
constexpr unsigned int MaxUuencodeAttempts = 3;

constexpr uint32_t CrpNone = 0xFFFFFFFFu;
constexpr uint32_t Crp1 = 0x12345678u;
constexpr uint32_t Crp2 = 0x87654321u;
constexpr uint32_t Crp3 = 0x43218765u;
constexpr uint32_t CrpNoIsp = 0x4E697370u;
constexpr FilePosition CrpOffset = 0x2FCu;

bool IsCrpProtected(uint32_t value) noexcept
{
	return value == Crp1 || value == Crp2 || value == Crp3 || value == CrpNoIsp;
}

const char *FirmwareFilename(const StringRef& filenameRef) noexcept
{
	return filenameRef.IsEmpty() ? DefaultFirmwareFile : filenameRef.c_str();
}

bool ReadLine(char *buffer, size_t bufferSize, uint32_t timeout) noexcept
{
	size_t length = 0;
	const uint32_t started = millis();
	while (millis() - started < timeout)
	{
		while (lpcUart.available() != 0)
		{
			const int value = lpcUart.read();
			if (value < 0)
			{
				continue;
			}
			const char c = static_cast<char>(value);
			if (c == '\r')
			{
				continue;
			}
			if (c == '\n')
			{
				if (length == 0)
				{
					continue;
				}
				buffer[length] = 0;
				return true;
			}
			if (length + 1 >= bufferSize)
			{
				return false;
			}
			buffer[length++] = c;
		}
		delay(1);
	}
	return false;
}

bool WaitForLine(const char *expected, uint32_t timeout) noexcept
{
	char line[96];
	const uint32_t started = millis();
	while (millis() - started < timeout)
	{
		const uint32_t remaining = timeout - (millis() - started);
		if (!ReadLine(line, sizeof(line), remaining))
		{
			return false;
		}
		const char *text = line;
		while (*text == '?')
		{
			++text;
		}
		if (strcmp(text, expected) == 0)
		{
			return true;
		}
	}
	return false;
}

void WriteText(const char *text) noexcept
{
	lpcUart.write(reinterpret_cast<const uint8_t *>(text), strlen(text));
	lpcUart.flush();
}

bool SendCommand(const char *command) noexcept
{
	WriteText(command);
	return WaitForLine("0", CommandTimeout);
}

bool SendFormattedCommand(const char *format, uint32_t a, uint32_t b = 0, uint32_t c = 0) noexcept
{
	char command[64];
	SafeSnprintf(command, sizeof(command), format, static_cast<unsigned long>(a), static_cast<unsigned long>(b), static_cast<unsigned long>(c));
	return SendCommand(command);
}

void SetBootPins(bool ispLow) noexcept
{
	SetPinMode(LpcIspPin, ispLow ? OUTPUT_LOW : OUTPUT_HIGH);
	SetPinMode(LpcResetPin, OUTPUT_LOW);
	delay(10);
	digitalWrite(LpcResetPin, true);
	delay(50);
	if (ispLow)
	{
		digitalWrite(LpcIspPin, true);
	}
}

bool Synchronize() noexcept
{
	lpcUart.ClearReceiveBuffer();
	WriteText("?");
	if (!WaitForLine("Synchronized", SyncTimeout))
	{
		return false;
	}

	WriteText("Synchronized\r\n");
	if (!WaitForLine("OK", SyncTimeout))
	{
		return false;
	}

	WriteText("12000\r\n");
	if (!WaitForLine("OK", SyncTimeout))
	{
		return false;
	}

	// Disable boot-ROM command echo so subsequent replies are unambiguous.
	WriteText("A 0\r\n");
	return WaitForLine("0", SyncTimeout);
}

bool ParseU32(const char *text, uint32_t& value) noexcept
{
	if (*text == 0)
	{
		return false;
	}
	uint32_t parsed = 0;
	for (; *text != 0; ++text)
	{
		if (*text < '0' || *text > '9')
		{
			return false;
		}
		const uint32_t digit = static_cast<uint32_t>(*text - '0');
		if (parsed > (UINT32_MAX - digit) / 10u)
		{
			return false;
		}
		parsed = parsed * 10u + digit;
	}
	value = parsed;
	return true;
}

bool CheckPartId() noexcept
{
	WriteText("J\r\n");
	if (!WaitForLine("0", CommandTimeout))
	{
		return false;
	}
	char line[32];
	if (!ReadLine(line, sizeof(line), CommandTimeout))
	{
		return false;
	}
	uint32_t id;
	return ParseU32(line, id) && id == PartId;
}

char UuencodeChar(uint8_t value) noexcept
{
	value &= 0x3Fu;
	return (value == 0) ? '`' : static_cast<char>(value + 0x20u);
}

bool SendUuencodedBlock(const uint8_t *data, size_t length) noexcept
{
	for (unsigned int attempt = 0; attempt < MaxUuencodeAttempts; ++attempt)
	{
		uint32_t checksum = 0;
		for (size_t offset = 0; offset < length; offset += UuencodeLineBytes)
		{
			const size_t lineLength = ((length - offset) < UuencodeLineBytes) ? (length - offset) : UuencodeLineBytes;
			char line[64];
			size_t out = 0;
			line[out++] = UuencodeChar(static_cast<uint8_t>(lineLength));
			for (size_t i = 0; i < lineLength; i += 3)
			{
				const uint8_t a = data[offset + i];
				const uint8_t b = (i + 1 < lineLength) ? data[offset + i + 1] : 0;
				const uint8_t c = (i + 2 < lineLength) ? data[offset + i + 2] : 0;
				checksum += a;
				if (i + 1 < lineLength) { checksum += b; }
				if (i + 2 < lineLength) { checksum += c; }
				line[out++] = UuencodeChar(a >> 2);
				line[out++] = UuencodeChar(static_cast<uint8_t>((a << 4) | (b >> 4)));
				line[out++] = UuencodeChar(static_cast<uint8_t>((b << 2) | (c >> 6)));
				line[out++] = UuencodeChar(c);
			}
			line[out++] = '\r';
			line[out++] = '\n';
			lpcUart.write(reinterpret_cast<const uint8_t *>(line), out);
		}
		lpcUart.flush();

		char checksumLine[24];
		SafeSnprintf(checksumLine, sizeof(checksumLine), "%lu\r\n", static_cast<unsigned long>(checksum));
		WriteText(checksumLine);

		char response[16];
		if (!ReadLine(response, sizeof(response), CommandTimeout))
		{
			return false;
		}
		if (strcmp(response, "OK") == 0)
		{
			return true;
		}
		if (strcmp(response, "RESEND") != 0)
		{
			return false;
		}
	}
	return false;
}

bool ProgramBlock(FileStore& firmware, FilePosition imageLength, uint32_t flashOffset) noexcept
{
	uint8_t data[TransferSize];
	memset(data, 0xFF, sizeof(data));
	const size_t bytesToRead = static_cast<size_t>(((imageLength - flashOffset) < TransferSize) ? (imageLength - flashOffset) : TransferSize);
	if (!firmware.Seek(flashOffset) || firmware.Read(data, bytesToRead) != static_cast<int>(bytesToRead))
	{
		return false;
	}

	if (!SendFormattedCommand("W %lu %lu\r\n", RamBuffer, TransferSize) || !SendUuencodedBlock(data, sizeof(data)))
	{
		return false;
	}

	const uint32_t sector = flashOffset / SectorSize;
	if (!SendFormattedCommand("P %lu %lu\r\n", sector, sector)
		|| !SendFormattedCommand("C %lu %lu %lu\r\n", flashOffset, RamBuffer, TransferSize))
	{
		return false;
	}

	// The boot ROM remaps the first 512 bytes while ISP is active, so its M
	// command cannot reliably verify block zero. The UUencode checksum still
	// protects the RAM transfer and the ROM reports copy-to-flash failures.
	if (flashOffset == 0)
	{
		return true;
	}
	return SendFormattedCommand("M %lu %lu %lu\r\n", flashOffset, RamBuffer, TransferSize);
}

bool ProgramRange(FileStore& firmware, FilePosition imageLength, uint32_t begin, uint32_t end) noexcept
{
	for (uint32_t offset = begin; offset < end; offset += TransferSize)
	{
		if (!ProgramBlock(firmware, imageLength, offset))
		{
			return false;
		}
	}
	return true;
}

bool ValidateFirmware(FileStore& firmware, const char *filename, const StringRef& reply) noexcept
{
	const FilePosition length = firmware.Length();
	if (length < 32u || length > FlashSize)
	{
		reply.printf("LPC firmware \"%s\" has invalid size", filename);
		return false;
	}

	uint32_t vectors[8];
	if (!firmware.Seek(0) || firmware.Read(reinterpret_cast<uint8_t *>(vectors), sizeof(vectors)) != static_cast<int>(sizeof(vectors)))
	{
		reply.printf("Unable to read LPC firmware \"%s\"", filename);
		return false;
	}
	uint32_t vectorSum = 0;
	for (uint32_t value : vectors)
	{
		vectorSum += value;
	}
	const uint32_t resetVector = vectors[1];
	if (vectors[0] != RamTop || (resetVector & 1u) == 0 || static_cast<FilePosition>(resetVector & ~1u) >= length || vectorSum != 0)
	{
		reply.printf("LPC firmware \"%s\" has an invalid vector table", filename);
		return false;
	}

	uint32_t crp = CrpNone;
	if (length > CrpOffset)
	{
		if (!firmware.Seek(CrpOffset) || firmware.Read(reinterpret_cast<uint8_t *>(&crp), sizeof(crp)) != static_cast<int>(sizeof(crp)))
		{
			reply.printf("Unable to read LPC firmware CRP word from \"%s\"", filename);
			return false;
		}
	}
	if (IsCrpProtected(crp))
	{
		reply.printf("LPC firmware \"%s\" enables Code Read Protection and cannot be installed", filename);
		return false;
	}
	return true;
}

}

GCodeResult CheckFirmwareFile(const StringRef& filenameRef, const StringRef& reply) noexcept
{
	const char * const filename = FirmwareFilename(filenameRef);
	FileStore * const firmware = reprap.GetPlatform().OpenFile(FIRMWARE_DIRECTORY, filename, OpenMode::read);
	if (firmware == nullptr)
	{
		reply.printf("LPC firmware \"%s%s\" not found", FIRMWARE_DIRECTORY, filename);
		return GCodeResult::error;
	}
	const bool valid = ValidateFirmware(*firmware, filename, reply);
	firmware->Close();
	return valid ? GCodeResult::ok : GCodeResult::error;
}

void Update(const StringRef& filenameRef) noexcept
{
	const char * const filename = FirmwareFilename(filenameRef);
	FileStore * const firmware = reprap.GetPlatform().OpenFile(FIRMWARE_DIRECTORY, filename, OpenMode::read);
	if (firmware == nullptr)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "LPC update: can't open %s\n", filename);
		return;
	}
	String<StringLength100> validationReply;
	if (!ValidateFirmware(*firmware, filename, validationReply.GetRef()))
	{
		firmware->Close();
		reprap.GetPlatform().MessageF(ErrorMessage, "LPC update: %s\n", validationReply.c_str());
		return;
	}
	const FilePosition imageLength = firmware->Length();
	const uint32_t programLength = static_cast<uint32_t>((imageLength + TransferSize - 1u) & ~(TransferSize - 1u));

	reprap.GetPlatform().MessageF(FirmwareUpdateMessage, "Updating LPC1115 from %s\n", filename);
	LpcInterface::PrepareForFirmwareUpdate();
	SetBootPins(true);

	bool ok = Synchronize();
	if (ok) { ok = SendCommand("U 23130\r\n"); }
	if (ok) { ok = CheckPartId(); }
	if (ok) { ok = SendFormattedCommand("P %lu %lu\r\n", 0u, NumSectors - 1u); }
	if (ok) { ok = SendFormattedCommand("E %lu %lu\r\n", 0u, NumSectors - 1u); }

	// Program every block above sector zero first, then sector-zero blocks 1..7,
	// and the vector/checksum block at address zero last. A power loss therefore
	// leaves the LPC in ROM ISP until the final block has been written.
	if (ok && programLength > SectorSize)
	{
		ok = ProgramRange(*firmware, imageLength, SectorSize, programLength);
	}
	if (ok && programLength > TransferSize)
	{
		const uint32_t sectorZeroEnd = (programLength < SectorSize) ? programLength : SectorSize;
		ok = ProgramRange(*firmware, imageLength, TransferSize, sectorZeroEnd);
	}
	if (ok)
	{
		ok = ProgramBlock(*firmware, imageLength, 0);
	}

	firmware->Close();
	SetBootPins(false);
	LpcInterface::FirmwareUpdateFinished();

	if (ok)
	{
		reprap.GetPlatform().Message(FirmwareUpdateMessage, "LPC1115 firmware update complete\n");
	}
	else
	{
		reprap.GetPlatform().Message(ErrorMessage, "LPC1115 firmware update failed; retry M997 S3 to re-enter ROM ISP\n");
	}
}

}

#endif
