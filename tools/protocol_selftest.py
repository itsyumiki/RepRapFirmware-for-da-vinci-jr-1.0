#!/usr/bin/env python3
from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = ROOT / "build" / "protocol-selftest"
SOURCE = BUILD_DIR / "main.cpp"
BINARY = BUILD_DIR / "protocol-selftest"

TEST_PROGRAM = r'''
#include <LpcProtocol.h>

#include <cassert>
#include <cstddef>
#include <cstdint>

using namespace LpcProtocol;

static void RoundTrip(MessageType type, const uint8_t* payload, uint8_t length)
{
    uint8_t encoded[MaxEncodedFrame] = {};
    const size_t encodedLength = Encode(type, payload, length, encoded);
    assert(encodedLength == static_cast<size_t>(length) + 4u);
    assert(encoded[0] == SyncByte);

    Decoder decoder{};
    Reset(decoder);
    Frame decoded{};
    for (size_t i = 0; i + 1 < encodedLength; ++i)
    {
        assert(!Feed(decoder, encoded[i], decoded));
    }
    assert(Feed(decoder, encoded[encodedLength - 1], decoded));
    assert(decoded.type == type);
    assert(decoded.length == length);
    for (uint8_t i = 0; i < length; ++i)
    {
        assert(decoded.payload[i] == payload[i]);
    }
}

int main()
{
    uint8_t payload[MaxPayload] = {};
    for (uint8_t i = 0; i < MaxPayload; ++i)
    {
        payload[i] = static_cast<uint8_t>(i * 17u + 3u);
    }

    const uint8_t version[] = { Version };
    const uint8_t pong[] = { Version, 1 };
    RoundTrip(MessageType::ping, version, sizeof(version));
    RoundTrip(MessageType::pong, pong, sizeof(pong));
    RoundTrip(MessageType::configurationReset, nullptr, 0);
    RoundTrip(MessageType::configurationComplete, nullptr, 0);
    RoundTrip(MessageType::gpioState, payload, 2);
    RoundTrip(MessageType::pwmWrite, payload, 5);
    RoundTrip(MessageType::thermalStatus, payload, 8);
    RoundTrip(MessageType::thermistorConfig, payload, MaxPayload);

     RoundTrip(MessageType::heaterTuningCommand, payload, 8);
     RoundTrip(MessageType::heaterTuningReportA, payload, 14);
     RoundTrip(MessageType::heaterTuningReportB, payload, 16);

    uint8_t encoded[MaxEncodedFrame] = {};
    size_t length = Encode(MessageType::gpioWrite, payload, 2, encoded);
    assert(length != 0);
    encoded[length - 1] ^= 0x01u;
    Decoder decoder{};
    Reset(decoder);
    Frame decoded{};
    for (size_t i = 0; i < length; ++i)
    {
        assert(!Feed(decoder, encoded[i], decoded));
    }

    assert(Encode(MessageType::ping, payload, static_cast<uint8_t>(MaxPayload + 1u), encoded) == 0);

    Reset(decoder);
    assert(!Feed(decoder, SyncByte, decoded));
    assert(!Feed(decoder, static_cast<uint8_t>(MessageType::ping), decoded));
    assert(!Feed(decoder, static_cast<uint8_t>(MaxPayload + 1u), decoded));
    length = Encode(MessageType::pong, payload, 1, encoded);
    bool received = false;
    for (size_t i = 0; i < length; ++i)
    {
        received = Feed(decoder, encoded[i], decoded) || received;
    }
    assert(received);
    assert(decoded.type == MessageType::pong);
    assert(decoded.length == 1);
    assert(decoded.payload[0] == payload[0]);
    return 0;
}
'''


def main() -> int:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    SOURCE.write_text(TEST_PROGRAM)
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "Shared" / "src"),
            str(SOURCE),
            str(ROOT / "Shared" / "src" / "LpcProtocol.cpp"),
            "-o",
            str(BINARY),
        ],
        check=True,
    )
    subprocess.run([str(BINARY)], check=True)
    print("LPC protocol self-test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
