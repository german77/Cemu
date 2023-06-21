// SPDX-FileCopyrightText: Copyright 2022 yuzu Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

// Based on dkms-hid-nintendo implementation, CTCaer joycon toolkit and dekuNukem reverse
// engineering https://github.com/nicman23/dkms-hid-nintendo/blob/master/src/hid-nintendo.c
// https://github.com/CTCaer/jc_toolkit
// https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering

#pragma once

#include <vector>

#include "input/api/Joycon/JoyconCommonProtocol.h"
#include "input/api/Joycon/JoyconTypes.h"

namespace InputCommon::Joycon {

class NfcProtocol final : private JoyconCommonProtocol {
public:
    explicit NfcProtocol(std::shared_ptr<JoyconHandle> handle);

    DriverResult EnableNfc();

    DriverResult DisableNfc();

    DriverResult StartNFCPollingMode();

    DriverResult StopNFCPollingMode();

    DriverResult GetTagInfo(Joycon::TagInfo& tag_info);

    DriverResult ReadAmiibo(std::vector<uint8>& data);

    DriverResult WriteAmiibo(std::span<const uint8> data);

    DriverResult ReadMifare(std::span<const MifareReadChunk> read_request,
                            std::span<MifareReadData> out_data);

    DriverResult WriteMifare(std::span<const MifareWriteChunk> write_request);

    bool HasAmiibo();

    bool IsEnabled() const;

    bool IsPolling() const;

private:
    // Number of times the function will be delayed until it outputs valid data
    static constexpr std::size_t AMIIBO_UPDATE_DELAY = 15;

    struct TagFoundData {
        uint8 type;
        uint8 uuid_size;
        TagUUID uuid;
    };

    DriverResult WaitUntilNfcIs(NFCStatus status);

    DriverResult IsTagInRange(TagFoundData& data, std::size_t timeout_limit = 1);

    DriverResult GetAmiiboData(std::vector<uint8>& data);

    DriverResult WriteAmiiboData(const TagUUID& tag_uuid, std::span<const uint8> data);

    DriverResult GetMifareData(const MifareUUID& tag_uuid,
                               std::span<const MifareReadChunk> read_request,
                               std::span<MifareReadData> out_data);

    DriverResult WriteMifareData(const MifareUUID& tag_uuid,
                                 std::span<const MifareWriteChunk> write_request);

    DriverResult SendStartPollingRequest(MCUCommandResponse& output,
                                         bool is_second_attempt = false);

    DriverResult SendStopPollingRequest(MCUCommandResponse& output);

    DriverResult SendNextPackageRequest(MCUCommandResponse& output, uint8 packet_id);

    DriverResult SendReadAmiiboRequest(MCUCommandResponse& output, NFCPages ntag_pages);

    DriverResult SendWriteAmiiboRequest(MCUCommandResponse& output, const TagUUID& tag_uuid);

    DriverResult SendWriteDataAmiiboRequest(MCUCommandResponse& output, uint8 block_id,
                                            bool is_last_packet, std::span<const uint8> data);

    DriverResult SendReadDataMifareRequest(MCUCommandResponse& output, uint8 block_id,
                                           bool is_last_packet, std::span<const uint8> data);

    std::vector<uint8> SerializeWritePackage(const NFCWritePackage& package) const;

    std::vector<uint8> SerializeMifareReadPackage(const MifareReadPackage& package) const;

    std::vector<uint8> SerializeMifareWritePackage(const MifareWritePackage& package) const;

    NFCWritePackage MakeAmiiboWritePackage(const TagUUID& tag_uuid, std::span<const uint8> data) const;

    NFCDataChunk MakeAmiiboChunk(uint8 page, uint8 size, std::span<const uint8> data) const;

    MifareReadPackage MakeMifareReadPackage(const MifareUUID& tag_uuid,
                                            std::span<const MifareReadChunk> read_request) const;

    MifareWritePackage MakeMifareWritePackage(const MifareUUID& tag_uuid,
                                              std::span<const MifareWriteChunk> read_request) const;

    NFCReadBlockCommand GetReadBlockCommand(NFCPages pages) const;

    TagUUID GetTagUUID(std::span<const uint8> data) const;

    bool is_enabled{};
    bool is_polling{};
    std::size_t update_counter{};
};

} // namespace InputCommon::Joycon
