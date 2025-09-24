#include "fixed_frame.hpp"
#include <algorithm>

namespace USBCANBridge {
    /**
     * @brief Initialization of fixed fields in the storage array.
     * Sets constant fields to their default values.
     * This function is called during construction to ensure the frame is
     * properly initialized.
     */
    void FixedFrame::init_fixed_fields() {
        // ? [START] is already set by BaseFrame constructor
        // * [HEADER]
        storage_[to_size_t(FixedSizeIndex::HEADER)] = to_byte(Constants::MSG_HEADER);
        // * [TYPE]
        storage_[to_size_t(FixedSizeIndex::TYPE)] = to_byte(Type::DATA_FIXED);
        // * [FRAME_TYPE]
        storage_[to_size_t(FixedSizeIndex::FRAME_TYPE)] = to_byte(FrameType::STD_FIXED);
        id_size_ = 2; // default to standard ID size
        // * [FRAME_FMT]
        storage_[to_size_t(FixedSizeIndex::FRAME_FMT)] = to_byte(FrameFmt::DATA_FIXED);
        // * [RESERVED]
        storage_[to_size_t(FixedSizeIndex::RESERVED)] = to_byte(Constants::RESERVED0);
        // ? [CHECKSUM] - will be calculated on first serialize() call
        dirty_ = true; // mark checksum as dirty
        checksum_ = 0; // initialize checksum to zero
    }

    // ? Base Frame Protocol Interface
    /**
     * @brief Get the size of the fixed frame.
     *
     * @return Result<std::size_t>
     */
    Result<std::size_t> FixedFrame::impl_size() const {
        return Result<std::size_t>::success(Traits::FRAME_SIZE);
    }
    /**
     * @brief Clear the frame to default state.
     * Resets all fields to zero and reinitializes fixed fields.
     * Marks the checksum as dirty for recalculation.
     *
     * @return Result<void>
     */
    Result<Status> FixedFrame::impl_clear() {
        // Reset to default state
        std::fill(storage_.begin(), storage_.end(), 0);
        // Reinitialize fixed fields
        init_fixed_fields();
        // Mark checksum as dirty
        dirty_ = true;
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the Type field of the frame.
     *
     * For FixedFrame, this should always be Type::DATA_FIXED.
     *
     * @return Result<Type>
     */
    Result<Type> FixedFrame::impl_get_type() const {
        return Result<Type>::success(
            static_cast<Type>(
                storage_[to_size_t(FixedSizeIndex::TYPE)]
            )
        );
    }
    /**
     * @brief Set the Type field of the frame.
     *
     * For FixedFrame, this should always be Type::DATA_FIXED.
     *
     * @param type
     * @return Result<Status>
     */
    Result<Status> FixedFrame::impl_set_type(const Type& type) {
        // Validate type for fixed frame
        if (type != Type::DATA_FIXED) {
            return Result<Status>::error(Status::WBAD_TYPE);
        }
        storage_[to_size_t(FixedSizeIndex::TYPE)] = to_byte(type);
        // Mark checksum as dirty
        dirty_ = true;
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the FrameType field of the frame.
     *
     * For FixedFrame, this can be FrameType::STD_FIXED or FrameType::EXT_FIXED.
     *
     * @return Result<FrameType>
     */
    Result<FrameType> FixedFrame::impl_get_frame_type() const {
        return Result<FrameType>::success(
            static_cast<FrameType>(storage_[to_size_t(FixedSizeIndex::FRAME_TYPE)])
        );
    }
    /**
     * @brief Set the FrameType field of the frame.
     * For FixedFrame, only FrameType::STD_FIXED and FrameType::EXT_FIXED are valid.
     * When setting the FrameType, the cached id_size_ is re-evaluated by calling the is_extended() method automatically.
     * @param frame_type New FrameType value.
     * @return Result<Status>
     */
    Result<Status> FixedFrame::impl_set_frame_type(const FrameType& frame_type) {
        // Validate frame type for fixed frame
        if (frame_type != FrameType::STD_FIXED && frame_type != FrameType::EXT_FIXED) {
            return Result<Status>::error(Status::WBAD_TYPE);
        }
        storage_[to_size_t(FixedSizeIndex::FRAME_TYPE)] = to_byte(frame_type);
        // Mark checksum as dirty
        dirty_ = true;
        // Cache the ID size based on type
        id_size_ = 0; // reset cached id_size_ to force re-evaluation
        is_extended(); // update id_size_ based on current FrameType
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the FrameFmt field of the frame.
     *
     * For FixedFrame, this can be FrameFmt::DATA_FIXED or FrameFmt::REMOTE_FIXED.
     *
     * @return Result<FrameFmt>
     */
    Result<FrameFmt> FixedFrame::impl_get_frame_fmt() const {
        return Result<FrameFmt>::success(
            static_cast<FrameFmt>(storage_[to_size_t(FixedSizeIndex::FRAME_FMT)])
        );
    }
    /**
     * @brief Set the FrameFmt field of the frame.
     *
     * For FixedFrame, this should be either FrameFmt::DATA_FIXED or FrameFmt::REMOTE_FIXED.
     *
     * @param frame_fmt
     * @return Result<Status>
     */
    Result<Status> FixedFrame::impl_set_frame_fmt(const FrameFmt& frame_fmt) {
        // Validate frame format for fixed frame
        if (frame_fmt != FrameFmt::DATA_FIXED && frame_fmt != FrameFmt::REMOTE_FIXED) {
            return Result<Status>::error(Status::WBAD_FORMAT);
        }
        storage_[to_size_t(FixedSizeIndex::FRAME_FMT)] = to_byte(frame_fmt);
        // Mark checksum as dirty
        dirty_ = true;
        return Result<Status>::success(Status::SUCCESS);
    }

    // * Wire protocol serialization/deserialization
    // Result<StorageType> FixedFrame::impl_serialize() const;
    // Result<Status> FixedFrame::impl_deserialize(const StorageType& data);

    // ? Fixed Frame specific Interface
    /**
     * @brief Get the ID field of the frame as a pair of byte array
     * and the size allowed by the current FrameType value.
     *
     * @return Result<FixedFrame::IDPair>
     */
    Result<FixedFrame::IDPair> FixedFrame::impl_get_id() const {
        IDPair id;
        // ! find how many bytes to copy based on FrameType
        size_t n_bytes = is_extended() ? 4 : 2;
        // Copy ID bytes from storage
        std::copy_n(
            storage_.begin() + to_size_t(FixedSizeIndex::ID_0),
            n_bytes,
            id.first.begin()
        );
        id.second = n_bytes;
        return Result<FixedFrame::IDPair>::success(id);
    }
    /**
     * @brief Set the ID field of the frame.
     * When invoked, this method validates the size of the provided ID
     * against the current FrameType (2 bytes for standard, 4 bytes for extended).
     *
     * @param id
     * @return Result<Status>
     */
    Result<Status> FixedFrame::impl_set_id(const IDPair& id) {
        // Validate ID size against current FrameType
        size_t expected_size = is_extended().value ? 4 : 2;
        if (id.second != expected_size) {
            return Result<Status>::error(Status::WBAD_ID);
        }
        // Copy ID bytes into storage
        std::copy_n(
            id.first.begin(),
            id.second,
            storage_.begin() + to_size_t(FixedSizeIndex::ID_0)
        );
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the Data Length Code (DLC) field of the frame.
     *
     * @return Result<std::byte>
     */
    Result<std::byte> FixedFrame::impl_get_dlc() const {
        return Result<std::byte>::success(
            storage_[to_size_t(FixedSizeIndex::DLC)]
        );
    }
    /**
     * @brief Set the Data Length Code (DLC) field of the frame.
     * Validates that the DLC is in the range 0-8.
     * Marks the checksum as dirty for recalculation.
     * This will directly affect how many data bytes are considered valid.
     * @param dlc
     * @return Result<Status>
     */
    Result<Status> FixedFrame::impl_set_dlc(const std::byte& dlc) {
        // Validate DLC range (0-8)
        auto dlc_value = static_cast<uint>(dlc);
        if (dlc_value > 8) {
            return Result<Status>::error(Status::WBAD_DLC);
        }
        storage_[to_size_t(FixedSizeIndex::DLC)] = dlc;
        // Mark checksum as dirty
        dirty_ = true;
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get a copy of the entire data payload and DLC from the frame.
     *
     * @return Result<FixedFrame::PayloadPair>
     */
    Result<FixedFrame::PayloadPair> FixedFrame::impl_get_data() const {
        PayloadPair data;
        // Copy DLC
        data.second = to_size_t(storage_[to_size_t(FixedSizeIndex::DLC)]);
        // Copy data payload
        std::copy_n(
            storage_.begin() + to_size_t(FixedSizeIndex::DATA_0),
            data.second,
            data.first.begin()
        );
        return Result<FixedFrame::PayloadPair>::success(data);
    }
    /**
     * @brief Set a new data payload and DLC for the frame.
     *
     * @param new_data
     * @return Result<Status>
     */
    Result<Status> FixedFrame::impl_set_data(const PayloadPair& new_data) {
        // Validate DLC range
        if (new_data.second > 8) {
            return Result<Status>::error(Status::WBAD_DLC);
        }
        // Copy new bytes into the frame data field
        auto frame_start = storage_.begin() + to_size_t(FixedSizeIndex::DATA_0);
        std::copy(new_data.first.begin(), new_data.first.begin() + new_data.second, frame_start);
        // Zero-fill unused bytes
        std::fill(frame_start + new_data.second, frame_start + 8, std::byte{0});
        // Update DLC
        storage_[to_size_t(FixedSizeIndex::DLC)] = to_byte(new_data.second);
        // Mark dirty for checksum recalculation
        dirty_ = true;
        return Result<Status>::success(Status::SUCCESS);
    }

    // * Validation methods
    /**
     * @brief Validate the integrity of the frame.
     * A frame is considered valid if:
     * - Start byte is 0xAA
     * - Header byte is 0x55
     * - Type is DATA_FIXED
     * - FrameType is STD_FIXED or EXT_FIXED
     * - FrameFmt is DATA_FIXED or REMOTE_FIXED
     * - DLC is in range 0-8
     * - Reserved byte is 0x00
     * - Checksum matches the computed checksum
     *
     * @return Result<Status>
     */
    Result<Status> FixedFrame::impl_validate() const {
        // Validate start byte
        if (storage_[to_size_t(FixedSizeIndex::START)] != to_byte(Constants::START_BYTE)) {
            return Result<Status>::error(Status::WBAD_START);
        }
        // Validate header byte
        if (storage_[to_size_t(FixedSizeIndex::HEADER)] != to_byte(Constants::MSG_HEADER)) {
            return Result<Status>::error(Status::WBAD_HEADER);
        }
        // Validate type
        if (storage_[to_size_t(FixedSizeIndex::TYPE)] != to_byte(Type::DATA_FIXED)) {
            return Result<Status>::error(Status::WBAD_TYPE);
        }
        // Validate frame type
        FrameType frame_type =
            static_cast<FrameType>(storage_[to_size_t(FixedSizeIndex::FRAME_TYPE)]);
        if (frame_type != FrameType::STD_FIXED && frame_type != FrameType::EXT_FIXED) {
            return Result<Status>::error(Status::WBAD_FRAME_TYPE);
        }
        // Validate frame format
        FrameFmt frame_fmt = static_cast<FrameFmt>(storage_[to_size_t(FixedSizeIndex::FRAME_FMT)]);
        if (frame_fmt != FrameFmt::DATA_FIXED && frame_fmt != FrameFmt::REMOTE_FIXED) {
            return Result<Status>::error(Status::WBAD_FORMAT);
        }
        // Validate DLC
        uint8_t dlc = static_cast<uint8_t>(storage_[to_size_t(FixedSizeIndex::DLC)]);
        if (dlc > 8) {
            return Result<Status>::error(Status::WBAD_DLC);
        }
        // Validate reserved byte
        if (storage_[to_size_t(FixedSizeIndex::RESERVED)] != to_byte(Constants::RESERVED0)) {
            return Result<Status>::error(Status::WBAD_RESERVED);
        }
        // Validate checksum
        uint8_t computed_checksum = 0;
        for (size_t i = to_size_t(FixedSizeIndex::HEADER); i < to_size_t(FixedSizeIndex::CHECKSUM);
            ++i) {
            computed_checksum += static_cast<uint8_t>(storage_[i]);
        }
        if (computed_checksum !=
            static_cast<uint8_t>(storage_[to_size_t(FixedSizeIndex::CHECKSUM)])) {
            return Result<Status>::error(Status::WBAD_CHECKSUM);
        }
        return Result<Status>::success(Status::SUCCESS);
    }



}