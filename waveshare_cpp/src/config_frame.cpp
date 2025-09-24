#include "config_frame.hpp"

namespace USBCANBridge {
    void ConfigFrame::init_fixed_fields() {
        // * [START]
        storage_[to_size_t(ConfigCommandIndex::START)] = to_byte(Constants::START_BYTE);
        // * [HEADER]
        storage_[to_size_t(ConfigCommandIndex::HEADER)] = to_byte(Constants::MSG_HEADER);
        // * [TYPE]
        storage_[to_size_t(ConfigCommandIndex::TYPE)] = to_byte(Type::CONF_VAR);
        // * [CAN_BAUD]
        storage_[to_size_t(ConfigCommandIndex::CAN_BAUD)] = to_byte(CANBaud::SPEED_1000K); // Default 1Mbps
        // * [FRAME_TYPE]
        storage_[to_size_t(ConfigCommandIndex::FRAME_TYPE)] = to_byte(FrameType::STD_VAR); // Default standard variable
        // * [FILTER_ID1-FILTER_ID4] [MASK_ID1-MASK_ID4] (no filtering/masking)
        std::size_t start = to_size_t(ConfigCommandIndex::FILTER_ID_1);
        std::size_t end = to_size_t(ConfigCommandIndex::MASK_ID_4);
        // ! using the following loop to zero out all reserved bytes
        for (std::size_t i = start; i <= end; i++) {
            // remove any filter or mask
            storage_[i] = std::byte{0x00};
            // zero out reserved bytes from BACKUP_0 to BACKUP_3
            storage_[to_size_t(ConfigCommandIndex::BACKUP_0) + (i - start)] = std::byte{0x00};
        }
        // * [CAN_MODE]
        storage_[to_size_t(ConfigCommandIndex::CAN_MODE)] = to_byte(CANMode::NORMAL); // Default normal mode
        // * [AUTO_RTX]
        storage_[to_size_t(ConfigCommandIndex::AUTO_RTX)] = to_byte(RTX::AUTO); // Default automatic retransmission
        // ? [CHECKSUM] - will be calculated on first serialize() call
        dirty_ = true; // mark checksum as dirty
        checksum_ = 0; // initialize checksum to zero
    }

    /**
     * @brief Get the size of the config frame.
     *
     * @return Result<std::size_t>
     */
    Result<std::size_t> ConfigFrame::impl_size() const {
        return Result<std::size_t>::success(Traits::FRAME_SIZE);
    }

    /**
     * @brief Clear the config frame.
     * Resets all fields to default values.
     * Marks the checksum as dirty for recalculation.
     * @return Result<Status>
     */
    Result<Status> ConfigFrame::impl_clear() {
        init_fixed_fields();
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the type of the config frame.
     * For ConfigFrame, this should always be Type::CONF_VAR or Type::CONF_FIXED.
     * @return Result<Type>
     */
    Result<Type> ConfigFrame::impl_get_type() const {
        Type type = static_cast<Type>(storage_[to_size_t(ConfigCommandIndex::TYPE)]);
        return Result<Type>::success(type);
    }
    /**
     * @brief Set the type of the config frame.
     * Validates the type before setting.
     * For ConfigFrame, valid types are Type::CONF_VAR and Type::CONF_FIXED.
     * @param type
     * @return Result<Status>
     */
    Result<Status> ConfigFrame::impl_set_type(const Type& type) {
        auto res = validate_type(type);
        if (res.fail()) {
            return res;
        }
        storage_[to_size_t(ConfigCommandIndex::TYPE)] = std::byte{static_cast<uint8_t>(type)};
        dirty_ = true;
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the frame type of the config frame.
     * For ConfigFrame, this can be any STD_FIXED or EXT_FIXED.
     *
     * @return Result<FrameType>
     */
    Result<FrameType> ConfigFrame::impl_get_frame_type() const {
        FrameType frame_type =
            static_cast<FrameType>(storage_[to_size_t(ConfigCommandIndex::FRAME_TYPE)]);
        return Result<FrameType>::success(frame_type);
    }

    /**
     * @brief Set the frame type of the config frame.
     * Validates the frame type before setting.
     * For ConfigFrame, valid frame types are STD_FIXED or EXT_FIXED.
     *
     * @param frame_type
     * @return Result<Status>
     */
    Result<Status> ConfigFrame::impl_set_frame_type(const FrameType& frame_type) {
        auto res = validate_frame_type(frame_type);
        if (res.fail()) {
            return res;
        }
        storage_[to_size_t(ConfigCommandIndex::FRAME_TYPE)] =
            std::byte{static_cast<uint8_t>(frame_type)};
        dirty_   = true;
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the frame format of the config frame.
     * For ConfigFrame, this can be FrameFmt::CONF_FIXED or FrameFmt::CONF_VAR.
     *
     * @return Result<FrameFmt>
     */
    Result<FrameFmt> ConfigFrame::impl_get_frame_fmt() const {
        FrameFmt frame_fmt =
            static_cast<FrameFmt>(storage_[to_size_t(ConfigCommandIndex::TYPE)]);
        return Result<FrameFmt>::success(frame_fmt);
    }
    /**
     * @brief Set the frame format of the config frame.
     * Validates the frame format before setting.
     * For ConfigFrame, valid frame formats are CONF_FIXED or CONF_VAR.
     *
     * @param frame_fmt
     * @return Result<Status>
     */
    Result<Status> ConfigFrame::impl_set_frame_fmt(const FrameFmt& frame_fmt) {
        // Validate frame format for config frame
        auto validation_result = validate_frame_fmt(frame_fmt);
        if (validation_result.fail()) {
            return validation_result;
        }
        storage_[to_size_t(ConfigCommandIndex::TYPE)] =
            std::byte{static_cast<uint8_t>(frame_fmt)};
        dirty_   = true;
        return Result<Status>::success(Status::SUCCESS);
    }
    // * Wire protocol serialization/deserialization
    /**
     * @brief Serialize the config frame to a byte array.
     * Validates the frame and return a copy of the internal storage with updated checksum.
     *
     * @return Result<StorageType>
     */
    Result<ConfigFrame::StorageType> ConfigFrame::impl_serialize() const {
        // Update checksum if dirty
        auto checksum_res = updateChecksum();

        // validate the entire frame before serialization
        auto validation_res = impl_validate();
        if (validation_res.fail()) {
            // initialize an array with error status for interface consistency
            StorageType error_storage = {to_byte(Status::WBAD_DATA)};
            Result<StorageType> res = Result<StorageType>();
            res.value = error_storage;
            res.status = validation_res.status;
            return res;
        }

        // If everything is valid, proceed with serialization
        StorageType storage_copy = storage_;

        return Result<StorageType>::success(storage_copy);
    }
    /**
     * @brief Deserialize a byte array into the config frame.
     * Validates the input data and updates internal storage if valid.
     *
     * @param data
     * @return Result<Status>
     */
    //Result<Status> ConfigFrame::impl_deserialize(const StorageType& data){}

    // ? Configuration Frame specific Interface
    /**
     * @brief Get the CAN baud rate from the config frame.
     * @see common.hpp for CANBaud enum details and possible values.
     */
    Result<ConfigFrame::BaudType> ConfigFrame::get_baud_rate() const {
        BaudType baud_rate =
            static_cast<BaudType>(storage_[to_size_t(ConfigCommandIndex::CAN_BAUD)]);
        return Result<ConfigFrame::BaudType>::success(baud_rate);
    }
    /**
     * @brief Set the CAN baud rate in the config frame.
     * Validates the baud rate before setting.
     * @see common.hpp for CANBaud enum details and possible values.
     *
     * @param baud_rate
     * @return Result<Status>
     */
    Result<Status> ConfigFrame::set_baud_rate(const BaudType& baud_rate) {
        auto res = validate_baud_rate(baud_rate);
        if (res.fail()) {
            return res;
        }
        storage_[to_size_t(ConfigCommandIndex::CAN_BAUD)] =
            std::byte{static_cast<uint8_t>(baud_rate)};
        dirty_   = true;
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the CAN mode from the config frame.
     * @see common.hpp for CANMode enum details and possible values.
     */
    Result<ConfigFrame::ModeType> ConfigFrame::get_can_mode() const {
        ModeType mode =
            static_cast<ModeType>(storage_[to_size_t(ConfigCommandIndex::CAN_MODE)]);
        return Result<ConfigFrame::ModeType>::success(mode);
    }
    /**
     * @brief Set the CAN mode in the config frame.
     * Validates the mode before setting.
     * @see common.hpp for CANMode enum details and possible values.
     *
     * @param mode
     * @return Result<Status>
     */
    Result<Status> ConfigFrame::set_can_mode(const ModeType& mode) {
        auto res = validate_can_mode(mode);
        if (res.fail()) {
            return res;
        }
        storage_[to_size_t(ConfigCommandIndex::CAN_MODE)] =
            std::byte{static_cast<uint8_t>(mode)};
        dirty_   = true;
        return Result<Status>::success(Status::SUCCESS);
    }
    /**
     * @brief Get the CAN ID filter from the config frame.
     * The filter is a 4-byte value.
     */
    // Result<ConfigFrame::ConfigType> ConfigFrame::get_id_filter() const {}
    /**
     * @brief Set the CAN ID filter in the config frame.
     * Validates the filter before setting.
     * The filter is a 4-byte value.
     * @param filter
     * @return Result<Status>
     */
    //Result<Status> ConfigFrame::set_id_filter(const ConfigType& filter) {}
    /**
     * @brief Get the CAN ID mask from the config frame.
     * The mask is a 4-byte value.
     */
    //Result<ConfigFrame::ConfigType> ConfigFrame::get_id_mask() const {}

}