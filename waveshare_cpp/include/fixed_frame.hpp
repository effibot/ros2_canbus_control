#pragma once

#include "base_frame.hpp"
#include <numeric>

namespace USBCANBridge {
    /**
     * @brief Fixed frame implementation.
     *
     * This class represents a fixed frame in the USBCANBridge library.
     * It inherits from the BaseFrame class and provides specific functionality
     * for fixed frames, such as checksum calculation and validation.
     * @see FixedSizeIndex in common.hpp for details on fixed frame structure.
     * @see BaseFrame for common frame functionalities.
     */
    class FixedFrame : public BaseFrame<FixedFrame> {
        protected:
            using PayloadPair = typename Traits::PayloadPair;
            using IDPair = typename Traits::IDPair;
            alignas(4) StorageType storage_ = {std::byte{0}};
            // dirty bit for lazy evaluation of checksum
            mutable bool dirty_ = true;
            // cached checksum value
            mutable uint8_t checksum_ = 0;
            // cached id size value
            mutable size_t id_size_ = 0;
        private:
            void init_fixed_fields();

        public:
            // <<< Constructors
            /**
             * @brief Default constructor.
             *
             * Initializes a FixedFrame object with default values.
             */
            FixedFrame() : BaseFrame() {
                init_fixed_fields();
            }
            ~FixedFrame() = default;
            // >>> Constructors

            // ? Base Frame Protocol Interface

            Result<std::size_t> impl_size() const;
            Result<Status> impl_clear();
            Result<Type> impl_get_type() const;
            Result<Status> impl_set_type(const Type& type);
            Result<FrameType> impl_get_frame_type() const;
            Result<Status> impl_set_frame_type(const FrameType& frame_type);
            Result<FrameFmt> impl_get_frame_fmt() const;
            Result<Status> impl_set_frame_fmt(const FrameFmt& frame_fmt);

            // * Wire protocol serialization/deserialization
            Result<StorageType> impl_serialize() const;
            Result<Status> impl_deserialize(const StorageType& data);

            // ? Fixed Frame specific Interface
            /**
             * @brief Inline method to check if the frame uses an extended CAN ID,
             * based on the FrameType field. If the cached id_size_ is zero, it retrieves
             * the FrameType and updates id_size_ accordingly (4 for extended, 2 for
             * standard).
             *
             * @return Result<bool>
             */
            Result<bool> is_extended() const {
                if (id_size_ == 0) {
                    auto frame_type_res = impl_get_frame_type();
                    if (frame_type_res.fail()) {
                        return Result<bool>::error(Status::WBAD_FRAME_TYPE);
                    }
                    auto frame_type = frame_type_res.value;
                    id_size_ = (frame_type == FrameType::EXT_FIXED) ? 4 : 2;
                }
                return Result<bool>::success(id_size_ == 4);
            }
            Result<IDPair> impl_get_id() const;
            Result<Status> impl_set_id(const IDPair& id);
            Result<std::byte> impl_get_dlc() const;
            Result<Status> impl_set_dlc(const std::byte& dlc);
            Result<PayloadPair> impl_get_data() const;
            Result<Status> impl_set_data(const PayloadPair& new_data);

            // * Validation methods
            Result<Status> impl_validate() const;

            // ? Individual field validation methods
            Result<Status> validate_start_byte() const;
            Result<Status> validate_header_byte() const;
            Result<Status> validate_type(const Type& type) const;
            Result<Status> validate_frame_type(const FrameType& frame_type) const;
            Result<Status> validate_frame_fmt(const FrameFmt& frame_fmt) const;
            Result<Status> validate_id_size(const IDPair& id) const;
            Result<Status> validate_dlc(const std::byte& dlc) const;
            Result<Status> validate_dlc(const std::size_t& dlc) const;
            Result<Status> validate_reserved_byte() const;

            // ? Checksum specific methods

            // Static methods for raw byte stream processing
            /**
             * @brief Calculate checksum from raw byte stream (static version).
             *
             * This static method calculates the checksum for a fixed frame from raw bytes
             * without requiring a FixedFrame object allocation. It operates on the checksum
             * range: TYPE to RESERVED inclusive (bytes 2-18 of a 20-byte frame).
             *
             * @param data Pointer to the raw byte data (must be at least 20 bytes)
             * @param size Size of the data buffer (must be at least 20 bytes)
             * @return uint8_t The calculated checksum (low 8 bits of sum)
             */
            static uint8_t calculateChecksum(const std::byte* data, std::size_t size) {
                // Validate input parameters
                if (!data || size < 20) {
                    return 0; // or throw an exception based on your error handling strategy
                }

                // Calculate checksum over bytes [TYPE..RESERVED] (indices 2-18)
                constexpr std::size_t start_idx = to_size_t(FixedSizeIndex::TYPE);
                constexpr std::size_t end_idx = to_size_t(FixedSizeIndex::RESERVED);

                uint8_t sum = 0;
                for (std::size_t i = start_idx; i <= end_idx; ++i) {
                    sum += static_cast<uint8_t>(data[i]);
                }

                return sum;
            }

            /**
             * @brief Validate checksum from raw byte stream (static version).
             *
             * This static method validates the checksum of a fixed frame from raw bytes
             * without requiring a FixedFrame object allocation. It calculates the expected
             * checksum and compares it with the stored checksum at position 19.
             *
             * @param data Pointer to the raw byte data (must be at least 20 bytes)
             * @param size Size of the data buffer (must be at least 20 bytes)
             * @return bool True if checksum is valid, false otherwise
             */
            static Result<Status> validateChecksum(const std::byte* data, std::size_t size) {
                // Validate input parameters
                if (!data || size < 20) {
                    return Result<Status>::error(Status::WBAD_LENGTH);
                }

                // Calculate expected checksum
                uint8_t calculated = calculateChecksum(data, size);

                // Get stored checksum from frame
                uint8_t stored = static_cast<uint8_t>(data[to_size_t(FixedSizeIndex::CHECKSUM)]);
                if (calculated != stored) {
                    return Result<Status>::error(Status::WBAD_CHECKSUM);
                }
                return Result<Status>::success(Status::SUCCESS);
            }

            /**
             * @brief Calculate the checksum for a fixed frame over the bytes:
             * type, frame_type, frame_fmt, id[4], dlc, data[8], reserved.
             * The checksum is the low 8 bits of the sum of these bytes
             * @param frame The fixed frame to calculate the checksum for.
             * @return uint8_t The calculated checksum.
             */
            Result<uint8_t> calculateChecksum() const {
                // Check if cached checksum is valid
                if (dirty_) {
                    // Perform fast checksum calculation using std::transform_reduce
                    uint8_t sum = std::transform_reduce(
                        storage_.begin() + to_size_t(FixedSizeIndex::TYPE),
                        storage_.begin() + to_size_t(FixedSizeIndex::RESERVED) + 1,
                        uint8_t{0},        // initial value
                        std::plus<uint8_t>{}, // reduction operation
                        [](std::byte b) {
                            return static_cast<uint8_t>(b);
                        } // transform operation
                    );

                    // Cache the result
                    checksum_ = sum;
                    dirty_ = false;
                }
                return Result<uint8_t>::success(checksum_);
            }
            /**
             * @brief Validate the checksum of the given fixed frame.
             * This validation is intended to be used during deserialization to ensure data integrity. When validating the current object, use impl_validate() instead which also checks other fields.
             * @param frame The fixed frame to validate.
             * @return Result<Status> The result of the validation.
             */
            Result<Status> validateChecksum(const FixedFrame& frame) const {
                // * Calculate checksum and compare to stored value
                auto calc_checksum = frame.calculateChecksum();
                if (calc_checksum.fail()) {
                    return Result<Status>::error(Status::UNKNOWN);
                }
                if (calc_checksum.value != frame.checksum_) {
                    return Result<Status>::error(Status::WBAD_CHECKSUM);
                }
                return Result<Status>::success(Status::SUCCESS);
            }
    };
}