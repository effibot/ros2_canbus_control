#pragma once

#include "base_frame.hpp"

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
			// dirty bit for lazy evaluation of checksum
			bool dirty_ = true;
			// cached checksum value
			uint8_t checksum_ = 0;
		public:
			// <<< Constructors
			/**
			 * @brief Default constructor.
			 *
			 * Initializes a FixedFrame object with default values.
			 */
			FixedFrame() : BaseFrame() {
				// * [HEADER]
				storage_[to_size_t(FixedSizeIndex::HEADER)] = to_byte(Constants::MSG_HEADER);
				// * [TYPE]
				storage_[to_size_t(FixedSizeIndex::TYPE)] = to_byte(Type::DATA_FIXED);
				// * [FRAME_TYPE]
				storage_[to_size_t(FixedSizeIndex::FRAME_TYPE)] = to_byte(FrameType::STD_FIXED);
				// * [FRAME_FMT]
				storage_[to_size_t(FixedSizeIndex::FRAME_FMT)] = to_byte(FrameFmt::DATA_FIXED);
				// * [RESERVED]
				storage_[to_size_t(FixedSizeIndex::RESERVED)] = to_byte(Constants::RESERVED0);
			}
	};
}