#pragma once

#include <stddef.h>
#include <stdint.h>

#include <sstream>
#include <string>
#include <string_view>

#include <tl/expected.hpp>

#include "intrinsic/utils/attributes.hpp"
#include "intrinsic/utils/log.hpp"
#include "intrinsic/utils/status.hpp"
#include "intrinsic/utils/time.hpp"
#include "intrinsic/shared_memory_manager/domain_socket_utils.hpp"
#include "intrinsic/shared_memory_manager/segment_header.hpp"

namespace intrinsic::hal
{

// Each memory segment has to be created and initialized by a
// `SharedMemoryManager`. The Read-Only as well as Read-Write memory segment
// classes below only provide an access handle to these segments, but don't
// create these.
class MemorySegment {
public:
  // Shared memory layout is: SegmentHeader | Data.
  struct SegmentDescriptor
  {
    // The starting address of the shared memory segment. Points to the
    // SegmentHeader.
    // Shared memory layout is described in:
    // intrinsic/icon/interprocess/shared_memory_manager/segment_header.h
    uint8_t * segment_start;
    // The size of the shared memory segment. This is the size of the entire
    // segment, including the SegmentHeader.
    size_t size;
  };

  ~MemorySegment();

  // Returns whether the memory segment is initialized and points to a valid
  // shared memory location. Returns false if the segment class is default
  // constructed.
  bool IsValid() const;

  // Returns the name of the shared memory segment.
  std::string Name() const;

  // Returns the header information of the shared memory segment.
  const SegmentHeader & Header() const;

  // Returns the SegmentHeader of the shared memory segment. May be nullptr!
  SegmentHeader * HeaderPointer();

  // Marks the time that the segment was updated.
  // 'current_cycle' is the control cycle that the segment was updated.
  void UpdatedAt(Time time, uint64_t current_cycle)
  {
    HeaderPointer()->UpdatedAt(time, current_cycle);
  }

  // Returns the size of the value stored in the shared memory segment.
  // Returns 0 if the segment is invalid.
  size_t ValueSize() const;

protected:
  enum class ReadWriteKind
  {
    kUnknown,
    kReadOnly,
    kReadWrite,
  };

  MemorySegment() = default;

  // Accesses the shared memory location with `name` and maps it into
  // user-space.
  // Returns the pointer and the size of the shared memory segment.
  // Returns NotFoundError if the shared memory segment with the given
  // name is not in `segment_name_to_file_descriptor_map` e.g. not previously
  // allocated by a `SharedMemoryManager`.
  // Returns InternalError if mapping the segment fails, or if the size of the
  // segment is too small to hold a SegmentHeader and at least one byte of
  // payload.
  static tl::expected<SegmentDescriptor, Status> Get(
    const SegmentNameToFileDescriptorMap & segment_name_to_file_descriptor_map,
    std::string_view name);

  // Returns the raw, untyped value of the shared memory segment.
  uint8_t * Value();
  const uint8_t * Value() const;

  MemorySegment(
    std::string_view name, SegmentDescriptor segment,
    ReadWriteKind kind,
    const log::Logger * logger INTR_ATTRIBUTE_LIFETIME_BOUND);
  MemorySegment(const MemorySegment & other) = delete;
  MemorySegment & operator=(const MemorySegment & other) = delete;
  MemorySegment(MemorySegment && other) noexcept;
  MemorySegment & operator=(MemorySegment && other) noexcept;

private:
  const log::Logger * logger_ = nullptr;
  // TODO(karstenknese) Put into header.
  std::string name_ = "";

  // The segment header as well as the actual payload (value) are located in the
  // same shared memory segment. We separate the pointers by a simple offset.
  SegmentHeader * header_ = nullptr;
  uint8_t * value_ = nullptr;
  // The size of the shared memory segment. This is the size of the entire
  // segment, including the SegmentHeader.
  size_t size_ = 0;
  ReadWriteKind read_write_kind_ = ReadWriteKind::kUnknown;

  void CleanUpSharedMemory() noexcept;
};

// Checks that the size of the shared memory segment is big enough to
// hold a SegmentHeader and at least sizeof(T) bytes.
// This check is only accurate for trivially_copyable data types like flatbuffer
// structs. The validity of flatbuffer tables is checked in
// intrinsic/icon/hal/get_hardware_interface.h.
// The parameter `segment_size` is the size of the entire
// shared memory segment, including the SegmentHeader.
// The parameter `segment_name` is only used for error reporting.
// Returns InternalError if `segment_size` is too small to hold <T>.
template<class T>
Status SharedMemorySegmentFitsLowerSizeBound(
  std::string_view segment_name, size_t segment_size)
{
  const size_t minimal_size = sizeof(T) + sizeof(SegmentHeader);
  if (segment_size < minimal_size) {
    return {
      .code = StatusCode::kInternal,
      .message = (std::stringstream()
                  << "Shared memory segment '" << segment_name
                  << "' of size " << segment_size
                  << "bytes must be >= " << minimal_size
                  << "bytes. This can be due to a version mismatch of your resources.").str(),
    };
  }
  return OkStatus();
}

// Read-Only access to a shared memory segment of type `T`.
template<class T>
class ReadOnlyMemorySegment final : public MemorySegment {
public:
  // Gets read-only access to a shared memory segment called `segment_name`.
  // Returns NotFoundError if the shared memory segment with the given
  // name is not in `segment_name_to_file_descriptor_map` e.g. not previously
  // allocated by a `SharedMemoryManager`.
  // Returns InternalError if mapping the segment fails, or if the size of the
  // segment is too small to hold a SegmentHeader and at least sizeof(T) bytes.
  static tl::expected<ReadOnlyMemorySegment, Status> Get(
    const SegmentNameToFileDescriptorMap & segment_name_to_file_descriptor_map,
    std::string_view segment_name,
    const log::Logger * logger INTR_ATTRIBUTE_LIFETIME_BOUND)
  {
    auto segment_info =
      MemorySegment::Get(segment_name_to_file_descriptor_map, segment_name);
    if (!segment_info) {
      return tl::unexpected(segment_info.error());
    }
    if (auto status = SharedMemorySegmentFitsLowerSizeBound<T>(
            segment_name, segment_info->size);
      status.code != StatusCode::kOk)
    {
      return tl::unexpected(status);
    }

    return ReadOnlyMemorySegment<T>(segment_name, *segment_info, logger);
  }

  ReadOnlyMemorySegment() = default;
  ReadOnlyMemorySegment(const ReadOnlyMemorySegment & other) = delete;
  ReadOnlyMemorySegment & operator=(
    const ReadOnlyMemorySegment & other) noexcept = delete;
  ReadOnlyMemorySegment(ReadOnlyMemorySegment && other) noexcept = default;
  ReadOnlyMemorySegment & operator=(ReadOnlyMemorySegment && other) noexcept =
  default;

  // Accesses the value of the shared memory segment.
  const T & GetValue() const {return *reinterpret_cast<const T *>(Value());}
  const uint8_t * GetRawValue() const {return Value();}

private:
  ReadOnlyMemorySegment(
    std::string_view name, SegmentDescriptor segment,
    const log::Logger * logger INTR_ATTRIBUTE_LIFETIME_BOUND)
  : MemorySegment(name, segment, MemorySegment::ReadWriteKind::kReadOnly, logger) {}
};

// Read-Write access to a shared memory segment of type `T`.
// The Read-Write is thread-compatible, however there is currently no
// concurrency model implemented, which means that multiple writers are
// potentially introducing a data race when trying to update the same shared
// memory segments at the same time. Note that depending on the data type of the
// shared memory segment, there might also be a race between a single writer and
// a single reader in which the reader might potentially read an inconsistent
// value while the writer updates it. It's therefore the application's
// responsibility to guarantee a safe execution when featuring multiple writers.
template<class T>
class ReadWriteMemorySegment final : public MemorySegment {
public:
  // Gets read-write access to a shared memory segment called `segment_name`.
  // Returns NotFoundError if the shared memory segment with the given
  // name is not in `segment_name_to_file_descriptor_map` e.g. not previously
  // allocated by a `SharedMemoryManager`.
  // Returns InternalError if mapping the segment fails, or if the size of the
  // segment is too small to hold a SegmentHeader and at least sizeof(T) bytes.
  static tl::expected<ReadWriteMemorySegment, Status> Get(
    const SegmentNameToFileDescriptorMap & segment_name_to_file_descriptor_map,
    std::string_view segment_name, const log::Logger * logger INTR_ATTRIBUTE_LIFETIME_BOUND)
  {
    auto segment_info =
      MemorySegment::Get(segment_name_to_file_descriptor_map, segment_name);
    if (!segment_info) {
      return tl::unexpected(segment_info.error());
    }

    if (auto status = SharedMemorySegmentFitsLowerSizeBound<T>(
            segment_name, segment_info->size);
      status.code != StatusCode::kOk)
    {
      return tl::unexpected(status);
    }

    return ReadWriteMemorySegment<T>(segment_name, *segment_info, logger);
  }

  ReadWriteMemorySegment() = default;
  ReadWriteMemorySegment(const ReadWriteMemorySegment & other) = delete;
  ReadWriteMemorySegment & operator=(const ReadWriteMemorySegment & other) =
  delete;
  ReadWriteMemorySegment(ReadWriteMemorySegment && other) noexcept = default;
  ReadWriteMemorySegment & operator=(ReadWriteMemorySegment && other) noexcept =
  default;

  // Accesses the value of the shared memory segment.
  T & GetValue() {return *reinterpret_cast<T *>(Value());}
  const T & GetValue() const {return *reinterpret_cast<const T *>(Value());}
  uint8_t * GetRawValue() {return Value();}
  const uint8_t * GetRawValue() const {return Value();}

  // Updates the value of the shared memory segment.
  // TODO(b/214080423) Add concurrency mechanism to avoid a data race between
  // multiple writers.
  void SetValue(const T & value) {*reinterpret_cast<T *>(Value()) = value;}

private:
  ReadWriteMemorySegment(
    std::string_view name, SegmentDescriptor segment,
    const log::Logger * logger INTR_ATTRIBUTE_LIFETIME_BOUND)
  : MemorySegment(name, segment, MemorySegment::ReadWriteKind::kReadWrite, logger)
  {
  }
};

}  // namespace intrinsic::icon
