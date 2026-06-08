#include "intrinsic/shared_memory_manager/segment_header.hpp"

#include <semaphore.h>

#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string>

#include "intrinsic/utils/log.hpp"

namespace intrinsic::hal
{
namespace
{
void IncrementRefCount(int & counter, sem_t * mutex)
{
  sem_wait(mutex);
  ++counter;
  sem_post(mutex);
}

void DecrementRefCount(int & counter, sem_t * mutex)
{
  sem_wait(mutex);
      // A reference counter can't be lower than zero.
  if (counter > 0) {
    --counter;
  }
  sem_post(mutex);
}
}   // namespace

SegmentHeader::SegmentHeader() noexcept
: logger_(nullptr), type_info_(TypeInfo("UNDEFINED")), flags_(0)
{
  // Initialize the unnamed semaphore as process-shared by setting second
  // argument to non-zero. See
  // https://man7.org/linux/man-pages/man3/sem_init.3.html for details.
  // We can feature an unnamed semaphore here as this header information will
  // be part of the shared memory segment and thus shared between multiple
  // processes.
  sem_init(&mutex_, 1, 1);
}

SegmentHeader::SegmentHeader(
  const std::string & type_id,
  const log::Logger * logger) noexcept
: logger_(logger), type_info_(TypeInfo(type_id)), flags_(0)
{
    // Initialize the unnamed semaphore as process-shared by setting second
    // argument to non-zero. See
    // https://man7.org/linux/man-pages/man3/sem_init.3.html for details.
    // We can feature an unnamed semaphore here as this header information will
    // be part of the shared memory segment and thus shared between multiple
    // processes.
  sem_init(&mutex_, 1, 1);
}

SegmentHeader::SegmentHeader(
  const std::string & type_id,
  const std::initializer_list<Flags> & flags,
  const log::Logger * logger) noexcept
: SegmentHeader(type_id, logger)
{
  for (const auto flag : flags) {
    flags_.set(static_cast<int>(flag));
  }
}

SegmentHeader::~SegmentHeader() noexcept
{
  if (ref_count_reader_ != 0 || ref_count_writer_ != 0) {
    INTRINSIC_SHARED_MEMORY_LOG(
        WARNING,
        logger_,
        "Shared memory segment cleaned up while being used by %d other entities.",
      (ref_count_reader_ + ref_count_writer_));
  }

  flags_.reset();
  sem_destroy(&mutex_);
}

int SegmentHeader::ReaderRefCount() const {return ref_count_reader_;}
void SegmentHeader::IncrementReaderRefCount()
{
  IncrementRefCount(ref_count_reader_, &mutex_);
}
void SegmentHeader::DecrementReaderRefCount()
{
  DecrementRefCount(ref_count_reader_, &mutex_);
}

int SegmentHeader::WriterRefCount() const {return ref_count_writer_;}
void SegmentHeader::IncrementWriterRefCount()
{
  return IncrementRefCount(ref_count_writer_, &mutex_);
}
void SegmentHeader::DecrementWriterRefCount()
{
  return DecrementRefCount(ref_count_writer_, &mutex_);
}

SegmentHeader::TypeInfo SegmentHeader::Type() const {return type_info_;}

bool SegmentHeader::FlagIsSet(SegmentHeader::Flags flag) const
{
  return flags_.test(static_cast<int>(flag));
}

Time SegmentHeader::LastUpdatedTime() const {return last_updated_time_;}

int64_t SegmentHeader::NumUpdates() const {return update_counter_;}

uint64_t SegmentHeader::LastUpdatedCycle() const {return updated_at_cycle_;}

void SegmentHeader::UpdatedAt(Time time, uint64_t current_cycle)
{
  if (time < last_updated_time_) {
    INTRINSIC_SHARED_MEMORY_LOG(
          WARNING,
          logger_,
          "Update for segment of type '%s' goes backwards in time.",
          type_info_.TypeID());
  }
  last_updated_time_ = time;
  updated_at_cycle_ = current_cycle;
    // Not using update_counter_ =
    // static_cast<int64_t>(static_cast<uint64_t>(update_counter_) + 1);
    // Because it is compiler dependent.
  if (update_counter_ == std::numeric_limits<int64_t>::max()) {
    [[unlikely]] {
      update_counter_ = std::numeric_limits<int64_t>::min();
    }
  } else {
    update_counter_++;
  }
}
}
