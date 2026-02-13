#include "icon_hwm_controller/shared_memory_manager.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <string_view>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tl/expected.hpp>
#include <unistd.h>
#include <unordered_map>
#include <utility>
#include <vector>

#include "flatbuffer_utils.hpp"
#include "icon_hwm_controller/status.hpp"

namespace intrinsic::hal {


using ::intrinsic_fbs::FlatbufferArrayNumElements;

// Max string size as defined in `segment_info.fbs`
inline constexpr size_t kMaxSegmentStringSize =
    FlatbufferArrayNumElements(&intrinsic_fbs::SegmentName::value);

inline constexpr size_t kMaxNumberOfSegments =
    FlatbufferArrayNumElements(&intrinsic_fbs::SegmentInfo::names);

namespace {
Status VerifyName(std::string_view name) {
  if (name.empty()) {
    return {
      .code=StatusCode::kInvalidArgument,
      .message="Shm segment name cannot be empty.",
    };
  }
  if (name.size() >= kMaxSegmentStringSize) {
    std::stringstream error;
    error << "Shm segment name \"" << name << "\" can't exceed " <<
        (kMaxSegmentStringSize - 1) << " characters.";
    return {
      .code=StatusCode::kInvalidArgument,
      .message=error.str(),
    };
  }

  if (std::find(name.begin(), name.end(), '/') != name.end()) {
    std::stringstream error;
    error << "Shm segment name \"" << name << "\" can't have forward slashes.";
    return {
      .code=StatusCode::kInvalidArgument,
      .message=error.str(),
    };
  }

  return OkStatus();
}

intrinsic_fbs::SegmentInfo SegmentInfoFromHashMap(
    const std::unordered_map<
        std::string, SharedMemoryManager::MemorySegmentInfo>& segments) {
  intrinsic_fbs::SegmentInfo segment_info(segments.size());
  uint32_t index = 0;
  for (const auto& [memory_name, buf] : segments) {
    intrinsic_fbs::SegmentName segment;
    segment.mutate_must_be_used(buf.must_be_used);
    // fbs doesn't have char as datatype, only int8_t which is byte compatible.
    auto* data = reinterpret_cast<char*>(segment.mutable_value()->Data());
    std::memset(data, '\0', kMaxSegmentStringSize);
    std::snprintf(data, kMaxSegmentStringSize, "%s", memory_name.c_str());
    segment_info.mutable_names()->Mutate(index, segment);
    ++index;
  }

  return segment_info;
}
}  // namespace

SharedMemoryManager::SharedMemoryManager(
    std::string_view module_name, std::string_view shared_memory_namespace)
    : module_name_(std::string(module_name)),
      shared_memory_namespace_(std::string(shared_memory_namespace)) {}

// static
tl::expected<std::unique_ptr<SharedMemoryManager>, Status>
SharedMemoryManager::Create(std::string_view shared_memory_namespace,
                            std::string_view module_name) {
  if (module_name.empty()) {
    return tl::unexpected(Status{
            .code=StatusCode::kInvalidArgument,
            .message="Module name can't be empty.",
      });
  }

  return std::unique_ptr<SharedMemoryManager>(new SharedMemoryManager(
      /*module_name=*/module_name,
      /*shared_memory_namespace=*/shared_memory_namespace));
}

SharedMemoryManager::~SharedMemoryManager() {
  // unlink all created shm segments
  for (const auto& segment : memory_segments_) {
    auto* header = reinterpret_cast<SegmentHeader*>(segment.second.data);
    int fd = segment.second.fd;
    const std::string segment_name = segment.first;

    // We've used placement new during the initialization. We have to call the
    // destructor explicitly to cleanup.
    header->~SegmentHeader();

    if (close(fd) == -1) {
      // TODO(nilsb): Proper logging
      std::cerr << "Failed to close shm_fd for '" << segment_name
                << "'. with error: " << strerror(errno)
                << ". Continuing anyways." << std::endl;
      /*
      LOG(WARNING) << "Failed to close shm_fd for '" << segment_name
                   << "'. with error: " << strerror(errno)
                   << ". Continuing anyways.";
      */
    }
    if (segment.second.data != nullptr) {
      if (munmap(segment.second.data, segment.second.length) == -1) {
        // TODO(nilsb): Proper logging
        std::cerr << "Failed to unmap memory for '" << segment_name
                  << "'. with error: " << strerror(errno)
                  << ". Continuing anyways.";
        /*
          LOG(WARNING) << "Failed to unmap memory for '" << segment_name
                     << "'. with error: " << strerror(errno)
                     << ". Continuing anyways.";
        */
      }
    }
  }
}

const SegmentHeader* SharedMemoryManager::GetSegmentHeader(
    std::string_view name) {
  uint8_t* header = GetRawSegment(name);
  return reinterpret_cast<SegmentHeader*>(header);
}

Status SharedMemoryManager::InitSegment(std::string_view name,
                                              bool must_be_used,
                                              size_t payload_size,
                                              const std::string& type_id) {
  if (memory_segments_.size() >= kMaxNumberOfSegments) {
    std::stringstream error;
    error << "Unable to add \"" << name << "\". Max number of segments (" <<
        kMaxNumberOfSegments << ") exceeded.";
    return {
      .code=StatusCode::kResourceExhausted,
      .message=error.str(),
    };
  }
  if (type_id.size() > SegmentHeader::TypeInfo::kMaxSize) {
    std::stringstream error;
    error << "Type id [" << type_id << "] exceeds max size of [" <<
        SegmentHeader::TypeInfo::kMaxSize << "].";
    return {
      .code=StatusCode::kInvalidArgument,
      .message=error.str(),
    };
  }
  if (memory_segments_.contains(std::string(name))) {
    std::stringstream error;
    error << "Shm segment \"" << name << "\" exists already.";
    return {
      .code=StatusCode::kAlreadyExists,
      .message=error.str(),
    };
  }
  if (auto status = VerifyName(name); !status.ok()) {
    return status;
  }

  // Creates an anonymous memory segment and stores the fd
  // https://man7.org/linux/man-pages/man2/memfd_create.2.html
  // Default flags are O_RDWR | O_LARGEFILE.
  int shm_fd = memfd_create(name.data(), 0);
  if (shm_fd == -1) {
    ;
    return {
      .code=StatusCode::kInternal,
      .message=(
          std::stringstream()
          << "Failed to create shm segment \"" << name
          << "\" with error: " << strerror(errno) << ".")
      .str(),
    };
  }

  const auto segment_size = sizeof(SegmentHeader) + payload_size;
  if (ftruncate(shm_fd, segment_size) == -1) {
    // Resizes new shm segments.
    return {
      .code=StatusCode::kInternal,
      .message=(
          std::stringstream()
          << "Unable to resize shared memory segment \"" << name
          << "\" with error: " << strerror(errno) << ".")
      .str(),
    };
  }

  struct stat shared_memory_stats;
  if (fstat(shm_fd, &shared_memory_stats) != 0) {
    // Return an error and forward errno
    return {
      .code=StatusCode::kInternal,
      .message=(
          std::stringstream()
          << "Failed to read size of segment \"" << name
          << "\". 'fstat' failed with:" << strerror(errno)
                ).str(),
    };
  }
  // The opening logic depends on the size of the segment.
  if (shared_memory_stats.st_size != segment_size) {
    return {
      .code=StatusCode::kInternal,
      .message=(
          std::stringstream()
          << "The size of the shared memory segment \"" << name << "\" of " <<
          shared_memory_stats.st_size << "bytes is not the expected size of " <<
          segment_size << "bytes."
                ).str(),
    };
  }

  auto* data =
      static_cast<uint8_t*>(mmap(nullptr, segment_size, PROT_READ | PROT_WRITE,
                                 MAP_SHARED | MAP_LOCKED, shm_fd, 0));
  if (data == nullptr || data == MAP_FAILED) {
    return {
      .code=StatusCode::kInternal,
      .message=(
          std::stringstream()
          << "Unable to map shared memory segment \"" << name <<
                     "\" with error: " << strerror(errno) << "."
                ).str(),
    };
  }

  // Additionally locking the pages as recommended by
  // https://man7.org/linux/man-pages/man2/mmap.2.html, because major faults are
  // not acceptable after the initialization of the mapping.
  if (mlock(/*__addr=*/data, /*__len=*/segment_size) != 0) {
    return {
      .code=StatusCode::kInternal,
      .message=(
          std::stringstream()
          << "Unable to mlock shared memory segment \"" << name <<
                     "\" with error: " << strerror(errno) << "."
                ).str(),
    };
  }

  const std::string name_str(name);
  segment_name_to_file_descriptor_map_.insert({name_str, shm_fd});
  // We use a placement new operator here to initialize the "raw" segment
  // data correctly.
  new (data) SegmentHeader(type_id);
  memory_segments_.insert({
      name_str,
      {
          .data = data,
          .length = segment_size,
          .must_be_used = must_be_used,
          .fd = shm_fd,
      },
  });
  return OkStatus();
}

uint8_t* SharedMemoryManager::GetRawValue(std::string_view name) {
  auto* data = GetRawSegment(name);
  if (data == nullptr) {
    return data;
  }
  return data + sizeof(SegmentHeader);
}

uint8_t* SharedMemoryManager::GetRawSegment(std::string_view name) {
  auto result = memory_segments_.find(std::string(name));
  if (result == memory_segments_.end()) {
    return nullptr;
  }
  return result->second.data;
}

std::vector<std::string> SharedMemoryManager::GetRegisteredMemoryNames() const {
  std::vector<std::string> result;
  result.reserve(memory_segments_.size());
  for (const auto& [name, unused] : memory_segments_) {
    result.push_back(name);
  }
  return result;
}

std::string SharedMemoryManager::ModuleName() const { return module_name_; }

std::string SharedMemoryManager::SharedMemoryNamespace() const {
  return shared_memory_namespace_;
}

intrinsic_fbs::SegmentInfo SharedMemoryManager::GetSegmentInfo() const {
  return SegmentInfoFromHashMap(memory_segments_);
}

}
