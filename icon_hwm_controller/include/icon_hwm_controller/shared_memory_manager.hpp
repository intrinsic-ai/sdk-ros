#pragma once

#include <string>
#include <type_traits>
#include <string_view>
#include <memory>
#include <utility>
#include <vector>
#include <unordered_map>
#include <tl/expected.hpp>

#include "status.hpp"
#include "domain_socket_utils.hpp"
#include "memory_segment.hpp"
#include "segment_header.hpp"

namespace intrinsic::hal
{

  // A type `T` is suited for shared memory if it's trivially copyable (no heap
  // allocation internally) and is not a pointer type.
template<class T>
inline void AssertSharedMemoryCompatibility()
{
  static_assert(
        std::is_trivially_copyable_v<T>,
        "only trivially copyable data types are supported as shm segments");
  static_assert(!std::is_pointer_v<T>,
                  "pointer types are not supported as shm segments");
}

// The `SharedMemoryManager` creates and administers a set of anonymous shared
// memory segments.
//
// Creates segments as anonymous files using `memfd_create` (see
// https://man7.org/linux/man-pages/man2/memfd_create.2.html).
//
// Each allocated segmented is prefixed with a `SegmentHeader` to store some
// meta information about the allocated segment such as a reference counting.
// The overall data layout of each segment looks thus like the following:
//
// [SegmentHeader][Payload T]
// ^              ^
// Header()       Value()
//
// The manager additionally maintains a map of all allocated segments for
// further introspection of the segments. Once the manager goes out of scope, it
// unlinks all allocated memory; The kernel then eventually deletes the shared
// memory files once there's no further process using them.
// Once a segment is added via `AddSegment` it is fully initialized with a
// default value or any given value.
class SharedMemoryManager final {
public:
  struct MemorySegmentInfo
  {
    uint8_t * data = nullptr;
    // Required for unmapping the segment
    size_t length = 0;
    // A value of true indicates that this segment needs to be used
    // by ICON.
    bool must_be_used = false;
    // The file descriptor of the anonymous memory.
    int fd = -1;
  };

  // Creates a new `SharedMemoryManager`.
  // Returns an error of the module name is empty.
  // Returns unique_ptr because that allows taking stable references e.g. for
  // intrinsic/icon/hal/hardware_interface_registry.h.
  static tl::expected<std::unique_ptr<SharedMemoryManager>, Status> Create(
    std::string_view shared_memory_namespace, std::string_view module_name);

  SharedMemoryManager() = delete;

  // This class is move-only.
  SharedMemoryManager(const SharedMemoryManager & other) = delete;
  SharedMemoryManager & operator=(const SharedMemoryManager & other) = delete;
  // We need to clear other.memory_segments_ on moves in order to avoid
  // use-after-move bugs when accessing memory_segments_ in
  // ~SharedMemoryManager.
  SharedMemoryManager(SharedMemoryManager && other) noexcept
  : memory_segments_(std::move(other.memory_segments_))
  {
    other.memory_segments_.clear();
  }
  SharedMemoryManager & operator=(SharedMemoryManager && other) noexcept
  {
    memory_segments_ = std::move(other.memory_segments_);
    other.memory_segments_.clear();
    return *this;
  }
  // Closes all shared memory segments.
  ~SharedMemoryManager();

  // Provides  access to the shared memory location specified by `segment_name`.
  // Returns NotFoundError if no such segment has been added.
  // Forwards mapping errors.
  template<class MemorySegmentT>
  tl::expected<MemorySegmentT, Status> Get(std::string_view segment_name) const
  {
    static_assert(
        std::is_base_of_v<MemorySegment, MemorySegmentT>,
        "Template parameter for SharedMemoryManager::Get() must inherit from "
        "::intrinsic::hal::MemorySegment");
    return MemorySegmentT::Get(segment_name_to_file_descriptor_map_,
                               segment_name);
  }

  // Reference to the internal map of segment names to file descriptors.
  // Contains the names and file descriptors of all segments that are currently
  // registered with the SharedMemoryManager.
  // For tests where no HardwareModule Proxy is used.
  const SegmentNameToFileDescriptorMap & GetSegmentNameToFileDescriptorMap() const
  {
    return segment_name_to_file_descriptor_map_;
  }

  // Allocates a shared memory segment for the type `T` and initializes it with
  // the default value of `T`.
  // The type must be trivially copyable and not a pointer type; other types
  // fail to compile.
  // The name for the segment should be POSIX conform, in which the length is
  // not to exceed 255 characters.
  // The value of `must_be_used` indicates whether this segment needs to be
  // used by ICON.
  // Similarly, one can optionally pass in a type identifier string to uniquely
  // describe the type of the data segment. The string can't exceed a max length
  // of `SegmentHeader::TypeInfo::kMaxSize` and defaults to a compiler generated
  // typeid. Please note that the compiler generated default is not defined by
  // the C++ standard and thus may not conform across process boundaries with
  // different compilers.
  // Returns `InvalidArgumentError` if the name is not valid.
  // Returns `AlreadyExistsError` if the shared memory segment
  // with this name already exists
  // Returns `InternalError` if the underlying POSIX call fails.
  // Returns `OkStatus` is the shared memory segment was successfully
  // allocated.
  template<class T>
  Status AddSegmentWithDefaultValue(
    std::string_view name,
    bool must_be_used)
  {
    return AddSegmentWithDefaultValue<T>(name, must_be_used, typeid(T).name());
  }
  template<class T>
  Status AddSegmentWithDefaultValue(
    std::string_view name,
    bool must_be_used,
    const std::string & type_id)
  {
    AssertSharedMemoryCompatibility<T>();
    if (auto status = InitSegment(name, must_be_used, sizeof(T), type_id);
        status.code != StatusCode::kOk) {
      return status;
    }
    return SetSegmentValue(name, T());
  }

  // Allocates a shared memory segment for the type `T` and initializes it with
  // the specified value of `T`.
  // Besides the initialized value for the segment, this function behaves
  // exactly like `AddSegment` above.
  template<class T>
  Status AddSegment(
    std::string_view name, bool must_be_used,
    const T & value)
  {
    return AddSegment<T>(name, must_be_used, value, typeid(T).name());
  }
  template<class T>
  Status AddSegment(
    std::string_view name, bool must_be_used,
    const T & value, const std::string & type_id)
  {
    AssertSharedMemoryCompatibility<T>();
    if (auto status = InitSegment(name, must_be_used, sizeof(T), type_id);
        status.code != StatusCode::kOk) {
      return status;
    }
    return SetSegmentValue(name, value);
  }
  template<class T>
  Status AddSegment(
    std::string_view name, bool must_be_used,
    T && value)
  {
    return AddSegment<T>(name, must_be_used, std::forward<T>(value),
                         typeid(T).name());
  }
  template<class T>
  Status AddSegment(
    std::string_view name, bool must_be_used, T && value,
    const std::string & type_id)
  {
    if (auto status = InitSegment(name, must_be_used, sizeof(T), type_id);
        status.code != StatusCode::kOk) {
      return status;
    }
    return SetSegmentValue(name, std::forward<T>(value));
  }

  // Allocates a generic memory segment with a byte (uint8_t) array payload of
  // `payload_size` bytes.
  Status AddSegment(
    std::string_view name, bool must_be_used,
    size_t payload_size)
  {
    return AddSegment(name, must_be_used, payload_size, typeid(uint8_t).name());
  }
  // Allocates a memory segment of type `type_id` with a payload of
  // `payload_size` bytes.
  Status AddSegment(
    std::string_view name, bool must_be_used,
    size_t payload_size, const std::string & type_id)
  {
    return
      InitSegment(name, must_be_used, payload_size, type_id);
  }

  // Returns the `SegmentHeader` belonging to the shared memory segment
  // specified by the given name.
  // Returns null pointer if the segment with the given name does not exist.
  const SegmentHeader * GetSegmentHeader(std::string_view name);

  // Returns the value belonging to the shared memory segment specified by the
  // given name.
  // Returns `nullptr` if the segment with the given name does
  // not exist.
  // Note, the type `T` has to match the type with which the segment was
  // originally created. This function leads to undefined behavior otherwise.
  template<class T>
  const T * GetSegmentValue(std::string_view name)
  {
    return reinterpret_cast<T *>(GetRawValue(name));
  }

  // Copies the new value into an existing shared memory segment.
  // Returns `NotFoundError` if the segment with the given name does
  // not exist.
  // Note, the type `T` has to match the type with which the segment was
  // originally created. This function leads to undefined behavior otherwise.
  template<class T>
  Status SetSegmentValue(std::string_view name, const T & new_value)
  {
    uint8_t * value = GetRawValue(name);
    if (value == nullptr) {
      std::stringstream msg;
      msg << "memory segment not found: " << name;
      return {
        .code = StatusCode::kNotFound,
        .message = msg.str(),
      };
    }
    *reinterpret_cast<T *>(value) = new_value;
    return {};
  }
  template<class T>
  Status SetSegmentValue(std::string_view name, T && new_value)
  {
    uint8_t * value = GetRawValue(name);
    if (value == nullptr) {
      std::stringstream msg;
      msg << "memory segment not found: " << name;
      return {
        .code = StatusCode::kNotFound,
        .message = msg.str(),
      };
    }
    *reinterpret_cast<T *>(value) = std::forward<T>(new_value);
    return {};
  }

  // Returns a pointer to the untyped payload in the shared memory segment.
  // Memory layout is described in
  // intrinsic/icon/interprocess/shared_memory_manager/segment_header.h
  // This function might be used when access to the underlying generic memory
  // location is needed, e.g. via `std::memcpy`. One typical use case is to copy
  // a flatbuffer (or any other serialized data struct) into a shared memory
  // segment. Prefer accessing the values via `GetSegmentValue` or
  // `SetSegmentValue` for type safety.
  uint8_t * GetRawValue(std::string_view name);

  // Returns a list of names for all registered shared memory segments.
  std::vector<std::string> GetRegisteredMemoryNames() const;

  // Returns a SegmentInfo struct containing the list of registered memory
  // segments.
  intrinsic_fbs::SegmentInfo GetSegmentInfo() const;

  // Name of the module owning this SharedMemoryManager.
  std::string ModuleName() const;

  // Namespace for the shared memory interfaces using this SharedMemoryManager.
  std::string SharedMemoryNamespace() const;

private:
  explicit SharedMemoryManager(
    std::string_view module_name,
    std::string_view shared_memory_namespace);

  // Creates an anonymous shared memory segment with the given name and the size
  // of SegmentHeader + `payload_size`.
  // The SegmentHeader is initialized with the given `type_id`.
  // Returns `InternalError` if any of the underlying
  // POSIX calls fail.
  Status InitSegment(
    std::string_view name, bool must_be_used,
    size_t payload_size, const std::string & type_id);

  // Returns a pointer to the start of the memory segment.
  // The SegmentHeader of this segment lives at the address this pointer
  // indicates.
  // Returns nullptr if the segment with the given name does not exist.
  uint8_t * GetRawSegment(std::string_view name);

  // Can be generated from memory_segments_, but it's more efficient to simply
  // update the map when a new segment is added.
  SegmentNameToFileDescriptorMap segment_name_to_file_descriptor_map_;

  // We not only store the name of the each initialized segment, but also a
  // pointer to its allocated memory. That way we can later on provide
  // introspection tools around all allocated memory in the system.
  std::unordered_map<std::string, MemorySegmentInfo> memory_segments_;
  std::string module_name_;
  std::string shared_memory_namespace_;
};
}
