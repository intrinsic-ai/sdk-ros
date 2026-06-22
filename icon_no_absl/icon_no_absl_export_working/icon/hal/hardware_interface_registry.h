#ifndef ICON_HAL_HARDWARE_INTERFACE_REGISTRY_H_
#define ICON_HAL_HARDWARE_INTERFACE_REGISTRY_H_

#include <cstddef>
#include <string>
#include <string_view>
#include <tl/expected.hpp>

#include "flatbuffers/detached_buffer.h"
#include "icon/hal/get_hardware_interface.h"
#include "icon/hal/hardware_interface_handle.h"
#include "icon/hal/hardware_interface_traits.h"
#include "icon/interprocess/shared_memory_manager/shared_memory_manager.h"
#include "icon/utils/attributes.h"
#include "icon/utils/log.h"
#include "icon/utils/realtime_guard.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_macros.h"

namespace intrinsic::icon {

// Hardware Modules use the HardwareInterfaceRegistry to advertise their
// interfaces.
//
// See the documentation on the various Advertise functions for details, and
// note in particular the distinction between strict and non-strict interfaces.
class HardwareInterfaceRegistry {
 public:
  HardwareInterfaceRegistry() = delete;
  ~HardwareInterfaceRegistry() = default;

  // This class is move-only.
  HardwareInterfaceRegistry(const HardwareInterfaceRegistry& other) = delete;
  HardwareInterfaceRegistry& operator=(const HardwareInterfaceRegistry& other) =
      delete;
  HardwareInterfaceRegistry(HardwareInterfaceRegistry&& other) = default;
  HardwareInterfaceRegistry& operator=(HardwareInterfaceRegistry&& other) =
      default;

  // Constructs a new registry for hardware interfaces.
  // `shared_memory_manager` must outlive the return value of this function.
  explicit HardwareInterfaceRegistry(
      SharedMemoryManager& shared_memory_manager INTR_ATTRIBUTE_LIFETIME_BOUND);

  // Advertises a new hardware interface given an interface type and a set of
  // arguments used to initialize this interface.
  // The arguments are according to the respective `Builder` function as
  // specified via a call to the `INTRINSIC_ADD_HARDWARE_INTERFACE` macro, c.f.
  // `intrinsic/icon/hal/hardware_interface_traits.h`.
  // The interface is allocated within a shared memory segment.
  // Returns a non-mutable handle to the newly allocated hardware interface.
  // Prefer AdvertiseStrictInterface.
  template <class HardwareInterfaceT, typename... ArgsT>
  tl::expected<HardwareInterfaceHandle<HardwareInterfaceT>, Status>
  AdvertiseInterface(std::string_view interface_name, const log::Logger* logger,
                     ArgsT... args) {
    INTR_RETURN_UNEXPECTED_IF_ERROR(AdvertiseInterfaceT<HardwareInterfaceT>(
        interface_name, /*must_be_used=*/false, args...));
    return ::intrinsic::icon::GetInterfaceHandle<HardwareInterfaceT>(
        *shm_manager_, interface_name, logger);
  }

  // Advertises a new mutable hardware interface.
  // This functions behaves exactly like `AdvertiseInterface` except that it
  // returns a handle to mutable interface.
  // Prefer AdvertiseStrictInterface.
  template <class HardwareInterfaceT, typename... ArgsT>
  tl::expected<MutableHardwareInterfaceHandle<HardwareInterfaceT>, Status>
  AdvertiseMutableInterface(std::string_view interface_name,
                            const log::Logger* logger, ArgsT... args) {
    INTR_RETURN_UNEXPECTED_IF_ERROR(AdvertiseInterfaceT<HardwareInterfaceT>(
        interface_name, /*must_be_used=*/false, args...));
    return ::intrinsic::icon::GetMutableInterfaceHandle<HardwareInterfaceT>(
        *shm_manager_, interface_name, logger);
  }

  // Prefer AdvertiseMutableStrictInterface.
  template <class HardwareInterfaceT>
  tl::expected<MutableHardwareInterfaceHandle<HardwareInterfaceT>, Status>
  AdvertiseMutableInterface(std::string_view interface_name,
                            flatbuffers::DetachedBuffer&& message_buffer) {
    auto type_id =
        hardware_interface_traits::TypeID<HardwareInterfaceT>::kTypeString;
    INTR_RETURN_UNEXPECTED_IF_ERROR(AdvertiseInterfaceT(
        interface_name, /*must_be_used=*/false, message_buffer, type_id));
    return ::intrinsic::icon::GetMutableInterfaceHandle<HardwareInterfaceT>(
        *shm_manager_, interface_name);
  }

  // Advertises a new strict hardware interface using the interface type and a
  // set of arguments used to initialize this interface.
  //
  // A strict interface checks that it was updated in the same cycle the special
  // IconState interface reports as the current cycle when reading its data.
  //
  // Marks the hardware interface as required, which tells ICON that the
  // interface needs to be used in the ICON configuration.
  //
  // The arguments are according to the respective `Builder` function as
  // specified via a call to the `INTRINSIC_ADD_HARDWARE_INTERFACE` macro, c.f.
  // `intrinsic/icon/hal/hardware_interface_traits.h`.
  // The interface is allocated within a shared memory segment.
  // Returns a non-mutable handle to the newly allocated hardware interface.
  template <class HardwareInterfaceT, typename... ArgsT>
  tl::expected<StrictHardwareInterfaceHandle<HardwareInterfaceT>, Status>
  AdvertiseStrictInterface(std::string_view interface_name,
                           const log::Logger* logger, ArgsT... args) {
    INTR_RETURN_UNEXPECTED_IF_ERROR(AdvertiseInterfaceT<HardwareInterfaceT>(
        interface_name, /*must_be_used=*/true, args...));

    return ::intrinsic::icon::GetStrictInterfaceHandle<HardwareInterfaceT>(
        *shm_manager_, interface_name, logger);
  }

  // Advertises a new mutable strict hardware interface using the interface type
  // and a set of arguments used to initialize this interface.
  //
  // A strict interface checks that it was updated in the same cycle the special
  // IconState interface reports as the current cycle when reading its data.
  //
  // Marks the hardware interface as required, which tells ICON that the
  // interface needs to be used in the ICON configuration.
  //
  // The arguments are according to the respective `Builder` function as
  // specified via a call to the `INTRINSIC_ADD_HARDWARE_INTERFACE` macro, c.f.
  // `intrinsic/icon/hal/hardware_interface_traits.h`.
  // The interface is allocated within a shared memory segment.
  // Returns a mutable handle to the newly allocated hardware interface.
  template <class HardwareInterfaceT, typename... ArgsT>
  tl::expected<MutableStrictHardwareInterfaceHandle<HardwareInterfaceT>, Status>
  AdvertiseMutableStrictInterface(std::string_view interface_name,
                                  const log::Logger* logger, ArgsT... args) {
    INTR_RETURN_UNEXPECTED_IF_ERROR(AdvertiseInterfaceT<HardwareInterfaceT>(
        interface_name, /*must_be_used=*/true, args...));
    return ::intrinsic::icon::GetMutableStrictInterfaceHandle<
        HardwareInterfaceT>(*shm_manager_, interface_name, logger);
  }

  template <class HardwareInterfaceT>
  tl::expected<MutableStrictHardwareInterfaceHandle<HardwareInterfaceT>, Status>
  AdvertiseMutableStrictInterface(
      std::string_view interface_name, const log::Logger* logger,
      flatbuffers::DetachedBuffer&& message_buffer) {
    auto type_id =
        hardware_interface_traits::TypeID<HardwareInterfaceT>::kTypeString;
    INTR_RETURN_UNEXPECTED_IF_ERROR(AdvertiseInterfaceT(
        interface_name, /*must_be_used=*/true, message_buffer, type_id));
    return ::intrinsic::icon::GetMutableStrictInterfaceHandle<
        HardwareInterfaceT>(*shm_manager_, interface_name, logger);
  }

  // Convenience function that returns a handle to a registered interface.
  template <class HardwareInterfaceT>
  tl::expected<HardwareInterfaceHandle<HardwareInterfaceT>, Status>
  GetInterfaceHandle(std::string_view interface_name,
                     const log::Logger* logger) const {
    return ::intrinsic::icon::GetInterfaceHandle<HardwareInterfaceT>(
        *shm_manager_, interface_name, logger);
  }

  // Convenience function that returns a mutable handle to a registered
  // interface.
  template <class HardwareInterfaceT>
  tl::expected<MutableHardwareInterfaceHandle<HardwareInterfaceT>, Status>
  GetMutableInterfaceHandle(std::string_view interface_name,
                            const log::Logger* logger) const {
    return ::intrinsic::icon::GetMutableInterfaceHandle<HardwareInterfaceT>(
        *shm_manager_, interface_name, logger);
  }

  // Returns the number of registered interfaces.
  size_t Size() const {
    return shm_manager_->GetRegisteredMemoryNames().size();
  }

  // Name of the module owning this registry.
  std::string ModuleName() const INTRINSIC_NON_REALTIME_ONLY {
    return shm_manager_->ModuleName();
  }

  // Namespace for the shared memory interfaces using this registry.
  std::string SharedMemoryNamespace() const INTRINSIC_NON_REALTIME_ONLY {
    return shm_manager_->SharedMemoryNamespace();
  }

 private:
  // The parameter `must_be_used` tells ICON that this interface needs to be
  // used in its configuration.
  template <class HardwareInterfaceT, typename... ArgsT>
  Status AdvertiseInterfaceT(std::string_view interface_name, bool must_be_used,
                             ArgsT... args) {
    static_assert(
        hardware_interface_traits::BuilderFunctions<HardwareInterfaceT>::value,
        "No builder function defined.");
    flatbuffers::DetachedBuffer buffer =
        hardware_interface_traits::BuilderFunctions<HardwareInterfaceT>::kBuild(
            args...);
    auto type_id =
        hardware_interface_traits::TypeID<HardwareInterfaceT>::kTypeString;
    return AdvertiseInterfaceT(interface_name, must_be_used, buffer, type_id);
  }

  // The parameter `must_be_used` tells ICON that this interface needs to be
  // used in its configuration.
  Status AdvertiseInterfaceT(std::string_view interface_name, bool must_be_used,
                             const flatbuffers::DetachedBuffer& buffer,
                             std::string_view type_id);

  SharedMemoryManager* shm_manager_;
};

}  // namespace intrinsic::icon
#endif  // ICON_HAL_HARDWARE_INTERFACE_REGISTRY_H_
