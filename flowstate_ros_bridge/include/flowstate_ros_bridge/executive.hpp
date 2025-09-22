// Copyright 2024 Intrinsic Innovation LLC
//
// You are hereby granted a non-exclusive, worldwide, royalty-free license to
// use, copy, modify, and distribute this Intrinsic SDK in source code or binary
// form for use in connection with the services and APIs provided by Intrinsic
// Innovation LLC (“Intrinsic”).
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// If you use this Intrinsic SDK with any Intrinsic services, your use is
// subject to the Intrinsi Platform Terms of Service
// [https://intrinsic.ai/platform-terms].  If you create works that call
// Intrinsic APIs, you must agree to the terms of service for those APIs
// separately. This license does not grant you any special rights to use the
// services.
//
// This copyright notice shall be included in all copies or substantial portions
// of the software.

#ifndef FLOWSTATE_ROS_BRIDGE__EXECUTIVE_HPP_
#define FLOWSTATE_ROS_BRIDGE__EXECUTIVE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "absl/status/statusor.h"
#include "intrinsic/executive/proto/behavior_tree.pb.h"
#include "intrinsic/executive/proto/executive_execution_mode.pb.h"
#include "intrinsic/executive/proto/executive_service.grpc.pb.h"
#include "intrinsic/executive/proto/executive_service.pb.h"
#include "intrinsic/frontend/solution_service/proto/solution_service.grpc.pb.h"
#include "intrinsic/frontend/solution_service/proto/solution_service.pb.h"
#include "intrinsic/skills/proto/skill_registry.grpc.pb.h"
#include "intrinsic/skills/proto/skill_registry.pb.h"
#include "intrinsic/util/grpc/grpc.h"
#include "nlohmann/json.hpp"

namespace flowstate_ros_bridge {
using BehaviorTree = intrinsic_proto::executive::BehaviorTree;
using ExecutionMode = intrinsic_proto::executive::ExecutionMode;
using SimulationMode = intrinsic_proto::executive::SimulationMode;
using ExecutiveService = intrinsic_proto::executive::ExecutiveService;
using SkillRegistry = intrinsic_proto::skills::SkillRegistry;
using SolutionService = intrinsic_proto::solution::v1::SolutionService;
///=============================================================================
/// A class that establishes bi-directional communication with
/// Flowstate workcells.
class Executive : public std::enable_shared_from_this<Executive> {
 public:
  /**
   * @brief Constructs a new Executive object.
   *
   * Initializes the Executive component by configuring the network addresses
   * for its required dependent services and setting default timing parameters
   * for its operations.
   *
   * @param executive_service_address The network address (e.g., "host:port" or
   * URI) of the main Executive Service to connect to.
   * @param skill_registry_address The network address (e.g., "host:port" or
   * URI) of the Skill Registry Service.
   * @param solution_service_address The network address (e.g., "host:port" or
   * URI) of the Solution Service.
   * @param deadline_seconds The default timeout duration in seconds for
   * operations initiated by this Executive. Defaults to 5 seconds.
   * @param update_rate_millis The frequency in milliseconds at which the
   * Executive performs periodic tasks (e.g., status checks, updates). Defaults
   * to 1000 milliseconds (1 second).
   *
   * @note The validity or reachability of the provided addresses is typically
   * checked upon attempting connection, not necessarily within this
   * constructor.
   */
  Executive(const std::string &executive_service_address,
            const std::string &skill_registry_address,
            const std::string &solution_service_address,
            std::size_t deadline_seconds = 5,
            std::size_t update_rate_millis = 1000);

  // Establish connections with various services.
  absl::Status connect();

  // Get a behavior tree by specifying its name.
  absl::StatusOr<BehaviorTree> behavior_tree(const std::string &name) const;

  // Get a list of behavior trees in the solution.
  absl::StatusOr<std::vector<BehaviorTree>> behavior_trees() const;

  using ProcessCompletedCallback =
      std::function<void(const bool, const std::string &)>;
  using ProcessCancelCallback = std::function<void()>;

  class ProcessHandle;
  using ProcessHandlePtr = std::shared_ptr<ProcessHandle>;
  class ProcessHandle {
   public:
    friend class Executive;

    static ProcessHandlePtr make(
        std::shared_ptr<ExecutiveService::Stub> executive_stub,
        std::size_t deadline_seconds, google::longrunning::Operation operation,
        ProcessCompletedCallback completed_cb,
        std::size_t update_interval_millis);

    void cancel();

    bool done() const;

    ~ProcessHandle();

   private:
    ProcessHandle(std::shared_ptr<ExecutiveService::Stub> executive_stub,
                  std::size_t deadline_seconds,
                  google::longrunning::Operation operation,
                  ProcessCompletedCallback completed_cb);

    mutable std::mutex mutex_;
    std::shared_ptr<ExecutiveService::Stub> executive_stub_;
    size_t deadline_seconds_;
    google::longrunning::Operation current_operation_;
    ProcessCompletedCallback completed_cb_;
    std::thread update_thread_;
    std::shared_ptr<bool> cancelled_;
  };
  // Start a process.
  absl::StatusOr<ProcessHandlePtr> start(const BehaviorTree &bt,
                                         const ExecutionMode &execution_mode,
                                         const SimulationMode &simulation_mode,
                                         const nlohmann::json &process_params,
                                         ProcessCompletedCallback completed_cb);

 private:
  mutable std::recursive_mutex mutex_;
  std::string executive_service_address_;
  std::string skill_registry_address_;
  std::string solution_service_address_;
  std::size_t deadline_seconds_;
  std::size_t update_rate_millis_;
  bool connected_;
  std::shared_ptr<ExecutiveService::Stub> executive_stub_;
  std::unique_ptr<SkillRegistry::Stub> skill_registry_stub_;
  std::unique_ptr<SolutionService::Stub> solution_stub_;
  std::shared_ptr<ProcessHandle> current_process_;

  // Delete any operations started by this bridge and those already running.
  void clear_and_delete_operations();

  /**
   * @brief Populates the 'parameters' field (google.protobuf.Any) in a
   * StartOperationRequest.
   *
   * Derives parameter type information dynamically based on metadata from the
   * provided `current_operation`, converts the input `process_params` JSON
   * according to that type information, and sets the resulting binary data
   * and type URL into the `parameters` field of the `start_request`.
   *
   * This function encapsulates the workflow for handling dynamic parameters:
   * 1. Extracts necessary identifiers (e.g., process/skill ID) from the
   * `current_operation.metadata()`.
   * 2. Fetches detailed parameter information (the expected parameter message's
   * type URL and its defining `FileDescriptorSet`) from a registry (e.g., Skill
   * Registry) using the extracted identifier.
   * 3. Builds a temporary `google::protobuf::DescriptorPool` populated with the
   * fetched `FileDescriptorSet`.
   * 4. Converts the provided `process_params` JSON object (which should contain
   * only the fields matching the fetched parameter type) into the standard
   * Protobuf binary format using `google::protobuf::util::JsonToBinaryStream`.
   * A `TypeResolver` backed by the temporary descriptor pool is used to
   * interpret the JSON based on the fetched type URL.
   * 5. Creates a `google::protobuf::Any` proto.
   * 6. Sets the `type_url` of the `Any` proto to the fetched parameter type
   * URL.
   * 7. Sets the `value` of the `Any` proto to the binary data generated in
   * step 4.
   * 8. Assigns the fully populated `Any` proto to the `parameters` field of the
   * input/output `start_request` object.
   *
   * If the input `process_params` is null or empty, this function may return
   * successfully without modifying the `start_request.parameters` field,
   * depending on the desired behavior (currently assumes the calling `start`
   * function checks for null `process_params` before calling this).
   *
   * @param current_operation [in] The `google::longrunning::Operation` object,
   * typically obtained from a preceding `CreateOperation` call. Its metadata is
   * essential for deriving the context needed to look up parameter definitions.
   * Assumes the required metadata (e.g., containing `RunMetadata`) is present
   * and correctly packed.
   * @param process_params [in] A `nlohmann::json` object containing the
   * key-value pairs representing the parameter fields for the operation being
   * started. This JSON object should *not* contain the '@type' field itself, as
   * the type is determined dynamically.
   * @param start_request [in, out] The
   * `intrinsic_proto::executive::StartOperationRequest` proto message to be
   * parameterized. This function will modify this object by setting its
   * `parameters` field (which must be of type `google.protobuf.Any`).
   *
   * @return absl::OkStatus() on successful parameterization and setting the
   * field in `start_request`.
   * @return absl::InvalidArgumentError if input `process_params` fails JSON
   * dumping.
   * @return absl::FailedPreconditionError if required information cannot be
   * extracted (e.g., process ID from metadata, parameter type name from skill
   * description).
   * @return absl::InternalError if an internal step fails (e.g., unpacking
   * metadata, creating TypeResolver, building DescriptorPool, JSON-to-binary
   * conversion).
   * @return Other specific absl::Status codes originating from underlying gRPC
   * calls (e.g., to the Skill Registry) or Protobuf operations.
   *
   * @note This function builds a temporary `DescriptorPool` on each call where
   * parameters are present. If performance becomes an issue, consider caching
   * descriptors based on the process/skill ID.
   * @note This implementation uses direct population of the `Any` proto
   * (setting type_url and value) after JSON->binary conversion, avoiding an
   * intermediate `DynamicMessage` step for efficiency in this context.
   */
  absl::Status parameterize_start_request(
      const google::longrunning::Operation &current_operation,
      const nlohmann::json &start_params,
      intrinsic_proto::executive::StartOperationRequest &start_request);
};
}  // namespace flowstate_ros_bridge.

#endif  // FLOWSTATE_ROS_BRIDGE__EXECUTIVE_HPP_
