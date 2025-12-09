// Copyright 2024 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "flowstate_ros_bridge/executive.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>  // Optional, if building pool dynamically
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/type_resolver_util.h>

#include <utility>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/notification.h"
#include "intrinsic/executive/proto/run_metadata.pb.h"
#include "intrinsic/skills/proto/skills.pb.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/proto/descriptors.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

namespace flowstate_ros_bridge {
using RunMetadata = intrinsic_proto::executive::RunMetadata;
using GetSkillRequest = intrinsic_proto::skills::GetSkillRequest;
using GetSkillResponse = intrinsic_proto::skills::GetSkillResponse;
using GetSkillsResponse = intrinsic_proto::skills::GetSkillsResponse;

static constexpr std::string PROCESS_PARAMS_KEY = "parameters";
static constexpr std::string RESOURCE_PARAMS_KEY = "resources";
static constexpr std::size_t kListBehaviorTreesPageSize = 50;
static constexpr std::size_t kListOperationsPageSize = 100;

///=============================================================================
Executive::Executive(const std::string& executive_service_address,
                     const std::string& skill_registry_address,
                     const std::string& solution_service_address,
                     std::size_t deadline_seconds,
                     std::size_t update_rate_millis)
    : executive_service_address_(std::move(executive_service_address)),
      skill_registry_address_(std::move(skill_registry_address)),
      solution_service_address_(std::move(solution_service_address)),
      deadline_seconds_(deadline_seconds),
      update_rate_millis_(update_rate_millis),
      connected_(false),
      current_process_(nullptr) {
  // Do nothing.
}

///=============================================================================
absl::Status Executive::connect() {
  if (connected_) {
    return absl::OkStatus();
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);

  grpc::ChannelArguments channel_args = intrinsic::DefaultGrpcChannelArgs();
  // The skill registry may need to call out to one or more skill information
  // services. Those services might not be ready at startup. We configure a
  // retry policy to mitigate b/283020857.
  // (See
  // https://github.com/grpc/grpc-go/blob/master/examples/features/retry/README.md
  //  for an example of this gRPC feature.)
  channel_args.SetServiceConfigJSON(R"(
      {
        "methodConfig": [{
          "name": [{"service": "intrinsic_proto.skills.SkillRegistry"}],
          "waitForReady": true,
          "timeout": "300s",
          "retryPolicy": {
              "maxAttempts": 10,
              "initialBackoff": "0.1s",
              "maxBackoff": "10s",
              "backoffMultiplier": 1.5,
              "retryableStatusCodes": [ "UNAVAILABLE" ]
          }
        }]
      })");
  channel_args.SetMaxReceiveMessageSize(10000000);  // 10 MB
  channel_args.SetMaxSendMessageSize(10000000);     // 10 MB

  // Connect to the executive service.
  LOG(INFO) << "Connecting to executive service at address "
            << executive_service_address_;
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> executive_channel,
      intrinsic::CreateClientChannel(
          executive_service_address_,
          absl::Now() + absl::Seconds(deadline_seconds_), channel_args));
  INTR_RETURN_IF_ERROR(intrinsic::WaitForChannelConnected(
      executive_service_address_, executive_channel,
      absl::Now() + absl::Seconds(deadline_seconds_)));
  executive_stub_ = ExecutiveService::NewStub(std::move(executive_channel));
  LOG(INFO) << "Successfully connected to executive service!";

  // Connect to the solution service.
  LOG(INFO) << "Connecting to solution service at address "
            << solution_service_address_;
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> solution_channel,
      intrinsic::CreateClientChannel(
          solution_service_address_,
          absl::Now() + absl::Seconds(deadline_seconds_), channel_args));
  INTR_RETURN_IF_ERROR(intrinsic::WaitForChannelConnected(
      solution_service_address_, solution_channel,
      absl::Now() + absl::Seconds(deadline_seconds_)));
  solution_stub_ = SolutionService::NewStub(std::move(solution_channel));
  LOG(INFO) << "Successfully connected to solution service!";

  // Connect to the skill registry.
  LOG(INFO) << "Connecting to skill registry at address "
            << skill_registry_address_;
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> skill_channel,
      intrinsic::CreateClientChannel(
          skill_registry_address_,
          absl::Now() + absl::Seconds(deadline_seconds_), channel_args));
  INTR_RETURN_IF_ERROR(intrinsic::WaitForChannelConnected(
      skill_registry_address_, skill_channel,
      absl::Now() + absl::Seconds(deadline_seconds_)));
  skill_registry_stub_ = SkillRegistry::NewStub(std::move(skill_channel));
  LOG(INFO) << "Successfully connected to the skill registry!";

  // TODO(Yadunund): Check if we can add hooks to reconnect if disconnected
  // during runtime.
  connected_ = true;

  // Clear all operations.
  clear_and_delete_operations();

  return absl::OkStatus();
}

namespace {
///=============================================================================
std::unique_ptr<grpc::ClientContext> make_client_context(
    std::size_t deadline_seconds) {
  auto client_context = std::make_unique<grpc::ClientContext>();
  client_context->set_deadline(std::chrono::system_clock::now() +
                               std::chrono::seconds(deadline_seconds));
  return client_context;
}

///=============================================================================
void cancel_operation(const std::string& operation_name,
                      std::shared_ptr<ExecutiveService::Stub> executive_stub,
                      size_t deadline_seconds) {
  auto client_context = make_client_context(deadline_seconds);
  google::longrunning::CancelOperationRequest cancel_request;
  cancel_request.set_name(operation_name);
  auto status = executive_stub->CancelOperation(
      client_context.get(), std::move(cancel_request), nullptr);
  if (!status.ok()) {
    LOG(INFO) << "Error while cancelling operation " << operation_name;
  } else {
    LOG(INFO) << "Successfully cancelled operation " << operation_name;
  }
}

///=============================================================================
void delete_operation(const std::string& operation_name,
                      std::shared_ptr<ExecutiveService::Stub> executive_stub,
                      size_t deadline_seconds) {
  auto client_context = make_client_context(deadline_seconds);
  google::longrunning::DeleteOperationRequest delete_request;
  delete_request.set_name(operation_name);
  auto status = executive_stub->DeleteOperation(
      client_context.get(), std::move(delete_request), nullptr);
  if (!status.ok()) {
    LOG(INFO) << "Error while deleting operation " << operation_name;
  } else {
    LOG(INFO) << "Successfully deleted operation " << operation_name;
  }
}

///=============================================================================
absl::Status add_to_descriptor_database(
    google::protobuf::SimpleDescriptorDatabase* db,
    const google::protobuf::FileDescriptorProto& file_descriptor,
    absl::flat_hash_map<std::string, google::protobuf::FileDescriptorProto>&
        file_by_name) {
  for (const std::string& dependency : file_descriptor.dependency()) {
    if (auto fd_iter = file_by_name.find(dependency);
        fd_iter != file_by_name.end()) {
      // add dependency now and remove from elements to visit
      google::protobuf::FileDescriptorProto dependency_file_descriptor =
          fd_iter->second;
      file_by_name.erase(fd_iter);
      INTR_RETURN_IF_ERROR(add_to_descriptor_database(
          db, dependency_file_descriptor, file_by_name));
    }
  }
  if (!db->Add(file_descriptor)) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Failed to add descriptor '%s' to descriptor database",
                        file_descriptor.name()));
  }
  return absl::OkStatus();
}

///=============================================================================
absl::Status populate_descriptor_database(
    google::protobuf::SimpleDescriptorDatabase* db,
    const google::protobuf::FileDescriptorSet& file_descriptor_set) {
  absl::flat_hash_map<std::string, google::protobuf::FileDescriptorProto>
      file_by_name;
  for (const google::protobuf::FileDescriptorProto& file_descriptor :
       file_descriptor_set.file()) {
    file_by_name[file_descriptor.name()] = file_descriptor;
  }
  while (!file_by_name.empty()) {
    google::protobuf::FileDescriptorProto file_descriptor =
        file_by_name.begin()->second;
    file_by_name.erase(file_by_name.begin());
    INTR_RETURN_IF_ERROR(
        add_to_descriptor_database(db, file_descriptor, file_by_name));
  }
  return absl::OkStatus();
}
}  // namespace

///=============================================================================
void Executive::clear_and_delete_operations() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!connected_) {
    return;
  }

  if (current_process_ != nullptr && !current_process_->done()) {
    LOG(INFO) << "Deleting operation created by bridge with name: "
              << current_process_->current_operation_.name();
    current_process_->cancel();
    current_process_ = nullptr;
  }
  // Now delete any other operations that may have been started.
  // First list active operations.
  auto client_context = make_client_context(this->deadline_seconds_);
  google::longrunning::ListOperationsRequest list_request;
  list_request.set_page_size(kListOperationsPageSize);
  do {
    google::longrunning::ListOperationsResponse list_response;
    const auto status = executive_stub_->ListOperations(
        client_context.get(), list_request, &list_response);

    if (!status.ok()) {
      LOG(INFO) << "Error while listing operations.";
      return;
    }
    for (const auto& operation : list_response.operations()) {
      // TODO(Yadunund): If we want this bridge to not override
      // all the operations started by users from the frontend, we might
      // want to attach active operations to this bridge and monitor its
      // state until completion before clearing.
      delete_operation(operation.name(), executive_stub_, deadline_seconds_);
    }
    list_request.set_page_token(list_response.next_page_token());
  } while (!list_request.page_token().empty());
}

///=============================================================================
auto Executive::behavior_tree(const std::string& name) const
    -> absl::StatusOr<BehaviorTree> {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!connected_) {
    return absl::InvalidArgumentError("Not connected to solution service.");
  }

  auto client_context = std::make_unique<grpc::ClientContext>();
  client_context->set_deadline(std::chrono::system_clock::now() +
                               std::chrono::seconds(deadline_seconds_));
  intrinsic_proto::solution::v1::GetBehaviorTreeRequest request;
  request.set_name(name);
  BehaviorTree response;
  INTR_RETURN_IF_ERROR(intrinsic::ToAbslStatus(solution_stub_->GetBehaviorTree(
      client_context.get(), request, &response)));

  return response;
}

///=============================================================================
auto Executive::behavior_trees() const
    -> absl::StatusOr<std::vector<BehaviorTree>> {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!connected_) {
    return absl::InvalidArgumentError("Not connected to solution service.");
  }

  intrinsic_proto::solution::v1::ListBehaviorTreesRequest request;
  request.set_page_size(kListBehaviorTreesPageSize);
  std::vector<BehaviorTree> bts = {};
  do {
    auto client_context = std::make_unique<grpc::ClientContext>();
    client_context->set_deadline(std::chrono::system_clock::now() +
                                 std::chrono::seconds(deadline_seconds_));
    
    intrinsic_proto::solution::v1::ListBehaviorTreesResponse response;
    INTR_RETURN_IF_ERROR(
        intrinsic::ToAbslStatus(solution_stub_->ListBehaviorTrees(
            client_context.get(), request, &response)));

    for (auto& bt : *response.mutable_behavior_trees()) {
      bts.emplace_back(std::move(bt));
    }
    request.set_page_token(response.next_page_token());
  } while (!request.page_token().empty());
  return bts;
}

///=============================================================================
Executive::ProcessHandle::ProcessHandle(
    std::shared_ptr<ExecutiveService::Stub> executive_stub,
    std::size_t deadline_seconds, google::longrunning::Operation operation,
    ProcessFeedbackCallback feedback_cb, ProcessCompletedCallback completed_cb)
    : executive_stub_(std::move(executive_stub)),
      deadline_seconds_(std::move(deadline_seconds)),
      current_operation_(std::move(operation)),
      feedback_cb_(std::move(feedback_cb)),
      completed_cb_(std::move(completed_cb)) {
  cancelled_ = std::make_shared<bool>(false);
}

///=============================================================================
auto Executive::ProcessHandle::make(
    std::shared_ptr<ExecutiveService::Stub> executive_stub,
    std::size_t deadline_seconds, google::longrunning::Operation operation,
    ProcessFeedbackCallback feedback_cb, ProcessCompletedCallback completed_cb,
    std::size_t update_interval_millis) -> ProcessHandlePtr {
  auto handle = std::shared_ptr<ProcessHandle>(new ProcessHandle(
      std::move(executive_stub), std::move(deadline_seconds),
      std::move(operation), std::move(feedback_cb), std::move(completed_cb)));

  // Spawn a thread to monitor lifecycle of the process.
  handle->update_thread_ =
      std::thread(
          [handle](const std::size_t update_interval_millis) -> void {
            while (true) {
              std::lock_guard<std::mutex> lock(handle->mutex_);
              if (*handle->cancelled_) {
                // We assume the operation has been cancelled and reset.
                return;
              }
              // Get the latest state of the current operation.
              auto client_context =
                  make_client_context(handle->deadline_seconds_);
              google::longrunning::GetOperationRequest get_request;
              get_request.set_name(handle->current_operation_.name());
              auto status = handle->executive_stub_->GetOperation(
                  client_context.get(), std::move(get_request),
                  &handle->current_operation_);
              if (!status.ok()) {
                std::stringstream ss;
                ss << "[process thread] Error while getting operation "
                   << handle->current_operation_.name();
                LOG(INFO) << ss.str();
                handle->feedback_cb_(false, absl::UnavailableError(ss.str()));
              }

              handle->feedback_cb_(handle->current_operation_.done(),
                                   intrinsic::ToAbslStatus(status));

              if (handle->current_operation_.done()) {
                // The operation may be done because it succeeded or errored
                // out.
                if (handle->current_operation_.has_error()) {
                  const auto& error = handle->current_operation_.error();
                  LOG(INFO) << "[process thread] Current operation "
                            << handle->current_operation_.name()
                            << "has failed due to error: " << error.message();
                  handle->completed_cb_(false, error.message());
                  return;
                } else if (handle->current_operation_.has_response()) {
                  const auto& response = handle->current_operation_.response();
                  LOG(INFO) << "[process thread] Current operation "
                            << handle->current_operation_.name()
                            << "completed successfully.";
                  handle->completed_cb_(true, response.DebugString());
                  return;
                } else {
                  // This should never happen.
                  LOG(INFO)
                      << "[process thread] Current operation completed without "
                         "error or response. This should never happen.";
                  handle->completed_cb_(false, "failure");
                  return;
                }
              } else {
                // The operation is still running so we sleep for a bit before
                // polling again.
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(update_interval_millis));
              }
            }
          },
          update_interval_millis);
  return handle;
}

///=============================================================================
Executive::ProcessHandle::~ProcessHandle() {
  *cancelled_ = true;
  if (update_thread_.joinable()) {
    update_thread_.join();
  }
}

///=============================================================================
void Executive::ProcessHandle::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  cancel_operation(current_operation_.name(), executive_stub_,
                   deadline_seconds_);
  delete_operation(current_operation_.name(), executive_stub_,
                   deadline_seconds_);
  *cancelled_ = true;
}

///=============================================================================
bool Executive::ProcessHandle::done() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_operation_.done();
}

///=============================================================================
absl::StatusOr<Executive::ProcessHandlePtr> Executive::start(
    const BehaviorTree& bt, const ExecutionMode& execution_mode,
    const SimulationMode& simulation_mode, const nlohmann::json& process_params,
    Executive::ProcessFeedbackCallback feedback_cb,
    Executive::ProcessCompletedCallback completed_cb,
    const std::optional<std::string> scene_id) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!connected_) {
    return absl::InvalidArgumentError("Not connected to executive service.");
  }
  clear_and_delete_operations();
  google::longrunning::Operation current_operation;
  // To start a process, we create an operation first and then start
  // the operation.
  auto create_client_context = std::make_unique<grpc::ClientContext>();
  create_client_context->set_deadline(std::chrono::system_clock::now() +
                                      std::chrono::seconds(deadline_seconds_));
  // Create operation.
  intrinsic_proto::executive::CreateOperationRequest create_request;
  *create_request.mutable_behavior_tree() = bt;
  INTR_RETURN_IF_ERROR(intrinsic::ToAbslStatus(executive_stub_->CreateOperation(
      create_client_context.get(), std::move(create_request),
      &current_operation)));
  LOG(INFO) << "Successfully created a operation with name: "
            << current_operation.name();

  // Create a StratOperationRequest.
  intrinsic_proto::executive::StartOperationRequest start_request;
  auto start_client_context = std::make_unique<grpc::ClientContext>();
  start_client_context->set_deadline(std::chrono::system_clock::now() +
                                     std::chrono::seconds(deadline_seconds_));

  // If process_params is provided, we need to appropriate fill the parameters
  // field in the StartOperationRequest.
  // The assumption here is that the process to be started has parameters
  // defined by it's root node.
  const auto parameterize_status = this->parameterize_start_request(
      current_operation, process_params, start_request);
  if (parameterize_status.ok()) {
    LOG(INFO) << "Process was successfully parameterized with "
              << process_params.dump();
  } else {
    LOG(INFO) << "Process was not parameterized with " << process_params.dump();
  }

  start_request.set_name(current_operation.name());
  start_request.set_execution_mode(execution_mode);
  start_request.set_simulation_mode(simulation_mode);
  if (scene_id.has_value()) {
    start_request.set_scene_id(scene_id.value());
  }
  INTR_RETURN_IF_ERROR(intrinsic::ToAbslStatus(executive_stub_->StartOperation(
      start_client_context.get(), std::move(start_request),
      &current_operation)));

  current_process_ =
      ProcessHandle::make(executive_stub_, deadline_seconds_,
                          std::move(current_operation), std::move(feedback_cb),
                          std::move(completed_cb), this->update_rate_millis_);

  return current_process_;
}

///=============================================================================
absl::Status Executive::parameterize_start_request(
    const google::longrunning::Operation& current_operation,
    const nlohmann::json& start_params,
    intrinsic_proto::executive::StartOperationRequest& start_request) {
  if (start_params.is_null() || start_params.empty()) {
    return absl::OkStatus();
  }

  // Populate resources if present.
  if (start_params.contains(RESOURCE_PARAMS_KEY)) {
    // Iterate over nested JSON.
    if (start_params[RESOURCE_PARAMS_KEY].is_structured()) {
      google::protobuf::Map<std::string, std::string>* resources_map =
          start_request.mutable_resources();
      for (const auto& resource : start_params[RESOURCE_PARAMS_KEY].items()) {
        const std::string& key = resource.key();
        const nlohmann::json& val = resource.value();
        std::string value_as_string;
        // Validate and convert the value to a string.
        if (val.is_string()) {
          // If it's already a string, get its value directly.
          value_as_string = val.get<std::string>();
        } else {
          // For non-string types (number, boolean, null), dump() will convert
          // them to a string representation (e.g., 3 -> "3", true -> "true").
          value_as_string = val.dump();
        }
        // Insert the key and the converted value into the proto map.
        (*resources_map)[key] = value_as_string;
      }
    }
  }

  // Now populate process params.
  if (!start_params.contains(PROCESS_PARAMS_KEY)) {
    return absl::OkStatus();
  }
  if (!start_params[PROCESS_PARAMS_KEY].is_structured()) {
    LOG(INFO) << PROCESS_PARAMS_KEY
              << " specified in start parameters but data is not structured as "
                 "a JSON."
              << " Skipping filling in the Any proto.";
    return absl::OkStatus();
  }

  const nlohmann::json& process_params = start_params[PROCESS_PARAMS_KEY];
  RunMetadata run_metadata;
  if (!current_operation.metadata().UnpackTo(&run_metadata)) {
    return absl::InternalError(
        "Failed to unpack RunMetadata from operation metadata.");
  }

  // Get FileDescriptorSet and parameter type name directly from RunMetadata's
  // BehaviorTreeDescription.
  const auto& bt_description = run_metadata.behavior_tree().description();
  const auto& parameter_desc = bt_description.parameter_description();
  const auto& file_descriptor_set =
      parameter_desc.parameter_descriptor_fileset();
  const std::string& parameter_type_name =
      parameter_desc.parameter_message_full_name();

  LOG(INFO) << "Parameterizing process using FileDescriptorSet from "
               "RunMetadata. Parameter message full name: "
            << parameter_type_name;
  // Uncomment for debugging.
  // LOG(INFO) << "FileDescriptorSet: " << file_descriptor_set.DebugString();

  // Create Descriptor Pool using SimpleDescriptorDatabase, similar to
  // ProtobufManager.
  auto db = std::make_unique<google::protobuf::SimpleDescriptorDatabase>();

  absl::Status populate_status =
      populate_descriptor_database(db.get(), file_descriptor_set);
  if (!populate_status.ok()) {
    LOG(ERROR) << "Failed to populate descriptor database: " << populate_status;
    return absl::InternalError(absl::StrCat(
        "Failed to populate descriptor database from FileDescriptorSet: ",
        populate_status.ToString()));
  }

  auto descriptor_pool =
      std::make_unique<google::protobuf::DescriptorPool>(db.get());

  // 3. Prepare JSON string with @type for parsing into Any
  // TODO(Yadunund): Avoid copy.
  nlohmann::json parameter_json_with_type = process_params;
  const std::string url_prefix = "type.googleapis.com";
  const std::string& fully_qualified_type =
      absl::StrCat(url_prefix, "/", parameter_type_name);
  // 4. Prepare TypeResolver.
  std::unique_ptr<google::protobuf::util::TypeResolver> resolver(
      google::protobuf::util::NewTypeResolverForDescriptorPool(
          url_prefix, descriptor_pool.get()));
  if (!resolver) {
    LOG(INFO) << "Failed to create type resolver";
    return absl::InternalError("Failed to create TypeResolver.");
  }
  // 5. Convert JSON to Binary Stream
  std::string input_json_string;
  std::string binary_output_string;
  try {
    // Dump the input JSON (which should *not* contain @type).
    input_json_string = process_params.dump();
  } catch (const nlohmann::json::exception& e) {
    LOG(ERROR) << "Failed to dump input JSON: " << e.what();
    return absl::InvalidArgumentError("Input parameters are not valid JSON.");
  }
  google::protobuf::io::ArrayInputStream json_input_stream(
      input_json_string.data(), input_json_string.size());
  google::protobuf::io::StringOutputStream binary_output_stream(
      &binary_output_string);
  // Convert the JSON to binary, interpreting it according to
  // fully_qualified_type The resolver uses the pool to find the descriptor for
  // fully_qualified_type.
  absl::Status status_j2b = google::protobuf::util::JsonToBinaryStream(
      resolver.get(), fully_qualified_type, &json_input_stream,
      &binary_output_stream);
  if (!status_j2b.ok()) {
    LOG(ERROR) << "Error converting JSON to binary: " << status_j2b.ToString();
    return absl::InternalError("Error converting JSON to binary.");
  }
  google::protobuf::Any parameter_any_proto;

  // Set the type_url and value fields of the Any proto.
  parameter_any_proto.set_type_url(fully_qualified_type);
  parameter_any_proto.set_value(binary_output_string);
  *start_request.mutable_parameters() = parameter_any_proto;
  return absl::OkStatus();
}

}  // namespace flowstate_ros_bridge.
