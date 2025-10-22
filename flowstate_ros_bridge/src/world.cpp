// Copyright 2025 Intrinsic Innovation LLC
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

#include "flowstate_ros_bridge/world.hpp"

#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "intrinsic/geometry/proto/geometry_service.grpc.pb.h"
#include "intrinsic/util/grpc/grpc.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/proto/object_world_service.grpc.pb.h"

namespace flowstate_ros_bridge {

std::unique_ptr<ChannelFactory>
default_channel_factory(std::size_t deadline_seconds) {
  grpc::ChannelArguments channel_args = intrinsic::DefaultGrpcChannelArgs();
  // We might eventually need a retry policy here, like in executive (?)
  // Some of the meshes that we'll receive in the geometry client are large,
  // like a few 10's of MB.
  channel_args.SetMaxReceiveMessageSize(-1);    // no limit
  channel_args.SetMaxSendMessageSize(10000000); // 10 MB

  return std::make_unique<ClientChannelFactory>(
    absl::Seconds(deadline_seconds),std::move(channel_args));
}

World::World(std::shared_ptr<intrinsic::PubSub> pubsub,
             const std::string &world_service_address,
             const std::string &geometry_service_address,
             std::size_t deadline_seconds,
             std::optional<std::unique_ptr<ChannelFactory>> channel_factory)
    : pubsub_(pubsub), world_service_address_(world_service_address),
      geometry_service_address_(geometry_service_address),
      deadline_seconds_(deadline_seconds),
      channel_factory_(channel_factory.has_value()
        ? std::move(channel_factory).value()
        : default_channel_factory(deadline_seconds)) {
  // Do nothing. Connections will happen later.
}

absl::StatusOr<std::shared_ptr<intrinsic::Subscription>>
World::CreateTfSubscription(
    intrinsic::SubscriptionOkCallback<intrinsic_proto::TFMessage> callback) {
  auto sub =
      pubsub_->CreateSubscription("tf", intrinsic::TopicConfig(), callback);
  if (!sub.ok()) {
    return sub.status();
  }
  return std::make_shared<intrinsic::Subscription>(std::move(*sub));
}

absl::Status World::connect() {
  if (connected_) {
    return absl::OkStatus();
  }

  LOG(INFO) << "Connecting to world service at " << world_service_address_;
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> world_channel,
      channel_factory_->make_channel(world_service_address_));
  INTR_RETURN_IF_ERROR(intrinsic::WaitForChannelConnected(
      world_service_address_, world_channel,
      absl::Now() + absl::Seconds(deadline_seconds_)));
  std::shared_ptr<ObjectWorldService::StubInterface> object_world_stub =
      ObjectWorldService::NewStub(std::move(world_channel));
  object_world_client_ = std::make_shared<intrinsic::world::ObjectWorldClient>(
      "world", std::move(object_world_stub));
  LOG(INFO) << "Successfully connected to world service!";

  LOG(INFO) << "Connecting to geometry service at "
            << geometry_service_address_;
  INTR_ASSIGN_OR_RETURN(
      std::shared_ptr<grpc::Channel> geometry_channel,
      channel_factory_->make_channel(geometry_service_address_));
  INTR_RETURN_IF_ERROR(intrinsic::WaitForChannelConnected(
      geometry_service_address_, geometry_channel,
      absl::Now() + absl::Seconds(deadline_seconds_)));
  geometry_stub_ = GeometryService::NewStub(std::move(geometry_channel));
  LOG(INFO) << "Successfully connected to geometry service!";

  connected_ = true;

  return absl::OkStatus();
}

absl::StatusOr<std::vector<intrinsic::world::WorldObject>> World::GetObjects(
    std::optional<std::vector<std::string>> object_names) {
  std::vector<intrinsic::world::WorldObject> objects;
  if (!connected_ || !object_world_client_) {
    return absl::InternalError("Not connected to world service");
  }
  if (!object_names.has_value()) {
    INTR_ASSIGN_OR_RETURN(objects, object_world_client_->ListObjects());
  } else {
    // Create hashmap of SceneObject names used to check
    // that the object names belong to SceneObjects and not frames.
    INTR_ASSIGN_OR_RETURN(
        std::vector<intrinsic::WorldObjectName> existing_object_names,
        object_world_client_->ListObjectNames());

    absl::flat_hash_set<std::string> existing_object_names_set;
    for (const auto& name : existing_object_names) {
      existing_object_names_set.insert(name.value());
    }

    for (const auto& object_name : object_names.value()) {
      if (existing_object_names_set.contains(object_name)) {
        INTR_ASSIGN_OR_RETURN(intrinsic::world::WorldObject object,
                              object_world_client_->GetObject(
                                  intrinsic::WorldObjectName(object_name)));
        objects.push_back(object);
      }
    }
  }

  return objects;
}

absl::StatusOr<std::string> World::GetGltf(const std::string& geometry_ref,
                                           const std::string& renderable_ref) {
  intrinsic_proto::geometry::GetRenderableRequest request;
  *(request.mutable_geometry_storage_refs()->mutable_geometry_ref()) =
      geometry_ref;
  *(request.mutable_geometry_storage_refs()->mutable_renderable_ref()) =
      renderable_ref;

  auto client_context = std::make_unique<grpc::ClientContext>();
  client_context->set_deadline(std::chrono::system_clock::now() +
                               std::chrono::seconds(deadline_seconds_));

  intrinsic_proto::geometry::RenderableWithMetadata response;

  INTR_RETURN_IF_ERROR(intrinsic::ToAbslStatus(
      geometry_stub_->GetRenderable(client_context.get(), request, &response)));

  return response.renderable().gltf_string();
}

}  // namespace flowstate_ros_bridge
