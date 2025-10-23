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

#include "channel_factory.hpp"

#include <intrinsic/util/grpc/channel.h>
#include <intrinsic/util/status/status_macros.h>

#include <absl/log/log.h>

namespace local_flowstate_ros_bridge {

absl::StatusOr<std::shared_ptr<::grpc::Channel>>
ClusterChannelFactory::make_channel(absl::string_view /* address */) {
  LOG(INFO) << "Creating gRPC channel for cluster [" << cluster_ << "]"
            << " at [" << org_project_ << "]";
  INTR_ASSIGN_OR_RETURN(auto org_info,
                        intrinsic::Channel::OrgInfo::FromString(org_project_));
  INTR_ASSIGN_OR_RETURN(
      auto chan, intrinsic::Channel::MakeFromCluster(org_info, cluster_));
  return chan->GetChannel();
}

} // namespace local_flowstate_ros_bridge
