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

#ifndef LOCAL_FLOWSTATE_ROS_BRIDGE__CHANNEL_FACTORY_HPP
#define LOCAL_FLOWSTATE_ROS_BRIDGE__CHANNEL_FACTORY_HPP

#include <flowstate_ros_bridge/channel_factory.hpp>
#include <grpcpp/grpcpp.h>

namespace local_flowstate_ros_bridge {

class ClusterChannelFactory : public flowstate_ros_bridge::ChannelFactory {
public:
  ClusterChannelFactory(absl::string_view org_project,
                        absl::string_view solution)
      : org_project_(org_project), cluster_(solution) {}

  absl::StatusOr<std::shared_ptr<::grpc::Channel>>
  make_channel(absl::string_view address) override;

private:
  absl::string_view org_project_;
  absl::string_view cluster_;
};

} // namespace local_flowstate_ros_bridge

#endif
