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

#ifndef FLOWSTATE_ROS_BRIDGE__CHANNEL_FACTORY_HPP
#define FLOWSTATE_ROS_BRIDGE__CHANNEL_FACTORY_HPP

#include <absl/time/time.h>
#include <grpcpp/grpcpp.h>

#include <absl/status/statusor.h>
#include <absl/strings/string_view.h>

#include <functional>

namespace flowstate_ros_bridge {
using ChannelFactory =
    std::function<absl::StatusOr<std::shared_ptr<::grpc::Channel>>(
        absl::string_view address)>;

class ClientChannelFactory {
public:
  explicit ClientChannelFactory(absl::Duration deadline,
                                grpc::ChannelArguments channel_args)
      : deadline_(std::move(deadline)), channel_args_(std::move(channel_args)) {
  }

  absl::StatusOr<std::shared_ptr<::grpc::Channel>>
  operator()(absl::string_view address);

private:
  absl::Duration deadline_;
  grpc::ChannelArguments channel_args_;
};

} // namespace flowstate_ros_bridge

#endif
