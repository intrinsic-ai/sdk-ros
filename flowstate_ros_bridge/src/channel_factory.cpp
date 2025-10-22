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

#include "flowstate_ros_bridge/channel_factory.hpp"

#include <intrinsic/util/grpc/channel.h>
#include <intrinsic/util/status/status_macros.h>

namespace flowstate_ros_bridge {

absl::StatusOr<std::shared_ptr<::grpc::Channel>>
ClientChannelFactory::make_channel(absl::string_view address) {
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
  channel_args.SetMaxReceiveMessageSize(10000000); // 10 MB
  channel_args.SetMaxSendMessageSize(10000000);    // 10 MB

  return intrinsic::CreateClientChannel(address, absl::Now() + this->deadline_,
                                        channel_args);
}

} // namespace flowstate_ros_bridge
