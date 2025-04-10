// Copyright 2024 Intrinsic Innovation LLC
/**
 * You are hereby granted a non-exclusive, worldwide, royalty-free license to use,
 * copy, modify, and distribute this Intrinsic SDK in source code or binary form for use
 * in connection with the services and APIs provided by Intrinsic Innovation LLC (“Intrinsic”).
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * If you use this Intrinsic SDK with any Intrinsic services, your use is subject to the Intrinsic
 * Platform Terms of Service [https://intrinsic.ai/platform-terms].  If you create works that call
 * Intrinsic APIs, you must agree to the terms of service for those APIs separately. This license
 * does not grant you any special rights to use the services.
 *
 * This copyright notice shall be included in all copies or substantial portions of the software.
 */

#include "test_cpp_skill.h"
#include "test_cpp_skill.pb.h"

#include <memory>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/notification.h"
#include "intrinsic/perception/proto/camera_config.pb.h"
#include "intrinsic/perception/service/proto/camera_server.grpc.pb.h"
#include "intrinsic/skills/cc/skill_utils.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"

using ::com::example::TestCppSkillParams;
using ::com::example::TestCppSkillResult;

using ::intrinsic::skills::ExecuteRequest;
using ::intrinsic::skills::SkillInterface;
using ::intrinsic::skills::ExecuteContext;
using ::intrinsic::WaitForChannelConnected;

// -----------------------------------------------------------------------------
// Skill signature.
// -----------------------------------------------------------------------------

std::unique_ptr<SkillInterface>
TestCppSkill::CreateSkill()
{
  return std::make_unique<TestCppSkill>();
}

// -----------------------------------------------------------------------------
// Skill execution.
// -----------------------------------------------------------------------------

absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
TestCppSkill::Execute(const ExecuteRequest& request, ExecuteContext& context)
{
  LOG(INFO) << "TestCppSkill::Execute";

  // Get parameters.
  INTR_ASSIGN_OR_RETURN(auto params, request.params<TestCppSkillParams>());

  auto result = std::make_unique<TestCppSkillResult>();
  return result;
}
