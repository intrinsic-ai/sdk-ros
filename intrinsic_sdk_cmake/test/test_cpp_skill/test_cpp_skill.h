// Copyright 2025 Intrinsic Innovation LLC
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

#ifndef TEST_CPP_SKILL_H
#define TEST_CPP_SKILL_H

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "gz/transport/Node.hh"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"


class TestCppSkill final : public intrinsic::skills::SkillInterface {
 public:
  static std::unique_ptr<intrinsic::skills::SkillInterface> CreateSkill();

  absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
  Execute(
    const intrinsic::skills::ExecuteRequest& request,
    intrinsic::skills::ExecuteContext& context
  ) override;
};

#endif  // TEST_CPP_SKILL_H
