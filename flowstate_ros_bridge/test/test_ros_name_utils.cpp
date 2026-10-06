// Copyright 2026 Intrinsic Innovation LLC
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

#include <gtest/gtest.h>

#include "flowstate_ros_bridge/ros_name_utils.hpp"

namespace flowstate_ros_bridge {
namespace {

TEST(TrimSlashesTest, StripsLeadingAndTrailingSlashes) {
  EXPECT_EQ(TrimSlashes(""), "");
  EXPECT_EQ(TrimSlashes("/"), "");
  EXPECT_EQ(TrimSlashes("///"), "");
  EXPECT_EQ(TrimSlashes("wc1"), "wc1");
  EXPECT_EQ(TrimSlashes("/wc1/"), "wc1");
  EXPECT_EQ(TrimSlashes("//wc1//"), "wc1");
  EXPECT_EQ(TrimSlashes("a/b"), "a/b");
  EXPECT_EQ(TrimSlashes("/a/b/"), "a/b");
}

TEST(NormalizeTfPrefixTest, ProducesSingleTrailingSlash) {
  EXPECT_EQ(NormalizeTfPrefix(""), "");
  EXPECT_EQ(NormalizeTfPrefix("/"), "");
  EXPECT_EQ(NormalizeTfPrefix("///"), "");
  EXPECT_EQ(NormalizeTfPrefix("wc1"), "wc1/");
  EXPECT_EQ(NormalizeTfPrefix("wc1/"), "wc1/");
  EXPECT_EQ(NormalizeTfPrefix("/wc1/"), "wc1/");
  EXPECT_EQ(NormalizeTfPrefix("//wc1//"), "wc1/");
  EXPECT_EQ(NormalizeTfPrefix("a/b"), "a/b/");
}

}  // namespace
}  // namespace flowstate_ros_bridge
