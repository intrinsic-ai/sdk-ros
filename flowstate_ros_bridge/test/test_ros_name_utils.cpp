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

TEST(PrefixFrameIdTest, PrependsPrefixAndDropsLeadingSlashes) {
  EXPECT_EQ(PrefixFrameId("", ""), "");
  EXPECT_EQ(PrefixFrameId("wc1/", ""), "");
  EXPECT_EQ(PrefixFrameId("wc1/", "/"), "");
  EXPECT_EQ(PrefixFrameId("", "robot/base_link"), "robot/base_link");
  EXPECT_EQ(PrefixFrameId("", "/robot/base_link"), "robot/base_link");
  EXPECT_EQ(PrefixFrameId("wc1/", "robot/base_link"), "wc1/robot/base_link");
  EXPECT_EQ(PrefixFrameId("wc1/", "/robot/base_link"), "wc1/robot/base_link");
  EXPECT_EQ(PrefixFrameId("wc1/", "//robot/base_link"), "wc1/robot/base_link");
}

TEST(ValidateWorkcellIdTest, AcceptsValidNamespaces) {
  EXPECT_EQ(ValidateWorkcellId(""), "");
  EXPECT_EQ(ValidateWorkcellId("wc1"), "");
  EXPECT_EQ(ValidateWorkcellId("cell_a"), "");
  EXPECT_EQ(ValidateWorkcellId("cell_a/arm"), "");
}

TEST(ValidateWorkcellIdTest, RejectsInvalidNamespaces) {
  EXPECT_NE(ValidateWorkcellId("wc-1"), "");
  EXPECT_NE(ValidateWorkcellId("wc 1"), "");
  EXPECT_NE(ValidateWorkcellId("a//b"), "");
  EXPECT_NE(ValidateWorkcellId("1wc"), "");
  EXPECT_NE(ValidateWorkcellId("a/1b"), "");
}

}  // namespace
}  // namespace flowstate_ros_bridge
