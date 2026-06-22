#include "icon/utils/strerror.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <string_view>

namespace intrinsic {
namespace {

struct StrErrorParam {
  int err_num;
  std::string expected_str;
};

class StrErrorTest : public ::testing::TestWithParam<StrErrorParam> {};

TEST_P(StrErrorTest, ReturnsCorrectMessage) {
  const auto& param{GetParam()};
  const auto error_str{StrError(param.err_num)};
  EXPECT_EQ(std::string_view{error_str.data()}, param.expected_str);
}

TEST_P(StrErrorTest, EqualsStrerror) {
  const auto& param{GetParam()};
  const auto error_str{StrError(param.err_num)};
  EXPECT_EQ(std::string_view{error_str.data()}, ::strerror(param.err_num));
}

INSTANTIATE_TEST_SUITE_P(
    KnownErrors, StrErrorTest,
    ::testing::Values(StrErrorParam{0, "Success"},
                      StrErrorParam{EPERM, "Operation not permitted"},
                      StrErrorParam{ENOENT, "No such file or directory"},
                      StrErrorParam{EINTR, "Interrupted system call"},
                      StrErrorParam{EIO, "Input/output error"},
                      StrErrorParam{ENOMEM, "Cannot allocate memory"},
                      StrErrorParam{EACCES, "Permission denied"},
                      StrErrorParam{EINVAL, "Invalid argument"},
                      StrErrorParam{ENOSYS, "Function not implemented"}));

INSTANTIATE_TEST_SUITE_P(
    UnknownErrors, StrErrorTest,
    ::testing::Values(StrErrorParam{12345, "Unknown error 12345"},
                      StrErrorParam{-1, "Unknown error -1"},
                      StrErrorParam{-99, "Unknown error -99"},
                      StrErrorParam{999999, "Unknown error 999999"}));

}  // namespace

}  // namespace intrinsic

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
