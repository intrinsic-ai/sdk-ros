#include "icon/flatbuffers/fixed_string.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string_view>

#include "flatbuffers/array.h"
#include "icon/flatbuffers/fixed_string_test.fbs.h"
#include "icon/utils/status.h"
#include "icon/utils/status_and_expected_test_macros.h"

namespace intrinsic_fbs {
namespace {

using ::testing::ElementsAreArray;
using ::testing::Eq;
using ::testing::HasSubstr;
using ::testing::Lt;

constexpr std::string_view kShortString = "foo";
constexpr int8_t kSignedShortString[] = {'f', 'o', 'o'};
constexpr uint8_t kUnsignedShortString[] = {'f', 'o', 'o'};

constexpr std::string_view kBufferSizedString = "foobarX";
constexpr int8_t kSignedBufferSizedString[] = {'f', 'o', 'o', 'b',
                                               'a', 'r', 'X'};
constexpr uint8_t kUnsignedBufferSizedString[] = {'f', 'o', 'o', 'b',
                                                  'a', 'r', 'X'};

constexpr std::string_view kBufferSizedString2 = "foobarXY";
constexpr int8_t kSignedBufferSizedString2[] = {'f', 'o', 'o', 'b',
                                                'a', 'r', 'X', 'Y'};
constexpr uint8_t kUnsignedBufferSizedString2[] = {'f', 'o', 'o', 'b',
                                                   'a', 'r', 'X', 'Y'};

constexpr std::string_view kLongString = "foobarbazz is a long string";
constexpr int8_t kSignedLongString[] = {
    'f', 'o', 'o', 'b', 'a', 'r', 'b', 'a', 'z', 'z', ' ', 'i', 's', ' ',
    'a', ' ', 'l', 'o', 'n', 'g', ' ', 's', 't', 'r', 'i', 'n', 'g'};
constexpr uint8_t kUnsignedLongString[] = {
    'f', 'o', 'o', 'b', 'a', 'r', 'b', 'a', 'z', 'z', ' ', 'i', 's', ' ',
    'a', ' ', 'l', 'o', 'n', 'g', ' ', 's', 't', 'r', 'i', 'n', 'g'};

constexpr std::string_view kStringWithNullByte = "foo\0bar";
constexpr int8_t kSignedStringWithNullByte[] = {'f', 'o', 'o', 0};
constexpr uint8_t kUnsignedStringWithNullByte[] = {'f', 'o', 'o', 0};

TEST(FixedCStyleStringTest, WhenSourceStringIsShorterThanDestination) {
  FixedStringsStruct fixed_strings;
  // Fill the buffers with ones (because we check for null-termination later).
  std::memset(fixed_strings.mutable_signed_message()->Data(), 1,
              fixed_strings.signed_message()->size());
  std::memset(fixed_strings.mutable_unsigned_message()->Data(), 1,
              fixed_strings.unsigned_message()->size());

  // Copy the short string into the buffers.
  EXPECT_EQ(
      StringCopy(fixed_strings.mutable_signed_message(), kShortString).code,
      intrinsic::StatusCode::kOk);
  EXPECT_EQ(
      StringCopy(fixed_strings.mutable_unsigned_message(), kShortString).code,
      intrinsic::StatusCode::kOk);

  // Verify that the buffers contains the short string up to the length of the
  // string.
  EXPECT_THAT(kSignedShortString,
              ElementsAreArray(fixed_strings.signed_message()->Data(),
                               sizeof(kSignedShortString)));
  EXPECT_THAT(kUnsignedShortString,
              ElementsAreArray(fixed_strings.unsigned_message()->Data(),
                               sizeof(kUnsignedShortString)));

  // Verify that the buffers are null-terminated.
  EXPECT_EQ(fixed_strings.signed_message()->Get(sizeof(kSignedShortString)), 0);
  EXPECT_EQ(fixed_strings.unsigned_message()->Get(sizeof(kUnsignedShortString)),
            0);

  // Verify that the string_view is the same as the short string.
  {
    auto converted = StringView(fixed_strings.signed_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(), kShortString);
  }
  {
    auto converted = StringView(fixed_strings.unsigned_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(), kShortString);
  }
}

TEST(FixedCStyleStringTest, WhenSourceStringHasNullByteThanDestination) {
  FixedStringsStruct fixed_strings;
  // Fill the buffers with ones (because we check for null-termination later).
  std::memset(fixed_strings.mutable_signed_message()->Data(), 1,
              fixed_strings.signed_message()->size());
  std::memset(fixed_strings.mutable_unsigned_message()->Data(), 1,
              fixed_strings.unsigned_message()->size());

  // Copy the short string into the buffers.
  EXPECT_EQ(
      StringCopy(fixed_strings.mutable_signed_message(), kStringWithNullByte)
          .code,
      intrinsic::StatusCode::kOk);
  EXPECT_EQ(
      StringCopy(fixed_strings.mutable_unsigned_message(), kStringWithNullByte)
          .code,
      intrinsic::StatusCode::kOk);

  // Verify that the buffers contains the short string up to the null byte.
  EXPECT_THAT(kSignedStringWithNullByte,
              ElementsAreArray(fixed_strings.signed_message()->Data(),
                               sizeof(kSignedStringWithNullByte)));
  EXPECT_THAT(kUnsignedStringWithNullByte,
              ElementsAreArray(fixed_strings.unsigned_message()->Data(),
                               sizeof(kUnsignedStringWithNullByte)));

  // Verify that the buffers are null-terminated.
  EXPECT_EQ(
      fixed_strings.signed_message()->Get(sizeof(kSignedStringWithNullByte)),
      0);
  EXPECT_EQ(fixed_strings.unsigned_message()->Get(
                sizeof(kUnsignedStringWithNullByte)),
            0);

  // Verify that the string_view is the same as the short string.
  {
    auto converted = StringView(fixed_strings.signed_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(), kStringWithNullByte);
    // Using std::string_view::data() as if it is null terminated could cause
    // undefined behavior, but here we know that it does contain a null byte.
    EXPECT_EQ(converted.value().length(),
              std::strlen(kStringWithNullByte.data()));
  }
  {
    auto converted = StringView(fixed_strings.unsigned_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(), kStringWithNullByte);
    // Using std::string_view::data() as if it is null terminated could cause
    // undefined behavior, but here we know that it does contain a null byte.
    EXPECT_EQ(converted.value().length(),
              std::strlen(kStringWithNullByte.data()));
  }
}

TEST(FixedCStyleStringTest, WhenSourceStringWith0IsSameLengthThanDestination) {
  FixedStringsStruct fixed_strings;
  // Fill the buffers with ones (because we check for null-termination later).
  std::memset(fixed_strings.mutable_signed_message()->Data(), 1,
              fixed_strings.signed_message()->size());
  std::memset(fixed_strings.mutable_unsigned_message()->Data(), 1,
              fixed_strings.unsigned_message()->size());

  // Copy the short string into the buffers.
  EXPECT_EQ(
      StringCopy(fixed_strings.mutable_signed_message(), kBufferSizedString)
          .code,
      intrinsic::StatusCode::kOk);
  EXPECT_EQ(
      StringCopy(fixed_strings.mutable_unsigned_message(), kBufferSizedString)
          .code,
      intrinsic::StatusCode::kOk);

  // Verify that the buffers contain the short string up to the length of the
  // string.
  EXPECT_THAT(kSignedBufferSizedString,
              ElementsAreArray(fixed_strings.signed_message()->Data(),
                               sizeof(kSignedBufferSizedString)));
  EXPECT_THAT(kUnsignedBufferSizedString,
              ElementsAreArray(fixed_strings.unsigned_message()->Data(),
                               sizeof(kUnsignedBufferSizedString)));
  // Verify that the buffers are null-terminated.
  EXPECT_EQ(
      fixed_strings.signed_message()->Get(sizeof(kSignedBufferSizedString)), 0);
  EXPECT_EQ(
      fixed_strings.unsigned_message()->Get(sizeof(kUnsignedBufferSizedString)),
      0);

  // Verify that the string_view is the same as the short string.
  {
    auto converted = StringView(fixed_strings.signed_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(), kBufferSizedString);
  }
  {
    auto converted = StringView(fixed_strings.unsigned_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(), kBufferSizedString);
  }
}

TEST(FixedCStyleStringTest, WhenSourceStringIsSameLengthAsDestination) {
  FixedStringsStruct fixed_strings;
  // Fill the buffers with ones (because we check for null-termination later).
  std::memset(fixed_strings.mutable_signed_message()->Data(), 1,
              fixed_strings.signed_message()->size());
  std::memset(fixed_strings.mutable_unsigned_message()->Data(), 1,
              fixed_strings.unsigned_message()->size());

  // Copy the short string into the buffers.
  INTR_EXPECT_OK(
      StringCopy(fixed_strings.mutable_signed_message(), kBufferSizedString2));
  INTR_EXPECT_OK(StringCopy(fixed_strings.mutable_unsigned_message(),
                            kBufferSizedString2));

  // Verify that the buffers contain the short string up to the length of the
  // string -1 (to leave room for a trailing null byte).
  EXPECT_THAT(absl::MakeSpan(kSignedBufferSizedString2,
                             sizeof(kSignedBufferSizedString2) - 1),
              ElementsAreArray(fixed_strings.signed_message()->Data(),
                               sizeof(kSignedBufferSizedString2) - 1));
  EXPECT_THAT(absl::MakeSpan(kUnsignedBufferSizedString2,
                             sizeof(kUnsignedBufferSizedString2) - 1),
              ElementsAreArray(fixed_strings.unsigned_message()->Data(),
                               sizeof(kUnsignedBufferSizedString2) - 1));
  // Verify that the buffers are null-terminated.
  EXPECT_EQ(fixed_strings.signed_message()->Get(
                sizeof(kSignedBufferSizedString2) - 1),
            0);
  EXPECT_EQ(fixed_strings.unsigned_message()->Get(
                sizeof(kUnsignedBufferSizedString2) - 1),
            0);

  // Verify that the string_view is the same as the short string minus the last
  // character, which is the null-terminator.
  {
    auto converted = StringView(fixed_strings.signed_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(),
              std::string_view(kBufferSizedString2.data(),
                               kBufferSizedString2.size() - 1));
    EXPECT_EQ(converted.value().data()[kBufferSizedString2.size() - 1], '\0');
  }
  {
    auto converted = StringView(fixed_strings.unsigned_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(),
              std::string_view(kBufferSizedString2.data(),
                               kBufferSizedString2.size() - 1));
    EXPECT_EQ(converted.value().data()[kBufferSizedString2.size() - 1], '\0');
  }
}

TEST(FixedCStyleStringTest, WhenSourceStringIsLongerThanDestination) {
  FixedStringsStruct fixed_strings;
  // Fill the buffers with ones (because we check for null-termination later).
  std::memset(fixed_strings.mutable_signed_message()->Data(), 1,
              fixed_strings.signed_message()->size());
  std::memset(fixed_strings.mutable_unsigned_message()->Data(), 1,
              fixed_strings.unsigned_message()->size());

  // Copy the short string into the buffers.
  EXPECT_EQ(
      StringCopy(fixed_strings.mutable_signed_message(), kLongString).code,
      intrinsic::StatusCode::kOk);
  EXPECT_EQ(
      StringCopy(fixed_strings.mutable_unsigned_message(), kLongString).code,
      intrinsic::StatusCode::kOk);

  // Verify that the buffers contain the short string up to the length of the
  // buffer.
  EXPECT_THAT(absl::MakeSpan(kSignedLongString,
                             fixed_strings.signed_message()->size() - 1),
              ElementsAreArray(fixed_strings.signed_message()->Data(),
                               fixed_strings.signed_message()->size() - 1));
  EXPECT_THAT(absl::MakeSpan(kUnsignedLongString,
                             fixed_strings.unsigned_message()->size() - 1),
              ElementsAreArray(fixed_strings.unsigned_message()->Data(),
                               fixed_strings.unsigned_message()->size() - 1));

  // Verify that the buffers are null-terminated.
  EXPECT_EQ(fixed_strings.signed_message()->Get(
                fixed_strings.signed_message()->size() - 1),
            0);
  EXPECT_EQ(fixed_strings.unsigned_message()->Get(
                fixed_strings.unsigned_message()->size() - 1),
            0);

  // Verify that the string_view is the same as the short string.
  {
    auto converted = StringView(fixed_strings.signed_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(),
              std::string_view(kLongString.data(),
                               fixed_strings.signed_message()->size() - 1));
    EXPECT_EQ(
        converted.value().data()[fixed_strings.signed_message()->size() - 1],
        '\0');
  }
  {
    auto converted = StringView(fixed_strings.unsigned_message());
    ASSERT_TRUE(converted.has_value()) << ToString(converted.error());
    EXPECT_EQ(converted.value(),
              std::string_view(kLongString.data(),
                               fixed_strings.unsigned_message()->size() - 1));
    EXPECT_EQ(
        converted.value().data()[fixed_strings.unsigned_message()->size() - 1],
        '\0');
  }
}

TEST(FixedCStyleStringTest, WhenArrayIsNullptr) {
  // Verify that StringCopy does not crash when trying to copy into a nullptr
  // array.
  ::flatbuffers::Array<int8_t, 8>* null_array = nullptr;
  {
    auto result = StringCopy(null_array, kShortString);
    EXPECT_EQ(result.code, intrinsic::StatusCode::kInvalidArgument)
        << ToString(result);
    EXPECT_THAT(result.GetMessage(), HasSubstr("nullptr")) << ToString(result);
  }

  // Verify that StringView does not crash when trying to create a string_view
  // from a nullptr array.
  {
    auto result = StringView(null_array);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, intrinsic::StatusCode::kInvalidArgument);
    EXPECT_THAT(result.error().GetMessage(), HasSubstr("nullptr"));
  }
}

TEST(FixedStringTest, WhenSourceStringIsShorterThanDestinationSigned) {
  TestMessageWithSignedArray signed_message;
  StringCopy(&signed_message, kShortString);
  // Verify that the size of the message is the same as the size of the short
  // string.
  EXPECT_EQ(signed_message.size(), kShortString.size());

  // Verify that the size of the message is less than the max size of the array.
  EXPECT_THAT(
      signed_message.size(),
      Lt(internal::flatbuffers::ArraySize<
          std::remove_cv_t<std::remove_pointer_t<
              TestMessageWithSignedArray::Traits::template FieldType<0>>>>::
             value));

  // Verify that the message contains the short string.
  EXPECT_THAT(
      absl::MakeSpan(signed_message.data()->data(), signed_message.size()),
      ElementsAreArray(kSignedShortString));

  // Verify that the string_view is the same as the short string.
  EXPECT_THAT(StringView(&signed_message), Eq(kShortString));
}

TEST(FixedStringTest, WhenSourceStringIsShorterThanDestinationUnsigned) {
  TestMessageWithUnsignedArray unsigned_message;
  StringCopy(&unsigned_message, kShortString);
  // Verify that the size of the message is the same as the size of the short
  // string.
  EXPECT_EQ(unsigned_message.size(), kShortString.size());

  // Verify that the size of the message is less than the max size of the array.
  EXPECT_THAT(
      unsigned_message.size(),
      Lt(internal::flatbuffers::ArraySize<
          std::remove_cv_t<std::remove_pointer_t<
              TestMessageWithSignedArray::Traits::template FieldType<0>>>>::
             value));

  // Verify that the message contains the short string.
  EXPECT_THAT(
      absl::MakeSpan(unsigned_message.data()->data(), unsigned_message.size()),
      ElementsAreArray(kUnsignedShortString));

  // Verify that the string_view is the same as the short string.
  EXPECT_THAT(StringView(&unsigned_message), Eq(kShortString));
}

TEST(FixedStringTest, WhenSourceStringSameSizeAsDestinationSigned) {
  TestMessageWithSignedArray signed_message;
  StringCopy(&signed_message, kBufferSizedString2);
  // Verify that the size of the message is the same as the size of the buffer
  // sized string.
  EXPECT_EQ(signed_message.size(), kBufferSizedString2.size());

  // Verify that the size of the message is max size of the array.
  EXPECT_EQ(
      signed_message.size(),
      internal::flatbuffers::ArraySize<std::remove_cv_t<std::remove_pointer_t<
          TestMessageWithSignedArray::Traits::template FieldType<0>>>>::value);

  // Verify that the message contains the buffer sized string.
  EXPECT_THAT(
      absl::MakeSpan(signed_message.data()->data(), signed_message.size()),
      ElementsAreArray(kBufferSizedString2));

  // Verify that the string_view is the same as the buffer sized string.
  EXPECT_THAT(StringView(&signed_message), Eq(kBufferSizedString2));
}

TEST(FixedStringTest, WhenSourceStringSameSizeAsDestinationUnsigned) {
  TestMessageWithUnsignedArray unsigned_message;
  StringCopy(&unsigned_message, kBufferSizedString2);
  // Verify that the size of the message is the same as the size of the buffer
  // sized string.
  EXPECT_EQ(unsigned_message.size(), kBufferSizedString2.size());

  // Verify that the size of the message is max size of the array.
  EXPECT_EQ(
      unsigned_message.size(),
      internal::flatbuffers::ArraySize<std::remove_cv_t<std::remove_pointer_t<
          TestMessageWithSignedArray::Traits::template FieldType<0>>>>::value);

  // Verify that the message contains the buffer sized string.
  EXPECT_THAT(
      absl::MakeSpan(unsigned_message.data()->data(), unsigned_message.size()),
      ElementsAreArray(kBufferSizedString2));

  // Verify that the string_view is the same as the buffer sized string.
  EXPECT_THAT(StringView(&unsigned_message), Eq(kBufferSizedString2));
}

TEST(FixedStringTest, WhenSourceStringIsLongerThanDestinationSigned) {
  TestMessageWithSignedArray signed_message;
  StringCopy(&signed_message, kLongString);

  // Verify that the size of the message is max size of the array.
  EXPECT_EQ(
      signed_message.size(),
      internal::flatbuffers::ArraySize<std::remove_cv_t<std::remove_pointer_t<
          TestMessageWithSignedArray::Traits::template FieldType<0>>>>::value);

  // Verify that the message contains the long string up to the size of the
  // array.
  EXPECT_THAT(
      absl::MakeSpan(signed_message.data()->data(), signed_message.size()),
      Eq(absl::MakeSpan(kSignedLongString, signed_message.size())));

  // Verify that the string_view is the same as the long string up to the size
  // of the array.
  EXPECT_THAT(StringView(&signed_message),
              ElementsAreArray(
                  absl::MakeSpan(kSignedLongString, signed_message.size())));
}

TEST(FixedStringTest, WhenSourceStringIsLongerThanDestinationUnsigned) {
  TestMessageWithUnsignedArray unsigned_message;
  StringCopy(&unsigned_message, kLongString);

  // Verify that the size of the message is max size of the array.
  EXPECT_EQ(
      unsigned_message.size(),
      internal::flatbuffers::ArraySize<std::remove_cv_t<std::remove_pointer_t<
          TestMessageWithSignedArray::Traits::template FieldType<0>>>>::value);

  // Verify that the message contains the long string up to the size of the
  // array.
  EXPECT_THAT(
      absl::MakeSpan(unsigned_message.data()->data(), unsigned_message.size()),
      Eq(absl::MakeSpan(kUnsignedLongString, unsigned_message.size())));

  // Verify that the string_view is the same as the long string up to the size
  // of the array.
  EXPECT_THAT(StringView(&unsigned_message),
              ElementsAreArray(absl::MakeSpan(kUnsignedLongString,
                                              unsigned_message.size())));
}

TEST(FixedStringTest, WhenArrayIsNullptr2) {
  // Verify that StringCopy does not crash when trying to copy into a nullptr
  // array.
  TestMessageWithUnsignedArray* null_message = nullptr;
  StringCopy(null_message, kShortString);

  // Verify that StringView does not crash when trying to create a string_view
  // from a nullptr array.
  std::string_view null_string_view = StringView(null_message);
  EXPECT_EQ(null_string_view, "");
}

}  // namespace
}  // namespace intrinsic_fbs

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
