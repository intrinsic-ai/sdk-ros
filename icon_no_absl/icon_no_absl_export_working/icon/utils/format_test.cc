#include "icon/utils/format.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>

namespace intrinsic {
namespace {

using ::testing::HasSubstr;

struct CustomStreamableType {
  std::string name;
  std::string title;
  int age;
};

template <class Ostream>
inline Ostream&& operator<<(Ostream&& str, const CustomStreamableType& t) {
  if (!t.title.empty()) {
    str << t.title << " ";
  }
  str << t.name << " (" << t.age << ")";
  return std::forward<Ostream>(str);
}
}  // namespace

TEST(FormatMap, WorksWithBuiltinTypes) {
  // someone who is good at the economy please help me budget this. my family is
  // dying
  auto accounts = std::unordered_map<std::string, int>{
      {"Food", 200},     {"Data", 150},    {"Rent", 800},
      {"Candles", 3600}, {"Utility", 150},
  };
  std::string formatted =
      FormatMap(accounts, /*element_separator=*/"\n", /*pair_separator=*/" $",
                /*key_quotes=*/"|", /*value_quotes=*/"*");
  // The map is unordered, so we expect all of these substrings, but in an
  // unknown order.
  EXPECT_THAT(formatted, HasSubstr("|Food| $*200*"));
  EXPECT_THAT(formatted, HasSubstr("|Data| $*150*"));
  EXPECT_THAT(formatted, HasSubstr("|Rent| $*800*"));
  EXPECT_THAT(formatted, HasSubstr("|Candles| $*3600*"));
  EXPECT_THAT(formatted, HasSubstr("|Utility| $*150*"));
}

TEST(FormatMap, WorksWithCustomType) {
  auto accounts = std::unordered_map<std::string, CustomStreamableType>{
      {"Captain", {.name = "Jane Doe", .title = "Dame", .age = 53}},
      {"Bosun", {.name = "Frank", .title = "", .age = 19}},
  };

  std::string formatted =
      FormatMap(accounts, /*element_separator=*/"\n", /*pair_separator=*/": ",
                /*key_quotes=*/"'",
                /*value_quotes=*/"");
  // The map is unordered, so we expect all of these substrings, but in an
  // unknown order.
  EXPECT_THAT(formatted, HasSubstr("'Captain': Dame Jane Doe (53)"));
  EXPECT_THAT(formatted, HasSubstr("'Bosun': Frank (19)"));
}

}  // namespace intrinsic

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
