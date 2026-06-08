#include "intrinsic/utils/time.hpp"

#include <cstring>
#include <time.h>

namespace intrinsic
{

namespace
{
time_t to_time_t(std::chrono::steady_clock::time_point t)
{
  return std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::now() +
        duration_cast<std::chrono::system_clock::duration>(t - std::chrono::steady_clock::now()));
}

time_t to_time_t(std::chrono::system_clock::time_point t)
{
  return std::chrono::system_clock::to_time_t(t);
}
}

std::array<char, std::size(kTimeFormat)> FormatTime(const Time & t)
{
  std::time_t now_c = to_time_t(t);
  std::array<char, std::size(kTimeFormat)> out{'\0'};
  struct tm local;
  // See https://linux.die.net/man/3/gmtime_r - at least for glibc, this should
  // evaluate to true if gmtime_r() is available.
  //
  // It would be nice to be able to rely on std::gmtime_r(), but that is a C++26
  // function.
  // (and plain C gmtime_r() is C23)
  //
  // TODO: Clean up once we can rely on C++26
  #if _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _BSD_SOURCE || _SVID_SOURCE || _POSIX_SOURCE
  {
    struct tm * tm = gmtime_r(&now_c, &local);
    if (tm == nullptr) {return out;}
  }
  #else
  local = *gmtime(&now_c);
  #endif
  auto written_chars = std::strftime(std::data(out), std::size(out), "%FT%TZ", &local);
  if (written_chars == 0) {
    // the contents of `out` are undefined at this point, overwrite with zeroes
    // to be safe.
    std::memset(std::data(out), '\0', std::size(out));
  }
  return out;
}

}  // namespace intrinsic
