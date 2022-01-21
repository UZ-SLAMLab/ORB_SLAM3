#include "common.hpp"

#include <cstdio>
#include <cstdlib>

namespace Sophus {
void ensureFailed(char const* function, char const* file, int line,
                  char const* description) {
  std::printf("Sophus ensure failed in function '%s', file '%s', line %d.\n",
              file, function, line);
  std::printf("Description: %s\n", description);
  std::abort();
}
}  // namespace Sophus
