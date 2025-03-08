#include <utility/high_prio.hpp>

#include <iostream>

#ifdef _WIN32
#  include <windows.h>

namespace lsfm {
void setHighPriority() {
  if (!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS)) {
    std::cout << "Failed to change priority" << std::endl;
  } else {
    std::cout << "Priority changed successfully!" << std::endl;
  }
}
}  // namespace lsfm
#else

#  include <sys/resource.h>

#  include <unistd.h>

namespace lsfm {

void setHighPriority() {
  if (setpriority(PRIO_PROCESS, getpid(), -sysconf(_SC_NZERO)) != 0) {
    std::cout << "Failed to change priority" << std::endl;
  } else {
    std::cout << "Priority changed successfully!" << std::endl;
  }
}
}  // namespace lsfm
#endif
