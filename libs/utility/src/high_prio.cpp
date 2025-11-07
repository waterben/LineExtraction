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

#  include <limits>
#  include <unistd.h>

namespace lsfm {

void setHighPriority() {
  const id_t pid = static_cast<id_t>(getpid());
  const long nice_quanta = sysconf(_SC_NZERO);
  const int requested =
      nice_quanta > std::numeric_limits<int>::max() ? -std::numeric_limits<int>::max() : -static_cast<int>(nice_quanta);

  if (setpriority(PRIO_PROCESS, pid, requested) != 0) {
    std::cout << "Failed to change priority" << std::endl;
  } else {
    std::cout << "Priority changed successfully!" << std::endl;
  }
}
}  // namespace lsfm
#endif
