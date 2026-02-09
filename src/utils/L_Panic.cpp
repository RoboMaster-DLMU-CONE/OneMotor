#include <stdexcept>

#include <one/utils/Panic.hpp>

namespace one {
void panic(const char *message) { throw std::runtime_error(message); }
} // namespace one
