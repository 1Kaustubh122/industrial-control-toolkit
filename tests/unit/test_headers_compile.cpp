#include "ictk/all.hpp"
#include <type_traits>   
#include <cassert>

int main() {
  // Basic API smoke
  ictk::Dims d{.ny=1,.nu=1,.nx=0};
  (void)d;
  static_assert(std::is_same_v<ictk::Scalar,double>);
  return 0;
}
