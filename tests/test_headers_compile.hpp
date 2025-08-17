#include "ictk/all.hpp"
#include <type_traits>

int main(){
    /*
    ictk::dims d;
    d.ny = 1;
    d.nu = 1;
    d.nx = 0;
    */
    ictk::Dims d{.ny=1, .nu=1, .nx=0};

    // Suppress Warning 
    (void)d;

    // Recheck scalar value, for future use -> build fail if Scaler != double
    static_assert(std::is_same_v<ictk::Scalar, double>);
    return 0;
}