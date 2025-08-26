#include <cmath>
#include <vector>
#include <cassert>

#include "ictk/safety/saturation.hpp"

using namespace ictk;

int main(){
    std::vector<Scalar> u{-2.0, -0.5, 0.0, 0.5, 3.0};

                        // // umin, umax 
    safety::Saturation sat_loose(-1.0, 2.0);  //wider feasible set
    auto u1 = u;  // independent copy
    sat_loose.apply(u1);

    safety::Saturation sat_tight(-0.5, 1.0); //subset of that set
    auto u2 = u;  // independent copy
    sat_tight.apply(u2);

    // Monotonicity : |u_tight| <= |u_loose| componenet wise

    for (std::size_t i=0; i<u.size(); ++i){
        assert(std::abs(u2[i]) <= std::abs(u1[i]) + 1e-15);  // compare magnitude wise | 1e-15 handles floating roundoff
    }
    return 0;
}
