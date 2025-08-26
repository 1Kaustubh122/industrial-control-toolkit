#include <cmath>
#include <vector>
#include <cassert>

#include "ictk/safety/bumpless_transfer.hpp"

using namespace ictk;

int main(){
    std::vector <Scalar> hold{10, -5, 2}; // u_hold
    std::vector<Scalar>  goal{0, 0, 0};   // target
    std::vector<Scalar>  out(3, 0);       // actual output
    const Scalar alpha = 0.2;

    // after k steps, error <= (1-alpha)^k * hold-goal
    std::vector<Scalar> err0(3);
    
    for (std::size_t i=0; i<3; ++i) err0[i] =  hold[i] - goal[i];

    std::vector<Scalar> cur = hold; 

    for (int k=1; k<=10; ++k){
        safety::BumplessMixer::mix(cur, goal, out, alpha);
        // bound
        for (std::size_t i=0; i<3; ++i){
            const Scalar e = out[i] - goal[i];  // // ek
            const Scalar bound = std::pow(1.0-alpha, k) * std::abs(err0[i]) + 1e-12;
            assert(std::abs(e) <= bound);
        }
        cur = out;      // advnace
    }
    return 0;

}