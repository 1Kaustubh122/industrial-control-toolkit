#include "ictk/all.hpp"
#include "util/alloc_interposer.hpp"
#include "util/dummy_controller.hpp"

#include <vector>
#include <cstring>
#include <cassert>

using namespace ictk;
using ictk_test::DummyController;

static std::vector<std::uint8_t> run_once(std::size_t N, Dims dims, dt_ns dt){
    DummyController ctrl;

    // // pre-allocate arena buffer -> for testing it would be unused by dummy but it matches the ICTK API.
    alignas(64) std::byte buffer[4096];
    MemoryArena arena(buffer, sizeof(buffer));

    auto st = ctrl.init(dims, dt, arena, {});
    assert(st == Status::kOK);

    st = ctrl.start();
    assert(st == Status::kOK);

    std::vector<std::uint8_t> bytes;
    bytes.reserve(N * (dims.nu * sizeof(Scalar)));

    // // prepare static IO
    std::vector<Scalar> y(dims.ny, 1.23);
    std::vector<Scalar> r(dims.ny, 0.42);

    PlantState ps{
        .y=std::span<const Scalar>(y.data(), y.size()),
        .xhat=std::span<const Scalar>(),
        .t=0,
        .valid_bits=~0ull
    };

    Setpoint sp{
        .r=std::span<const Scalar>(y.data(), y.size()),
        .preview_horizon_len=0
    };

    std::vector<Scalar> u (dims.nu, 0.0);

    Result res{
        .u = std::span<Scalar>(u.data(), u.size()),
        .health={}
    };

    // // sanpshot allocation counters after warmup allocations
    ictk_test::reset_alloc_stats();
    auto new0 = ictk_test::new_count();
    auto newA0 = ictk_test::new_aligned_count();

    for (std::size_t k=0; k<N; ++k){
        ps.t += dt;
        UpdateContext ctx{ps, sp};
        st = ctrl.update(ctx, res);
        assert(st == Status::kOK);

        // Serialized outputs
        const std::uint8_t* p= reinterpret_cast<const std::uint8_t*>(u.data());
        bytes.insert(bytes.end(), p, p+u.size()*sizeof(Scalar));
    }

    auto new1 = ictk_test::new_count();
    auto newA1 = ictk_test::new_aligned_count();


    // // no alloc allwed during update ticks
    assert(new1 == new0);
    assert(newA1 == newA0);

    return bytes;

}


int main(){
    const Dims dim{.ny=2, .nu=3, .nx=0};
    const dt_ns dt = 1'000'000; // // 1ms
    const std::size_t N = 1000;

    auto a = run_once(N, dim, dt);
    auto b = run_once(N, dim, dt);

    assert (a.size() == b.size());
    // // byte-exact determinism within same build
    assert(std::memcmp(a.data(), b.data(), a.size()) == 0);
    
    return 0;
}