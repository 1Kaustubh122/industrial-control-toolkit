#include "ictk/core/controller_base.hpp"
#include "ictk/all.hpp"
#include <vector>
#include <cassert>
#include <cstddef>

using namespace ictk;

struct Trace{
    std::vector<char> seq;  // // collects letters

    // // Pre and post match the hook signatures from IController; user is the opaque cookie
    static void pre(std::span<Scalar>, void* user){
        static_cast<Trace*>(user)->seq.push_back('P');
    }
    static void post(std::span<const Scalar>, std::span<Scalar>, void* user){
        static_cast<Trace*>(user)->seq.push_back('O');
    }
};

// // Spy controller to record order: Core(C) -> Prep(P) -> Sat(S) -> Rate(R) -> Jerk(K) -> AW(A) -> Post(O)
class SpyController final: public ControllerBase{
    public:
        explicit SpyController(Trace& t) : tr_(t){}

    private:
        [[nodiscard]] Status compute_core(const UpdateContext&, std::span<Scalar> u) noexcept override{
            for (auto& ui:u) ui = 0.0;
            tr_.seq.push_back('C');
            return Status::kOK;
        }

        SatStep apply_saturation(std::span<Scalar>) noexcept override{
            tr_.seq.push_back('S');
            return {};
        }
        std::uint64_t apply_rate_limit(std::span<Scalar>) noexcept override{
            tr_.seq.push_back('R');
            return 0;
        }
        std::uint64_t apply_jerk_limit(std::span<Scalar>) noexcept override{
            tr_.seq.push_back('J');
            return 0;
        }
        void anti_windup_update(const UpdateContext&, std::span<const Scalar>, std::span<const Scalar>) noexcept override{
            tr_.seq.push_back('A');
        }

        Trace & tr_;
};


 int main(){
    const Dims d{
        .ny =1,
        .nu = 1,
        .nx = 0
    };
    // // problem size

    const dt_ns dt = 1'000'000; // // 1 ms
    // // fixed sampke time

    Trace tr{};
    Hooks hooks{
        &Trace::pre, // function pointer for pre_clamp
        &Trace::post,// function pointer for post_arbitrate
        &tr          // opaque pointer to user data
    };
    // // install hoooksm user = &tr, that's how hooks access seq

    alignas(64) std::byte buf[1024];
    MemoryArena arena(buf, sizeof(buf));
    // // Pre allocated scratch buffer for deterministic memory
    
    SpyController c(tr);
    // // Create the controller

    [[maybe_unused]] auto st = c.init(d, dt, arena, hooks);
    // life cycle init
    assert(st == Status::kOK);

    st = c.start();
    // life cycle start
    assert(st == Status::kOK);

    std::vector<Scalar> y{1.0}, r{0.0}, u(d.nu, 0.0);

    PlantState ps{
        std::span<const Scalar>(y.data(), y.size()), // outputs
        {},                                          // xhat -> no state estimates
        0,                                           // initially time = 0
        ~0ull                                        // valid bits
    };

    Setpoint sp{
        std::span<const Scalar>(r.data(), r.size()), // target or reference
        0                                            // horizen len
    };

    Result res{
        std::span<Scalar>(u.data(), u.size()), // command buffer
        {}                                     // health
    };

    ps.t += dt; // // one tick
    UpdateContext ctx{
        ps,
        sp
    };

    st = c.update(ctx, res);
    // // runs the pipeline once
    assert(st == Status::kOK);

    [[maybe_unused]] const char expected[] = {'C', 'P', 'S', 'R', 'J', 'A', 'O'};
    /*
    Safety step order:
    C -> compute_core  -> raw commands (eg: PID = Kp*error)
    P -> pre hook      -> let user see/modify raw command
    S -> saturation    -> clip to hard physical limits (eg: motor torque can't exceed +-5Nm)
    R -> rate limit    -> dont change too fast per tick (eg: don't jump 0->5 instantly, only 1Nm/ms)
    J -> jerk limit    -> smooth acceleration change
    A -> anti-windup   -> if we clipped, tell integrators to stop accumulating error
    O -> post hook     -> final hook, after all safety. User can log what actually went out.
    */
    assert(tr.seq.size() == sizeof(expected));

    for (std::size_t i=0; i< tr.seq.size(); ++i){
        assert(tr.seq[i] == expected[i]);
    }

    return 0;
 }