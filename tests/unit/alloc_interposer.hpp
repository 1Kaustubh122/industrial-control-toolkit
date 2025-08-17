#pragma once
#include <cstdint>

// // ictk_test:: out of ictk:: 
namespace ictk_test{
    struct AllocStats{
        std::uint64_t news{0};
        std::uint64_t deletes{0};
        std::uint64_t new_aligned{0};
        std::uint64_t delete_aligned{0};
    };
    
    // // read only
    const AllocStats& alloc_stats();

    // // reset counters
    void reset_alloc_stats();
    
    // Read fields, for convenience -> ictk_test::new_cout() instead of ictk_test::alloc_stats().news  (Same for rest)
    inline std::uint64_t new_count(){
        return alloc_stats().news;
    }

    inline std::uint64_t delete_count(){
        return alloc_stats().deletes;
    }

    inline std::uint64_t new_aligned_count(){
        return alloc_stats().new_aligned;
    }

    inline std::uint64_t delete_aligned_count(){
        return alloc_stats().delete_aligned;
    }
    
} // namespace ictk_test
