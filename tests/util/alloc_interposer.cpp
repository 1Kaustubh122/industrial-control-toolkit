#include "unit/alloc_interposer.hpp"
#include <atomic>
#include <cstdlib>
#include <new>
#include <cstddef>

#if defined(_WIN32)
    #include <malloc.h>
#endif


namespace ictk_test{

    // // atomic avoid race conditions
    static std::atomic<std::uint64_t> g_news{0}, g_deletes{0}, g_new_aligned{0}, g_delete_aligned{0};

    // // counters -> returns a const refernce to a function local static AllocStats s
    const AllocStats& alloc_stats(){
        static AllocStats s{};
        s.news = g_news.load(std::memory_order_relaxed);
        s.deletes = g_deletes.load(std::memory_order_relaxed);
        s.new_aligned = g_new_aligned.load(std::memory_order_relaxed);
        s.delete_aligned = g_delete_aligned.load(std::memory_order_relaxed);

        return s;
    }

} // namespace ictk_test


