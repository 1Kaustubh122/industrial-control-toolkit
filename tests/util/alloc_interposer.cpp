#include "util/alloc_interposer.hpp"
#include <atomic>
#include <cstdlib>   // for malloc and free
#include <new>       // for operator new and delete 
#include <cstddef>   // for unsigned int type

#if defined(_WIN32)     // only for windows system
    #include <malloc.h>
#endif


static inline void* ictk_fail_alloc() noexcept{
    #if defined(ICTK_NO_EXCEPTIONS)
        std::abort();
    # if defined(__GNUC__) || defined(__clang__)
        __builtin_unreachable();
    # endif
        return nullptr; 
    #else
        return ictk_fail_alloc();
    #endif
}

namespace ictk_test{

    // // atomic avoid race conditions | Global counter g_* -> tracks how many allocations happens 
    static std::atomic<std::uint64_t> g_news{0}, g_deletes{0}, g_new_aligned{0}, g_delete_aligned{0};
    /*  
        "No heap after start() for determinism and WCET"
        g -> global
        g_news = count all normal allocations -> operator new and operator new[]
        g_deletes = count all normal frees -> operator delete and operator delete[]
        g_new_aligned = count all normal allocations -> operator new(size t, std::align_val_t) and array form
        g_deletes_aligned = count all normal frees -> matching aligned news

     */

    // // counters -> returns a const refernce to a function local static AllocStats s
    const AllocStats& alloc_stats(){
        static AllocStats s{};
        s.news = g_news.load(std::memory_order_relaxed);
        s.deletes = g_deletes.load(std::memory_order_relaxed);
        s.new_aligned = g_new_aligned.load(std::memory_order_relaxed);
        s.delete_aligned = g_delete_aligned.load(std::memory_order_relaxed);

        return s;
    }

    // // Resetting global counter to 0, 
    void reset_alloc_stats(){
        g_news.store(0, std::memory_order_relaxed);
        g_deletes.store(0, std::memory_order_relaxed);
        g_new_aligned.store(0, std::memory_order_relaxed);
        g_delete_aligned.store(0, std::memory_order_relaxed);
    }

} // namespace ictk_test


// // Global operator new and delete override 
void * operator new(std::size_t sz){
    ictk_test::g_news.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(sz)){
        return p;
    }
    return ictk_fail_alloc();
}

void* operator new[](std::size_t sz){
    ictk_test::g_news.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(sz)){
        return p;
    }
    return ictk_fail_alloc();
}

void operator delete(void* p) noexcept{
    ictk_test::g_deletes.fetch_add(1, std::memory_order_relaxed);
    std::free(p);
}

void operator delete[](void* p) noexcept{
    ictk_test::g_deletes.fetch_add(1, std::memory_order_relaxed);
    std::free(p);
}


// // Sized deleteds -> covers (GCC, Clang, MSVC variants)
void operator delete(void* p, std::size_t) noexcept{
    ictk_test::g_deletes.fetch_add(1, std::memory_order_relaxed);
    std::free(p);
}

void operator delete[](void* p, std::size_t) noexcept{
    ictk_test::g_deletes.fetch_add(1, std::memory_order_relaxed);
    std::free(p);
}

void* operator new(std::size_t sz, std::align_val_t al){
    ictk_test::g_new_aligned.fetch_add(1, std::memory_order_relaxed);
    std::size_t align = static_cast<std::size_t>(al);
#if defined(_WIN32)
    void* p = _aligned_malloc(sz, align);
    if (!p){
        return ictk_fail_alloc();
    }
    return p;
#else
    void* p = nullptr;
    if (posix_memalign(&p, align, sz) != 0 || !p) return ictk_fail_alloc();
    return p;
#endif
}

void operator delete(void* p, std::align_val_t) noexcept{
    ictk_test::g_delete_aligned.fetch_add(1, std::memory_order_relaxed);
#if defined(_WIN32)
    _aligned_free(p);
#else
    std::free(p);
#endif
}

void* operator new[](std::size_t sz, std::align_val_t al){
    return ::operator new(sz, al);
}

void operator delete[](void* p, std::align_val_t al) noexcept{
    return ::operator delete(p, al);
}

void operator delete(void* p, std::size_t, std::align_val_t) noexcept{
    ictk_test::g_delete_aligned.fetch_add(1, std::memory_order_relaxed);
#if defined(_WIN32)
    _aligned_free(p);
#else
    std::free(p);
#endif
}

void operator delete[](void* p, std::size_t, std::align_val_t al) noexcept{
    return ::operator delete(p, al);
}

// // nothrow net and deltes
void* operator new(std::size_t sz, const std::nothrow_t&) noexcept{
    ictk_test::g_news.fetch_add(1, std::memory_order_relaxed);
    return std::malloc(sz);
}

void operator delete(void* p, const std::nothrow_t&) noexcept{
  ictk_test::g_deletes.fetch_add(1, std::memory_order_relaxed);
  std::free(p);
}
void* operator new[](std::size_t sz, const std::nothrow_t&) noexcept {
  ictk_test::g_news.fetch_add(1, std::memory_order_relaxed);
  return std::malloc(sz);
}
void operator delete[](void* p, const std::nothrow_t&) noexcept{
  ictk_test::g_deletes.fetch_add(1, std::memory_order_relaxed);
  std::free(p);
}