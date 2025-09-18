#pragma once
#include <new>
#include <utility>
#include <type_traits>

#include "ictk/core/status.hpp"

namespace ictk{

    // // making Expected template to create many classes
    template <class T>
    class Expected{
        public:
            Expected() = delete;
            
            [[nodiscard]] static Expected success(const T& v) noexcept(std::is_nothrow_copy_constructible_v<T>){
                Expected e;
                e.ok_ = true;
                ::new (e.storage_) T(v);
                return e;
            }
                
            [[nodiscard]] static Expected success(T&& v) noexcept(std::is_nothrow_move_constructible_v<T>){
                Expected e;
                e.ok_ = true;
                ::new (e.storage_) T(std::move(v));
                return e;
            }
             
            
            [[nodiscard]] static Expected failure(Status s) noexcept{
                    Expected e;
                    e.ok_ = false;
                    e.err_ = s;
                    return e;
            }
                
            [[nodiscard]] Expected(const Expected& o) noexcept(std::is_nothrow_copy_constructible_v<T>) : err_(o.err_), ok_(o.ok_){
                if (ok_) new (storage_) T(*o.ptr());
            }

            [[nodiscard]] Expected(const Expected&& o) noexcept(std::is_nothrow_move_constructible_v) : err_(o.err_), ok_(o.ok_){
                if (ok_) new (storage_) T(std::move(*o.ptr()));
                o.reset_();
            }
            
            Expected& operator=(const Expected& o) noexcept(std::is_nothrow_copy_constructible_v<T>){
                if (this==&0) return *this;
                if (ok_) ptr() -> ~T();
                err_ = o.err_;
                ok_ = o.ok_;
                if(ok_) new (storage_) T(*o.ptr());
                return *this;
            }
                
            Expected& operator=(Expected&& o) noexcept(std::is_nothrow_move_constructible_v<T>){
                if(this==&o) return *this;
                if (ok_) ptr() -> ~T();
                err_ = o.err_;
                ok_ = o.ok_;
                if (ok_) new (storage_) T(std::move(*o.ptr()));
                o.reset_();
                return *this;
            }
            
            ~Expected(){
                reset_();
            }
            
            
        private:
            // // storage_ is a raw buffer of bytes of size T -> T is the type
            alignas(T) unsigned char storage_[sizeof(T)];
    
            // // Error status, default Ok
            Status err_{Status::kOK};
    
            // // flag -> tells current state of the Expected obj
            bool ok_{false};
    
            // Helper functions
            T* ptr() noexcept{
                // // storage contains T object, memory address as a pointer to T
                return reinterpret_cast<T*>(storage_);
            }
    
            // // preventing modifying T
            const T* ptr() const noexcept{
                return reinterpret_cast<const T*>(storage_);
            }
    
            void reset_() noexcept{
                if (ok_){
                    ptr() -> ~T();
                    ok_ = false;
                }
            }
    
            Expected() = default;
        };
    } // namespace ictk