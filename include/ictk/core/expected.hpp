#pragma once
#include <new>         // for placement new and explicit destructor calls
#include <utility>     // for move and forward
#include <type_traits> // for noexcept() 

#include "ictk/core/status.hpp" // ret code tyoe 

namespace ictk{

    template <class T>
    class Expected{
        public:
            // // no discard: ignore unused warns, if return values are not used. 

            /*
            1. Expected => success state
            2. Call private default constructor, makes ok_ = false and create a empty buffer
            3. mark e.ok_ = true/false, that we are going to hold a T in the storage
            4. placement new -> construct real T inside the raw buffer using copy/move constructor from v
            5. return r  -> NRVO or move; no heap
            */

            // // Lvalue overload -> takes const t& -> calls with an existing obj -> no extra copy at call 
            [[nodiscard]] static Expected success(const T& v) noexcept(std::is_nothrow_copy_constructible_v<T>){
                Expected e;   
                e.ok_ = true;
                ::new (e.storage_) T(v);
                return e;
            }
            
            // // Rvalue overload 
            [[nodiscard]] static Expected success(T&& v) noexcept(std::is_nothrow_move_constructible_v<T>){
                Expected e;
                e.ok_ = true;
                ::new (e.storage_) T(std::move(v)); // casts v to rvalue on T's move ctor is selected (no heap)
                return e;
            }
            
            /*
            Direct construction: built T inplace from constructor args
            Args... = list of arguments
            forwarding -> to preserve value category, l/rvalues stay the same
            */
            template <class ... Args>
            [[nodiscard]] static Expected emplace(Args&&... args) noexcept(std::is_nothrow_constructible_v<T, Args...>){
                Expected e;
                e.ok_ = true;
                ::new (e.storage_) T(std::forward<Args>(args)...);
                return e;
            }
            
            // // Failure
            [[nodiscard]] static Expected failure(Status s) noexcept{
                Expected e;
                e.ok_ = false;
                e.err_ = s;
                return e;
            }
            // // Copy constructor -> build new Expected <T> from existing o                // member init
            Expected(const Expected& o) noexcept(std::is_nothrow_copy_constructible_v<T>) : err_(o.err_), ok_(o.ok_){
                if (ok_) ::new (storage_) T(*o.ptr());
            }

            // // Move constructor -> build a new obs by moving from o
            Expected(Expected&& o) noexcept(std::is_nothrow_move_constructible_v<T>) : err_(o.err_), ok_(o.ok_){
                if (ok_) ::new (storage_) T(std::move(*o.ptr()));
                o.reset_();
            }
            
            // // Copy assignment
            Expected& operator=(const Expected& o) noexcept(std::is_nothrow_copy_constructible_v<T>){
                // // self assign guard
                if (this==&o) return *this; 
                if (ok_) ptr() -> ~T();     // destroy old T if had one
                err_ = o.err_;
                ok_ = o.ok_;
                if(ok_) ::new (storage_) T(*o.ptr());   // // copy const from o
                return *this;
            }
            
            // // Move assignment
            Expected& operator=(Expected&& o) noexcept(std::is_nothrow_move_constructible_v<T>){
                // self move guard
                if(this==&o) return *this;
                if (ok_) ptr() -> ~T();     // destroy old T if had one
                err_ = o.err_;
                ok_ = o.ok_;
                if (ok_) ::new (storage_) T(std::move(*o.ptr()));   // move construct from o
                o.reset_(); // leave empty o
                return *this;
            }
            
            // // Destructor
            ~Expected() noexcept{
                reset_();
            }

            // // helper functions (query)
            [[nodiscard]] bool has_value() const noexcept{
                return ok_;
            }

            [[nodiscard]] explicit operator bool() const noexcept{
                return ok_;
            }

            [[nodiscard]] Status status() const noexcept{
                return ok_ ? Status::kOK : err_;
            }

            // // accessors: safe for RT if caller checks has_value()
            [[nodiscard]] T& value() noexcept{
                return *ptr();
            }
            
            [[nodiscard]] const T& value() const noexcept{
                return *ptr();
            }

            // // move out helper for zero copy handoff
            [[nodiscard]] T take() noexcept(std::is_nothrow_move_constructible_v<T>){
                T tmp = std::move(*ptr());  // move from in place T
                ptr()->~T();    // destroy in place
                ok_ = false;    // now empty
                return tmp;     // ret by value
            }
            
        private:
            // // storage_ is a raw buffer of bytes of size T -> T is the type
            alignas(T) unsigned char storage_[sizeof(T)]{};
    
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