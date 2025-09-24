#include <span>
#include <array>
#include <vector>
#include <cstring>
#include <cassert>
#include <cstdio>

#include "hash.hpp" 

// BLAKE3 C API 
#if ICTK_RECORDER_BACKEND_MCAP
extern "C"{
    #include "blake3.h"
}
#endif

namespace ictk::tools::hash{
    // In-memory BLAKE3 -> Initializes, feeds, finalizes (No heap) O(n) on input
    #if ICTK_RECORDER_BACKEND_MCAP
    std::array<std::uint8_t, 32> blake3_256(const void* data, size_t len){
        std::array<std::uint8_t, 32> out{};
        blake3_hasher h;
        blake3_hasher_init(&h);
        blake3_hasher_update(&h, data, len);
        blake3_hasher_finalize(&h, out.data(), out.size());
        return out;
    }

    // Stream BLAKE3 -> 64KiB chunking -> checks read error
    std::array<std::uint8_t, 32> blake3_256_file(const char* path){
        std::FILE* f = std::fopen(path, "rb");
        if (!f) return {};
        blake3_hasher h;
        blake3_hasher_init(&h);
        std::vector<unsigned char> buf(1<<16);
        size_t n = 0;
        while((n=fread(buf.data(), 1, buf.size(), f)) > 0){
            blake3_hasher_update(&h, buf.data(), n); 
        }
        std::fclose(f);
        std::array<std::uint8_t, 32> out{};
        blake3_hasher_finalize(&h, out.data(), out.size());
        return out;
    }
    #else
        // stubs when backend disables
        std::array<std::uint8_t, 32> blake3_256(const void*, size_t){
            return{};
        }
        std::array<std::uint8_t, 32> blake3_256_file(const char*){
            return {};
        }
    #endif

    // // tiny SHA-256 (portable)
    namespace{
        /*
        Internal SHA-256 state:
            1. bit length
            2. 8 word hash
            3. 512 bit block buffer
            4. index
        */
        struct  Ctx{
            std::uint64_t len_bits = 0;
            std::uint32_t h[8];
            unsigned char buf[64];
            std::size_t idx = 0;
        };

        // Internal SHA-256 primitives
        inline uint32_t ROTR(uint32_t x, int n){
            return (x>>n) | (x<<(32-n));
        }
        inline uint32_t Ch(uint32_t x, uint32_t y, uint32_t z){
            return (x&y) ^ (~x&z);
        }
        inline uint32_t Maj(uint32_t x, uint32_t y, uint32_t z){
            return (x&y) ^ (x&z) ^ (y&z);
        }
        inline uint32_t bigSigma0(uint32_t x){
            return ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22);
        }
        inline uint32_t bigSigma1(uint32_t x){
            return ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25);
        }
        inline uint32_t smallSigma0(uint32_t x){
            return ROTR(x, 7) ^ ROTR(x, 18) ^ (x >> 3);
        }
        inline uint32_t smallSigma1(uint32_t x){ 
            return ROTR(x,17) ^ ROTR(x,19) ^ (x>>10); 
        }

        // Round Constants
        // // Source: https://en.wikipedia.org/wiki/SHA-2
        static const uint32_t K[64] = {
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
            0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
            0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
            0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
            0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
            0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
            0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
        };

        // Initialize chaining state 
        void init(Ctx& c){
            c.len_bits = 0;
            c.idx = 0;
            uint32_t H[8] = {
                0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
            };
            std::memcpy(c.h, H, sizeof(H));
        }

        //  Core compression on one 512 bit block
        void transform(Ctx& c, const unsigned char* d){
            uint32_t w[64];
            // 1 to 16 -> first 16 W[0..15]
            for (int i=0; i<16; i++){
                w[i] = (std::uint32_t(d[4*i])<<24) | (std::uint32_t(d[4*i+1])<<16) | (std::uint32_t(d[4*i+2])<<8) | std::uint32_t(d[4*i+3]);
            }
            // remaining: W[16..63]
            for (int i=16;i<64;i++){
                w[i] = smallSigma1(w[i-2]) + w[i-7] + smallSigma0(w[i-15]) + w[i-16];
                }
            // init working var to current hash value
            std::uint32_t a=c.h[0], b=c.h[1], c0=c.h[2], d0=c.h[3], e=c.h[4], f=c.h[5], g=c.h[6], h=c.h[7];

            // compress function
            for (int i = 0; i < 64; i++){
                const std::uint32_t T1 = h + bigSigma1(e) + Ch(e, f, g) + K[i] + w[i];
                const std::uint32_t T2 = bigSigma0(a) + Maj(a, b, c0);

                h = g;
                g = f;
                f = e;
                e = d0 + T1;
                d0 = c0;
                c0 = b;
                b = a;
                a = T1 + T2;
            }

            // add the compressed chunk to the current hash value
            c.h[0] += a;
            c.h[1] += b;
            c.h[2] += c0;
            c.h[3] += d0;
            c.h[4] += e;
            c.h[5] += f;
            c.h[6] += g;
            c.h[7] += h;
        }

        // absorb bytes, update bit length, transform at 64 byte boundaries
        void update(Ctx &c, const void* data, size_t len){
            const unsigned char* p = static_cast<const unsigned char*>(data);
            c.len_bits += std::uint64_t(len) * 8u;
            while (len--){
                c.buf[c.idx++] = *p++;
                if (c.idx==64){
                    transform(c, c.buf);
                    c.idx = 0;
                }
            }
        }

        // SHA 256 padding and length encoding 
        void final(Ctx &c, unsigned char out[32]){
            c.buf[c.idx++] = 0x80;
            if (c.idx > 56){
                while (c.idx < 64) c.buf[c.idx++] = 0;
                transform(c, c.buf);
                c.idx = 0;
            }
            while (c.idx < 56) c.buf[c.idx++] = 0;
            for (int i=7; i>=0; i--) c.buf[c.idx++] = static_cast<unsigned char>((c.len_bits >> (8*i)) & 0xffu);

            transform(c, c.buf);
            for (int i=0; i<8; i++){
                out[4*i+0] = static_cast<unsigned char>((c.h[i] >> 24) & 0xffu);
                out[4*i+1] = static_cast<unsigned char>((c.h[i] >> 16) & 0xffu);
                out[4*i+2] = static_cast<unsigned char>((c.h[i] >> 8 ) & 0xffu);
                out[4*i+3] = static_cast<unsigned char>((c.h[i]      ) & 0xffu);
            }     
        }
    }

    // One shot in memory hash
    std::array<std::uint8_t, 32> sha256(const void* data, size_t len){
        Ctx c;
        init(c);
        update(c, data, len);
        std::array<std::uint8_t, 32> out{};
        final(c, out.data());
        return out;
    }

    // Streaming SHA 256 with error check
    std::array<std::uint8_t, 32> sha256_file(const char* path){
        std::FILE* f = std::fopen(path, "rb");
        if (!f) return {};
        Ctx c;
        init(c);

        std::vector<unsigned char> buf(1 << 16);
        size_t n=0;
        while ((n=fread(buf.data(), 1, buf.size(), f)) > 0){
            update(c, buf.data(), n);
        }

        std::fclose(f);
        std::array<std::uint8_t, 32> out{};
        final(c, out.data());
        return out;
    }

    std::string to_hex(std::span<const std::uint8_t> bytes){
        static const char* h = "0123456789abcdef";
        std::string s;
        s.reserve(bytes.size() * 2);
        for (auto b: bytes){
            const unsigned ub = static_cast<unsigned>(b);
            s.push_back(h[(ub >> 4) & 0x0F]);
            s.push_back(h[ub & 0x0F]);
        }
        return s;
    }
} // namespace ictk::tools::hash