#pragma once

#include <array>
#include <cmath>
#include <string>
#include <cstdint>
#include <charconv>
#include <string_view>

namespace ictk::tools::acr::jsond{
    // // helpers
    inline void _hex2(std::string& out, unsigned v){
        // lookup table of hexadecimal digits
        static constexpr char kHex[] = "0123456789abcdef";

        // shift bits of v by 4 position and mask it
        out.push_back(kHex[(v >> 4) & 0xF]);
        out.push_back(kHex[v & 0xF]);
    } // _hex2

    // // string escaping
    inline void esc(std::string& out, std::string_view s){
        for (unsigned char c:s){
            // case over all named escapes
            switch (c){
            case '\"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\b':
                out += "\\b";
                break;
            case '\f':
                out += "\\f";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                if (c < 0x20){
                    out += "\\u00";
                    _hex2(out, c);
                }else{
                    out.push_back(static_cast<char>(c));
                }
            }
        }
    }

    // wrap escape value in quotes
    inline void str(std::string& out, std::string_view v){
        out.push_back('"');
        esc(out, v);
        out.push_back('"');
    }

    // emit a quoted key plus a colon
    inline void key(std::string& out, std::string_view k){
        out.push_back('"');
        esc(out, k);
        out += "\":";
    }

    // // numbers -> locale independent
    inline void num(std::string& out, std::int64_t v){
        // fixed stack buffer 
        std::array<char, 24> buf{};

        // write decimal v into buf
        auto r = std::to_chars(buf.data(), buf.data() + buf.size(), v);
        out.append(buf.data(), r.ptr);
    }

    inline void unum(std::string& out, std::uint64_t v){
        // fixed stack buffer 
        std::array<char, 24> buf{};

        // write decimal v into buf
        auto r = std::to_chars(buf.data(), buf.data() + buf.size(), v);
        out.append(buf.data(), r.ptr);
    }

    inline void real(std::string& out, double v){
        // JSON have no NAN
        if (!std::isfinite(v)){
            out += "null";
            return;
        }

        // Normalizes -0.0 to 0.0*
        if (v == 0.0) v = 0.0;    
        
        // buffer
        std::array<char, 64> buf{};

        //                                                             // general picks fixed    // precision 17
        auto r = std::to_chars(buf.data(), buf.data() + buf.size(), v, std::chars_format::general, 17);
        out.append(buf.data(), r.ptr);
    }

    inline void boolean(std::string& out, bool b){
        out += (b ? "true" : "false");
    }

    inline void null(std::string& out){
        out += "null";
    }

    // // containers
    template <class F>
    inline void array(std::string& out, F emit_elems){
        out.push_back('[');
        emit_elems();
        out.push_back(']');
    }

    template <class F>
    inline void object(std::string& out, F emit_members){
        out.push_back('{');
        emit_members();
        out.push_back('}');
    }

    inline void comma(std::string& out){
        out.push_back(',');
    }
} // namespace ictk::tools::acr::jsond