#include <cctype>
#include <fstream>
#include <system_error>

#include "io/sidecar_semantics.hpp"

namespace ictk::tools::acr::sidecar{
    /// @brief confirm that it's exactly 64 hex char (32 bytes)
    /// @param s : non owning view of bytes
    /// @return false if not valid else true
    bool is_hex_64(std::string_view s){
        if (s.size() != 64) return false;
        for (unsigned char c : s){
            // must be 0-9 or a-f or A-F
            if (!std::isxdigit(c)) return false;
        }
        return true;
    } // is_hex_64

    /// @brief it's a Case-insnsitive equal check for hexadecimal string
    /// @param a first hex string (non owning view)
    /// @param b second hex string (non owning view)
    /// @return true iff a and b have identical length and matches every byte ignoring case, else false
    bool ieq_hex(std::string_view a, std::string_view b){
        // length check
        if (a.size() != b.size()) return false;

        // char a and char b to verify
        for (std::size_t i=0; i < a.size(); ++i){
            unsigned char ca = static_cast<unsigned char>(a[i]);
            unsigned char cb = static_cast<unsigned char>(b[i]);

            // A->F = a-f :=> compare each value
            if (std::tolower(ca) != std::tolower(cb)) return false; // if not matched
        }

        // if all matched
        return true;
    } // ieq_hex


    /// @brief open and read its entire contents into memory
    /// @param p file path to read 
    /// @return return full contents of the file as std::string type, or empty in the case of error
    static inline std::string read_all(const std::filesystem::path& p){
        // size = 256 KB
        constexpr std::uintmax_t kMaxSidecarBytes = 256 * 1024;

        std::error_code ec;
        const auto sz = std::filesystem::file_size(p, ec);

        // if sentinel 1 or empty file or > 256 KB -> fail
        if (ec || sz == static_cast<std::uintmax_t>(-1) || sz == 0 || sz > kMaxSidecarBytes) return {};

        // open the file in bindary mode
        std::ifstream in(p, std::ios::binary);

        if (!in) return{};

        std::string buf;

        // make buf same size as of the file
        buf.resize(static_cast<std::size_t>(sz));

        // read the file -> copy to buf
        in.read(buf.data(), static_cast<std::streamsize>(sz));

        if (!in) return {};
        return buf;
    } // read_all


    /// @brief parse a deterministic sidecar JSON file toextract payload, schema hash metadata
    /// @param p path of the sidecar JSON file
    /// @return Parsed SideCarData if valid, std::nullopt incase of error
    std::optional<SidecarData> parse_sidecar(const std::filesystem::path& p){
        /* 
        SidecarData: 
            payload_alg
            payload_hex
            bfbs_sha256
        create an empty struct to fill these details    
        */
        SidecarData out{};

        // read entire file into string s
        const std::string s = read_all(p);

        // if fail
        if (s.empty()) return std::nullopt;

        // // payload hash
        // search for word payload_hash
        const auto ph = s.find("\"payload_hash\"");
        // if not found
        if (ph == std::string::npos) return std::nullopt;

        // find '{' after ph ('payload_hash') then find '}' after obr ('{') 
        const auto obr = s.find('{', ph);
        const auto cbr = s.find('}', obr);
        // if not found
        if (obr == std::string::npos || cbr == std::string::npos) return std::nullopt;

        // now the string payload_obj contains the small section of text 
        const std::string payload_obj = s.substr(obr, cbr - obr + 1);

        // find keys inside the payload_obj -> check for "alg" and "value" fields 
        const auto alg_k = payload_obj.find("\"alg\"");
        const auto val_k = payload_obj.find("\"value\"");
        // if not found
        if (alg_k == std::string::npos || val_k == std::string::npos) return std::nullopt;

        // Extract quoted values
        const auto q1 = payload_obj.find('"', alg_k + 5);
        const auto q2 = (q1 == std::string::npos) ? q1 : payload_obj.find('"', q1 + 1);
        const auto q3 = payload_obj.find('"', val_k + 7);
        const auto q4 = (q3 == std::string::npos) ? q3 : payload_obj.find('"', q3 + 1);
        
        // if not found
        if (q1 == std::string::npos || q2 == std::string::npos || q3 == std::string::npos || q4 == std::string::npos) return std::nullopt;
        
        // now extract those values
        out.payload_alg = payload_obj.substr(q1 + 1, q2 - q1 -1);
        out.payload_hex = payload_obj.substr(q3 + 1, q4 - q3 -1);

        // validate them
        if (out.payload_alg != "BLAKE3-256") return std::nullopt;
        if (!is_hex_64(out.payload_hex)) return std::nullopt;


        // // bfbs hashes
        const auto bh = s.find("\"bfbs_hashes\"");
        if (bh != std::string::npos){
            // find "[" after bh "bfbs_hashes"
            const auto a1 = s.find('[', bh);

            // find "]" after a1 "["
            const auto a2 = (a1 == std::string::npos) ? a1 : s.find(']', a1);

            if (a1 != std::string::npos && a2 != std::string::npos){

                // now arr is the small section of values between [...]
                const std::string arr = s.substr(a1, a2 - a1 + 1);

                std::size_t pos = 0;
                // lopp through the arr
                while (true){
                    /*
                    Walk through the array text and slice each object like "alg" : "SHA-256", "value" : "..." 
                    */
                    const auto objb = arr.find('{', pos);
                    if (objb == std::string::npos) break;
                    const auto obje = arr.find('}', objb);
                    if (obje == std::string::npos) break;

                    const std::string obj = arr.substr(objb, obje - objb + 1);
                    pos = obje + 1;

                    const auto ak = obj.find("\"alg\"");
                    const auto vk = obj.find("\"value\"");

                    if (ak == std::string::npos || vk == std::string::npos) continue;

                    // find ""
                    const auto aqs = obj.find('"', ak + 5);
                    const auto aqe = (aqs == std::string::npos) ? aqs : obj.find('"', aqs + 1);
                    const auto vqs = obj.find('"', vk + 7);
                    const auto vqe = (vqs == std::string::npos) ? vqs : obj.find('"', vqs + 1);

                    if (aqs == std::string::npos || aqe == std::string::npos || vqs == std::string::npos || vqe == std::string::npos) continue;

                    // small section of found text
                    const std::string alg = obj.substr(aqs + 1, aqe - aqs - 1);
                    const std::string val = obj.substr(vqs + 1, vqe - vqs - 1);

                    // validate them
                    if (alg == "SHA-256" && is_hex_64(val)) out.bfbs_sha256.push_back(val);
                }
            }
        }
        // retrun final struct
        return out;
    }
} // namespace ictk::tools::acr::sidecar