#include <ictk/version.hpp>

extern "C" const char* ictk_version_string(){
    return ictk::kVersionStr;
}

extern "C" int ictk_c_abi_version(){
    return ictk::kCAbiVersion;
}