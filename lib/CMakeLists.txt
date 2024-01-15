add_subdirectory(Universal_hal)
add_subdirectory(Manikin_Software_Libraries_V2)

add_library(FastCRC FastCRC/src/FastCRCsw.cpp)
target_include_directories(FastCRC PUBLIC FastCRC/src)