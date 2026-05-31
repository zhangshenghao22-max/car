include(CMakeForceCompiler)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-a)

set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(WIN32)
    set(TARGET "D:/toolschain/gcc-arm-none-eabi-10-2020-q4-major-win32/bin/")
    set(CMAKE_C_COMPILER "${TARGET}arm-none-eabi-gcc.exe")
    set(CMAKE_CXX_COMPILER "${TARGET}arm-none-eabi-g++.exe")
    set(CMAKE_AR "${TARGET}arm-none-eabi-ar.exe")
else()
    set(CMAKE_C_COMPILER "aarch64-none-elf-gcc")
    set(CMAKE_CXX_COMPILER "aarch64-none-elf-g++")
    set(CMAKE_AR "aarch64-none-elf-ar")
endif()


set(CMAKE_C_FLAGS_INIT " -g -march=armv8-a -mtune=cortex-a55 -fdiagnostics-color=always -Wall -Wno-cpp -O0 -gdwarf-2 -ffunction-sections -fdata-sections -Dgcc -D'__attribute__(x)='" CACHE STRING "")
set(CMAKE_CXX_FLAGS_INIT " -g -march=armv8-a -mtune=cortex-a55 -fdiagnostics-color=always -Wall -Wno-cpp -O0 -gdwarf-2 -ffunction-sections -fdata-sections -Dgcc -D'__attribute__(x)='" CACHE STRING "")

get_filename_component(script_path "${CMAKE_CURRENT_LIST_FILE}" ABSOLUTE)

message("################################ Toolchain include ######################################")
string(REGEX MATCH "^(.*software)" BSP_ROOT_DIR "${script_path}")
message("BSP_ROOT_DIR Path: ${BSP_ROOT_DIR}")

include_directories(
    ${BSP_ROOT_DIR}/rt-thread/include              # kernel include
    ${BSP_ROOT_DIR}/rt-thread/components/finsh     # finsh include
    ${BSP_ROOT_DIR}/rt-thread/components/net/sal/include                      # #include <netdb.h>
    ${BSP_ROOT_DIR}/rt-thread/components/net/sal/include/socket               
    ${BSP_ROOT_DIR}/rt-thread/components/net/sal/include/socket/sys_socket    # #include <sys/socket.h>
    ${BSP_ROOT_DIR}/rt-thread/components/net/netdev/include
    ${BSP_ROOT_DIR}/rt-thread/components/net/netdev/include/arpa              # #include <arpa/inet.h>
    ${BSP_ROOT_DIR}/RK3588                                                           # rtconfig.h
)

set(__BIG_ENDIAN__ 0)
