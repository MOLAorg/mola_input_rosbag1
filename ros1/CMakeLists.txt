cmake_minimum_required(VERSION 3.12)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-unused-parameter -Wno-deprecated-declarations")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wno-deprecated-declarations")

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 9)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-catch-value")
endif()

#find_package(catkin REQUIRED COMPONENTS cpp_common roscpp_serialization roscpp_traits rostime roslz4)
find_package(Boost REQUIRED COMPONENTS date_time filesystem program_options regex)
find_package(BZip2 REQUIRED)
find_library(lz4_LIBRARIES NAMES lz4)

file(GLOB CONSOLE_BRIDGE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/console_bridge/src/*.cpp")
file(GLOB CPP_COMMON_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/roscpp_core/cpp_common/src/*.cpp")
file(GLOB ROSCPP_SERIALIZATION_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/roscpp_core/roscpp_serialization/src/*.cpp")
file(GLOB ROSTIME_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/roscpp_core/rostime/src/*.cpp")

file(GLOB ROSBAG_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/ros_comm/tools/rosbag_storage/src/*.cpp")
file(GLOB ROSLZ4_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/ros_comm/utilities/roslz4/src/[a-z]*.c")

add_library(rosbag STATIC ${ROSBAG_SRCS} ${ROSTIME_SRCS} ${CPP_COMMON_SRCS} ${ROSCPP_SERIALIZATION_SRCS} ${ROSLZ4_SRCS} ${CONSOLE_BRIDGE_SRCS})

# Vendored roslz4 (ros_comm/utilities/roslz4) embeds its own xxhash.c, whose
# XXH32/XXH32_update/XXH32_digest symbol names collide with those exported by
# the system's libxxhash.so (pulled in transitively via -llz4 on distros that
# link liblz4 against a shared libxxhash instead of embedding it). Without
# hidden visibility, ELF symbol interposition can silently redirect this
# library's own XXH32_update/XXH32_digest calls to the system library's
# incompatible internal state layout, corrupting the LZ4 frame checksum while
# leaving the actual decompressed bytes untouched -- surfacing as a spurious
# "ROSLZ4_DATA_ERROR: malformed data to decompress" on bags that are not
# actually corrupt. Hiding this target's symbols keeps its internal calls
# bound to its own definitions regardless of what else is loaded in-process.
set_target_properties(rosbag PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden)

target_include_directories(rosbag SYSTEM PUBLIC
    include
    console_bridge/include
    roscpp_core/cpp_common/include
    roscpp_core/rostime/include
    roscpp_core/roscpp_serialization/include
    roscpp_core/roscpp_traits/include
    ros_comm/utilities/roslz4/include
    ros_comm/tools/rosbag_storage/include)

target_link_libraries(rosbag PUBLIC ${Boost_LIBRARIES} ${BZIP2_LIBRARIES} ${lz4_LIBRARIES})

