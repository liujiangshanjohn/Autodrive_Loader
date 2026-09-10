# Findabsl.cmake
# 查找 absl 库（Abseil C++ 库）
# 注意：CMake 的 find_package 默认查找 <Package>Config.cmake
# 如果没有找到，会查找 Find<Package>.cmake（MODULE 模式）
# 这里使用 MODULE 模式

# 查找库文件（支持多种架构）
set(ABSL_LIBS "")
# 尝试查找 aarch64 (ARM64) 架构
file(GLOB ABSL_LIBS "/usr/lib/aarch64-linux-gnu/libabsl_*.so.20210324")
if(NOT ABSL_LIBS)
    # 尝试查找不带版本号的库文件
    file(GLOB ABSL_LIBS "/usr/lib/aarch64-linux-gnu/libabsl_*.so")
endif()
# 如果没找到，尝试 x86_64 架构（向后兼容）
if(NOT ABSL_LIBS)
    file(GLOB ABSL_LIBS "/usr/lib/x86_64-linux-gnu/libabsl_*.so.20210324")
endif()
if(NOT ABSL_LIBS)
    file(GLOB ABSL_LIBS "/usr/lib/x86_64-linux-gnu/libabsl_*.so")
endif()

if(ABSL_LIBS)
    message(STATUS "Found absl libraries: ${ABSL_LIBS}")
    set(absl_FOUND TRUE)
    set(absl_LIBRARIES ${ABSL_LIBS})
else()
    message(WARNING "Could not find absl libraries")
    set(absl_FOUND FALSE)
endif()

# 设置变量供 CMake 使用
if(absl_FOUND)
    set(absl_FOUND TRUE CACHE BOOL "absl library found")
    mark_as_advanced(absl_FOUND absl_LIBRARIES)
endif()
