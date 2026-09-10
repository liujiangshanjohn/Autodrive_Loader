# abslConfig.cmake
# 为 absl 库提供 CMake 配置文件

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
    
    # 创建导入目标（如果需要）
    if(NOT TARGET absl::absl)
        add_library(absl::absl INTERFACE IMPORTED)
        set_target_properties(absl::absl PROPERTIES
            INTERFACE_LINK_LIBRARIES "${ABSL_LIBS}"
        )
    endif()
else()
    message(WARNING "Could not find absl libraries")
    set(absl_FOUND FALSE)
endif()

# 设置变量
set(absl_FOUND ${absl_FOUND} CACHE BOOL "absl library found")
set(absl_LIBRARIES ${absl_LIBRARIES} CACHE STRING "absl libraries")
mark_as_advanced(absl_FOUND absl_LIBRARIES)
