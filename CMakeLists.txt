
# 调用构建指令时添加工程名PROJ选项( ... -DPROJ=PROJ)
if(NOT PROJ)
  set(PROJ "empty")
  message(
    STATUS
      "PROJ not set, use ${PROJ} as PROJ. Also, you can set it manually. e.g. -DPROJ=hello_world"
  )
else()
  message("PROJ = ${PROJ}")
endif()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
cmake_minimum_required(VERSION 3.20)

# riscv架构设置
set(MARCH rv32imac)
set(MABI ilp32)

# 工具链设置
include(./cmake/toolchain.cmake)

# 项目设置
project(${PROJ} C CXX ASM)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)

# 编译参数 一般不用改
add_compile_options(
  -march=${MARCH} -mabi=${MABI}
  -mcmodel=medany 
  -msmall-data-limit=8 
  -mno-save-restore
)
add_compile_options(
  -fmessage-length=0 
  -fsigned-char 
  -ffunction-sections 
  -fdata-sections 
  -fno-common

  -g3
  -gdwarf-4
  -fno-omit-frame-pointer
)

# 编译等级
add_compile_options(-O0)

# 编译信息等级
add_compile_options(-Wall)

# 头文件路径
include_directories(
  
  # third_party
  code_project/third_party/wch/ch32v20x/core
  code_project/third_party/wch/ch32v20x/stdperiph/inc
  code_project/third_party/wch/ch32v20x/system
  code_project/third_party/wch/ch32v20x/iqmath
  # platform
  code_project/platform/ch32v203/inc
  # hal
  code_project/hal/inc
  # bsp
  code_project/bsp/inc
  # service
  code_project/service/inc    
  # app
  code_project/app/inc

)

# 宏定义
set(CMAKE_BUILD_TYPE Debug)   #禁止编译器调试优化,添加-g选项
# add_definitions(-DDEBUG=1)
# IQmath 通用 Q 格式库的全局 Q 设定（库头 ch32v20x/iqmath/IQmath_RV32.h 用 #ifndef 保护，
# 故在此统一指定：platform_ch32v203.h 里的 PLAT_IQ_* 封装即按此格式解释）
add_definitions(-DGLOBAL_Q=15)

# 源码文件
file(GLOB_RECURSE SOURCES 

  # third_party
  "code_project/third_party/wch/ch32v20x/core/*.c"
  "code_project/third_party/wch/ch32v20x/system/*.c"
  "code_project/third_party/wch/ch32v20x/startup/*.S"
  "code_project/third_party/wch/ch32v20x/stdperiph/src/*.c"
  "code_project/third_party/wch/ch32v20x/iqmath/*.a"

  # platform
  "code_project/platform/ch32v203/src/*.c"
  # hal
  "code_project/hal/src/*.c"
  # bsp
  "code_project/bsp/src/*.c"
  # service
  "code_project/service/src/*.c"    
  # app
  "code_project/app/src/*.c"

)

# file(GLOB_RECURSE CORE_SOURCES 

  # # Core
  # "components/cmsis/core_riscv.c" 
  # "components/startup/startup_ch32v20x_D6.S"

# )

# 链接参数
set(LINKER_SCRIPT  ${CMAKE_SOURCE_DIR}/code_project/third_party/wch/ch32v20x/linker/Link.ld)
add_link_options(
    -march=${MARCH} -mabi=${MABI}
    -nostartfiles 
    -Xlinker --gc-sections  
    -Wl,--print-memory-usage
    -Wl,-Map,./lib_obj/${PROJECT_NAME}.map 
    --specs=nano.specs 
    --specs=nosys.specs
    # -Wl,--no-warn-rwx-segments #工具链riscv-none-embed(8.2.0)不兼容
)
add_link_options(-T ${LINKER_SCRIPT})

# build and program
include(./cmake/executable.cmake)   
