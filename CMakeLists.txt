
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
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# 构建期选择 FOC 算术实现；应用层控制参数目前仍按 Q15 编码。
set(FOC_ARITHMETIC "Q15" CACHE STRING "FOC arithmetic implementation (Q15 or FLOAT)")   # FOC_ARITHMETIC默认为Q15
set_property(CACHE FOC_ARITHMETIC PROPERTY STRINGS Q15 FLOAT)
string(TOUPPER "${FOC_ARITHMETIC}" FOC_ARITHMETIC_NORMALIZED)
if(FOC_ARITHMETIC_NORMALIZED STREQUAL "Q15")
  set(FOC_COMPILE_DEFINITION FOC_USE_Q15=1)
elseif(FOC_ARITHMETIC_NORMALIZED STREQUAL "FLOAT")
  set(FOC_COMPILE_DEFINITION FOC_USE_FLOAT=1)
  message(WARNING "FOC service builds with float; app_foc_control remains Q15-oriented")
else()
  message(FATAL_ERROR "FOC_ARITHMETIC must be Q15 or FLOAT; got '${FOC_ARITHMETIC}'")
endif()

# 编译等级
add_compile_options(-Os)

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

# 选择添加部分sdk外设驱动部分.c文件
set(CH32_STD_PERIPH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/code_project/third_party/wch/ch32v20x/stdperiph/src")
set(CH32_STD_PERIPH_SOURCES
  "${CH32_STD_PERIPH_DIR}/ch32v20x_adc.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_dma.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_gpio.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_opa.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_rcc.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_tim.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_spi.c"    
  "${CH32_STD_PERIPH_DIR}/ch32v20x_usart.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_it.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_misc.c"
  "${CH32_STD_PERIPH_DIR}/ch32v20x_flash.c"  
)

# 源码文件
file(GLOB_RECURSE SOURCES 

  # third_party
  "code_project/third_party/wch/ch32v20x/core/*.c"
  "code_project/third_party/wch/ch32v20x/system/*.c"
  "code_project/third_party/wch/ch32v20x/startup/*.S"
  # "code_project/third_party/wch/ch32v20x/stdperiph/src/*.c"
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
# platform
list(APPEND SOURCES ${CH32_STD_PERIPH_SOURCES})


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
