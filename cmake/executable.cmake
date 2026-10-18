
# 编译可执行文件
set(SUFFIX ".elf")

add_executable(${PROJECT_NAME} ${SOURCES} ${LINKER_SCRIPT})

set_target_properties(${PROJECT_NAME} PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib_obj"
  SUFFIX "${SUFFIX}"
  LINKER_LANGUAGE C
)

# 链接静态库
target_link_libraries(
  ${PROJECT_NAME}
  -Wl,--start-group
  gcc
  m     # 用到了三角函数（svc_foc 浮点路径的 sinf/cosf），链接数学库 libm.a
  c
  -Wl,--whole-archive
  -Wl,--no-whole-archive
  -Wl,--end-group
  # -Wl,-u,_printf_float
  -Wl,-lIQmath_RV32     # IQmath定点运算浮点
)


# Build target
add_custom_command(
  TARGET ${PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}${SUFFIX}
          ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex DEPENDS ${PROJECT_NAME}
  COMMENT "Generating .hex file ...")

# Erase and Flash Program
set(OPENOCD_EXE "$ENV{OPENOCD_RISCV}/bin/openocd.exe")
set(OPENOCD_CFG "$ENV{OPENOCD_RISCV}/bin/wch-riscv.cfg")
set(HEX_FILE "${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex")

# set(CMD_ERASE   "-c init -c halt -c \"flash erase_sector 0 last\" -c exit")
# set(CMD_PROGRAM "-c init -c halt -c \"program \\\"${HEX_FILE}\\\" verify\" -c exit")
# set(CMD_RESET   "-c init -c halt -c wlink_reset_resume -c exit")

# add_custom_command(OUTPUT erase_sector   COMMAND ${OPENOCD_EXE} -f ${OPENOCD_CFG} ${CMD_ERASE})
# add_custom_command(OUTPUT program_image  COMMAND ${OPENOCD_EXE} -f ${OPENOCD_CFG} ${CMD_PROGRAM})
# add_custom_command(OUTPUT wlink_resume   COMMAND ${OPENOCD_EXE} -f ${OPENOCD_CFG} ${CMD_RESET})

# add_custom_target(flash DEPENDS erase_sector program_image wlink_resume) 

# Generate .bat for only programmer
add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    # 调用 CMake 的脚本模式 (-P) 来执行文件写入
    COMMAND ${CMAKE_COMMAND} 
        -DOPENOCD_EXE="${OPENOCD_EXE}" 
        -DOPENOCD_CFG="${OPENOCD_CFG}"
        -DHEX_FILE="${HEX_FILE}"   
        -DBUILD_DIR="${CMAKE_BINARY_DIR}" 
        -P "${CMAKE_SOURCE_DIR}/cmake/generate_flash.cmake"
    COMMENT "Generating flash.bat script after build..."
)


# add_custom_target(flash_only
#     COMMAND "${CMAKE_BINARY_DIR}/flash.bat"
#     DEPENDS "${CMAKE_BINARY_DIR}/flash.bat" ${PROJECT_NAME}
#     WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
#     COMMENT "Flashing ${PROJECT_NAME} via OpenOCD..."
# )

# add_custom_command(
#   OUTPUT erase_sector
#   COMMAND
#     # ${CMAKE_SOURCE_DIR}/OpenOCD/bin/openocd -f ${CMAKE_SOURCE_DIR}/OpenOCD/bin/wch-riscv.cfg -c
#     # init -c halt -c "flash erase_sector wch_riscv 0 last " -c exit)

#     $ENV{OPENOCD_RISCV}/bin/openocd -f $ENV{OPENOCD_RISCV}/bin/wch-riscv.cfg -c
#     init -c halt -c "flash erase_sector wch_riscv 0 last " -c exit)    

# add_custom_command(
#   OUTPUT program_image
#   COMMAND
#     # ${CMAKE_SOURCE_DIR}/OpenOCD/bin/openocd -f ${CMAKE_SOURCE_DIR}/OpenOCD/bin/wch-riscv.cfg -c
#     # init -c halt -c "program ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex " -c
#     # "verify_image ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex " -c exit)

#     $ENV{OPENOCD_RISCV}/bin/openocd -f $ENV{OPENOCD_RISCV}/bin/wch-riscv.cfg -c
#     init -c halt -c "program ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex " -c
#     "verify_image ${CMAKE_BINARY_DIR}/lib_obj/${PROJECT_NAME}.hex " -c exit)    

# # use wch-link reset MCU
# add_custom_command(
#   OUTPUT wlink_resume
#   COMMAND
#     # ${CMAKE_SOURCE_DIR}/OpenOCD/bin/openocd -f ${CMAKE_SOURCE_DIR}/OpenOCD/bin/wch-riscv.cfg -c
#     # init -c halt -c wlink_reset_resume -c exit)    
#     $ENV{OPENOCD_RISCV}/bin/openocd -f $ENV{OPENOCD_RISCV}/bin/wch-riscv.cfg -c
#     init -c halt -c wlink_reset_resume -c exit)     

# add_custom_target(flash DEPENDS erase_sector program_image wlink_resume) 

message("")
message("Project: ${PROJECT_NAME}")
message("  LIST_FILE=${CMAKE_PARENT_LIST_FILE}")
message("  TOOLCHAIN=${TOOLCHAIN}")
message("  GCC_SUFFIX=${GCC_SUFFIX}")
message("")
message("  CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
message("  CMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
message("  CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
message("  CMAKE_ASM_COMPILER=${CMAKE_ASM_COMPILER}")
message("  CMAKE_LINKER=${CMAKE_LINKER}")
message("  CMAKE_OBJCOPY=${CMAKE_OBJCOPY}")
message("  CMAKE_OBJDUMP=${CMAKE_OBJDUMP}")
message("  CMAKE_RANLIB=${CMAKE_RANLIB}")
message("  CMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}")
message("")
message("  CMAKE_C_FLAGS=${CMAKE_C_FLAGS}")
message("  CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
message("  CMAKE_ASM_FLAGS=${CMAKE_ASM_FLAGS}")
message("  LDFLAGS=${LDFLAGS}")
message("  CMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}")
message("Makefile created.")
message("")
