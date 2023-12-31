cmake_minimum_required(VERSION 3.16)

project(${DEVICE_NAME} LANGUAGES C CXX ASM)

set(CMAKE_VERBOSE_MAKEFILE ON)

# Include Dialog SDK
include(${CMAKE_SOURCE_DIR}/gcc/dialog-sdk.cmake)

# Print include directories
function(print_include_directories)
    message(STATUS "Include directories: ")
    get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    foreach(dir ${dirs})
        message(STATUS "${dir}")
    endforeach()
endfunction(print_include_directories)

add_executable(${PROJECT_NAME}
    ${DIALOG_SDK_SOURCES}
    # Libraries
    Libraries/RTT/SEGGER_RTT.c
    Libraries/RTT/SEGGER_RTT_printf.c
    # Components
    components/debug/src/debug.c
    # Source
    src/user_custs_config.c
    src/user_custs1_def.c
    src/user_periph_setup.c
    src/printf_gcc.c
    src/interrupts.c
    src/user_app.c
    src/ble_handlers.c
    src/user_app.c
)

set(GLOBAL_DEBUG_OPTIONS -mthumb -mcpu=cortex-m0plus -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -flto -Wall -Werror -g3)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C,CXX,ASM>:__DA14531__>
    $<$<COMPILE_LANGUAGE:C,CXX>:GPIO_DRV_PIN_ALLOC_MON_DISABLED>
    -DDEBUG_SEGGER
)

target_compile_options(${PROJECT_NAME} PRIVATE
# C, C++, Assembly
    $<$<COMPILE_LANGUAGE:C,CXX,ASM>:${GLOBAL_DEBUG_OPTIONS}>
# C, C++
    $<$<COMPILE_LANGUAGE:C,CXX>:-include${CMAKE_SOURCE_DIR}/components/config/inc/da1458x_config_basic.h>
    $<$<COMPILE_LANGUAGE:C,CXX>:-include${CMAKE_SOURCE_DIR}/components/config/inc/da1458x_config_advanced.h>
    $<$<COMPILE_LANGUAGE:C,CXX>:-include${CMAKE_SOURCE_DIR}/components/config/inc/user_config.h>
# C only
    $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
    $<$<COMPILE_LANGUAGE:C>:-Wno-int-conversion>
    $<$<COMPILE_LANGUAGE:C>:-Wno-unused-variable>
    $<$<COMPILE_LANGUAGE:C>:-Wno-address>
    # -fstrict-volatile-bitfields
    # -Werror
    # -Wextra
    # -Wcast-align
    # -Wconversion
    # -Wsign-conversion
    # -Wshadow
    # -Wlogical-op
    # -Wsuggest-final-types
    # -Wsuggest-final-methods
    # -pedantic
    # -Wno-expansion-to-defined
    # -Wno-unused-parameter
    # -Wno-shadow
    # -Wno-sign-compare
# C++ only
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++11>
    $<$<COMPILE_LANGUAGE:CXX>:-fms-extensions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>
    $<$<COMPILE_LANGUAGE:CXX>:-Wsuggest-override>
# Assembly
    $<$<COMPILE_LANGUAGE:ASM>:-xassembler-with-cpp>
)

set(USER_INCLUDES
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/components/config/inc
    ${CMAKE_SOURCE_DIR}/components/debug/inc
    ${CMAKE_SOURCE_DIR}/Libraries/RTT
    # GCC includes
    # ${GCC_TOOLCHAIN_PATH}/arm-none-eabi/include/
)

target_include_directories(${PROJECT_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C,CXX>:${DIALOG_SDK_INCLUDES}>
    $<$<COMPILE_LANGUAGE:C,CXX>:${USER_INCLUDES}>
)

# Linker file needs to be preprocessed
set(LINKER_SCRIPT_IN "${PROJECT_SOURCE_DIR}/gcc/ldscript_DA14531.lds.S")
set(LINKER_SCRIPT_OUT "ldscript.lds")

# Linker script includes
set(LINKER_SCRIPT_INCLUDES
    -I ${CMAKE_SOURCE_DIR}/components/config/inc
    -I ${PROJECT_SOURCE_DIR}/gcc/
    -I ${DIALOG_SDK_PATH}/sdk/common_project_files
)

link_directories(
    ${CMAKE_SOURCE_DIR}/gcc
)

function(generate_linker_script target_name linker_script_includes linker_script_defs linker_script_input linker_script_output)
    # Check the input file
    if(NOT EXISTS ${linker_script_input})
        message(FATAL_ERROR "Linker input file not found.")
    else()
        message(STATUS "Found linker input file: ${linker_script_input}")
    endif()

    add_custom_command(OUTPUT ${linker_script_output}
                    COMMAND ${CMAKE_C_COMPILER} ${linker_script_includes} ${linker_script_defs} -E -P -c ${linker_script_input} -o ${linker_script_output}
                    DEPENDS ${linker_script_input}
                    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
                    COMMENT "Generating linker script: ${linker_script_output}"
                    VERBATIM)

    add_custom_target(linkerscript DEPENDS ${linker_script_output} VERBATIM)

    # Add linker preprocessed file as dependency
    add_dependencies(${target_name} linkerscript)
    # Add dependency on linker input file
    set_target_properties(${target_name} PROPERTIES LINK_DEPENDS ${linker_script_input})
endfunction(generate_linker_script)

generate_linker_script(${PROJECT_NAME} "${LINKER_SCRIPT_INCLUDES}" "-D__DA14531__" "${LINKER_SCRIPT_IN}" "${LINKER_SCRIPT_OUT}")

target_link_directories(${PROJECT_NAME} PRIVATE
    ${DIALOG_SDK_PATH}/sdk/gcc
    ${DIALOG_SDK_PATH}/sdk/platform/system_library/output/Keil_5/
    ${DIALOG_SDK_PATH}/sdk/common_project_files/scatterfiles/
    ${DIALOG_SDK_PATH}/sdk/common_project_files/misc/
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    :da14531.lib
)

target_link_options(${PROJECT_NAME} PRIVATE
    ${GLOBAL_DEBUG_OPTIONS}
    "-T${LINKER_SCRIPT_OUT}"
    "-Xlinker"
    "--gc-sections"
    "--specs=nano.specs"
    "--specs=nosys.specs"
    "-v"
    "-Wl,--no-wchar-size-warning" # Suppress the warning from linking Dialog's system library
    "-Wl,-Map,${PROJECT_NAME}.map" # Produce map file
)

# Post build actions

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} --format=berkeley "$<TARGET_FILE:${PROJECT_NAME}>"
    COMMENT "Print output application size"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex "$<TARGET_FILE:${PROJECT_NAME}>" ${PROJECT_NAME}.hex
    COMMENT "Convert output to hex"
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Obinary "$<TARGET_FILE:${PROJECT_NAME}>" ${PROJECT_NAME}.bin
    COMMENT "Convert output to binary"
)
