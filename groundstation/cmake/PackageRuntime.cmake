if(NOT DEFINED PROJECT_BINARY_DIR OR PROJECT_BINARY_DIR STREQUAL "")
    message(FATAL_ERROR "PROJECT_BINARY_DIR 未设置，无法生成运行包。")
endif()

if(NOT DEFINED PACKAGE_DIR OR PACKAGE_DIR STREQUAL "")
    message(FATAL_ERROR "PACKAGE_DIR 未设置，无法生成运行包。")
endif()

if(NOT DEFINED CONFIG OR CONFIG STREQUAL "")
    set(CONFIG "Release")
endif()

file(REMOVE_RECURSE "${PACKAGE_DIR}")
file(MAKE_DIRECTORY "${PACKAGE_DIR}")

set(install_args
    --install "${PROJECT_BINARY_DIR}"
    --prefix "${PACKAGE_DIR}"
    --config "${CONFIG}"
)

execute_process(
    COMMAND "${CMAKE_COMMAND}" ${install_args}
    RESULT_VARIABLE install_result
)

if(NOT install_result EQUAL 0)
    message(FATAL_ERROR "运行包安装失败，cmake --install 返回 ${install_result}。")
endif()

file(MAKE_DIRECTORY "${PACKAGE_DIR}/bin/logs")
message(STATUS "运行包已生成：${PACKAGE_DIR}")
