cmake_minimum_required(VERSION 3.16)
project(as_nmpc_controller C)

set(CMAKE_C_STANDARD 99)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Path to the vendored DAQP sources (repo_root/third_party/daqp)
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../.." ABSOLUTE)
set(DAQP_DIR "${REPO_ROOT}/third_party/daqp")

add_library(daqp_static STATIC
  ${DAQP_DIR}/src/daqp.c
  ${DAQP_DIR}/src/auxiliary.c
  ${DAQP_DIR}/src/factorization.c
  ${DAQP_DIR}/src/api.c
  ${DAQP_DIR}/src/utils.c
  ${DAQP_DIR}/src/bnb.c
  ${DAQP_DIR}/src/hierarchical.c
  ${DAQP_DIR}/src/daqp_prox.c
  ${DAQP_DIR}/src/avi.c
)
target_include_directories(daqp_static PUBLIC ${DAQP_DIR}/include)
target_compile_options(daqp_static PRIVATE -O2)

# CasADi-generated kernels: large translation units, -O1 keeps gcc happy and
# compile times sane; runtime difference vs -O2 is negligible (memory-bound).
add_library(as_kernel STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/gen/kernel.c
  ${CMAKE_CURRENT_SOURCE_DIR}/gen/kernel_full.c
)
target_compile_options(as_kernel PRIVATE -O1 -fno-var-tracking)

add_executable(as_nmpc_benchmark
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/controller.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/controller_full.c
)
target_include_directories(as_nmpc_benchmark PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/gen
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_options(as_nmpc_benchmark PRIVATE -O2 -Wall)
target_link_libraries(as_nmpc_benchmark PRIVATE as_kernel daqp_static m)
