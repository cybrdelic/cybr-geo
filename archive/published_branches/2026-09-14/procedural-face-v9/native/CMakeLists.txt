cmake_minimum_required(VERSION 3.18)
project(mechanism_pathtrace LANGUAGES CXX)
find_package(OpenMP REQUIRED)

add_executable(mechanism_pathtrace pathtrace.cpp)
target_compile_features(mechanism_pathtrace PRIVATE cxx_std_17)
target_compile_options(mechanism_pathtrace PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-O3>)
target_link_libraries(mechanism_pathtrace PRIVATE OpenMP::OpenMP_CXX)

add_executable(mechanism_photoreal photoreal.cpp)
target_compile_features(mechanism_photoreal PRIVATE cxx_std_17)
target_compile_options(mechanism_photoreal PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-O3>)
target_link_libraries(mechanism_photoreal PRIVATE OpenMP::OpenMP_CXX)
