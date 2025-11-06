cmake_minimum_required(VERSION 3.10)
project(Matching-Algo CXX)

# 设置 C++ 标准（使用 C++17 以支持 std::optional）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 源文件
set(SOURCES
    src/match-raev.cpp
)

# 创建可执行文件
add_executable(match-raev ${SOURCES})

# 设置输出名称（Windows 下会自动添加 .exe）
set_target_properties(match-raev PROPERTIES
    OUTPUT_NAME "match-raev"
)

# 编译选项
if(MSVC)
    # MSVC 编译器选项
    target_compile_options(match-raev PRIVATE /W4)
else()
    # GCC/Clang 编译器选项
    target_compile_options(match-raev PRIVATE -Wall -Wextra)
endif()

