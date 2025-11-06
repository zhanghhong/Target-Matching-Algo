@echo off
echo Building Matching-Algo with CMake...

REM 创建构建目录
if not exist build mkdir build
cd build

REM 配置 CMake
cmake .. -G "MinGW Makefiles"

REM 如果 MinGW Makefiles 失败，尝试其他生成器
if %ERRORLEVEL% NEQ 0 (
    echo Trying Visual Studio generator...
    cmake ..
)

REM 编译
cmake --build . --config Release

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Build success! Executable is in: build\bin\match-raev.exe
) else (
    echo.
    echo Build failed!
    exit /b 1
)

cd ..

