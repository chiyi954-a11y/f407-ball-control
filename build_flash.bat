@echo off
setlocal
chcp 65001 >nul

rem ============================================================
rem  f407 一键编译 + 烧录（STM32CubeIDE 命令行工具链，不依赖 Keil）
rem  用法：双击本文件，或在本目录 cmd 里执行 build_flash.bat
rem ============================================================

set "IDE=C:\Users\pgrlw\OneDrive\Desktop\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins"
set "TC=%IDE%\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin"
set "MK=%IDE%\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe"
set "PRG=%IDE%\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin\STM32_Programmer_CLI.exe"

if not exist "%TC%\arm-none-eabi-gcc.exe"  goto :notool
if not exist "%MK%"                         goto :notool
if not exist "%PRG%"                        goto :notool

set "PATH=%TC%;%PATH%"
set "PROJ=%~dp0"
set "PROJ=%PROJ:~0,-1%"

echo.
echo [1/2] 编译 ...
pushd "%PROJ%\Debug"
"%MK%" -j8 all
set "RC=%ERRORLEVEL%"
popd
if not "%RC%"=="0" (
    echo.
    echo [!!] 编译失败，看上面的错误信息
    pause
    exit /b 1
)

echo.
echo [2/2] 烧录 ...
"%PRG%" -c port=SWD -w "%PROJ%\Debug\f407.elf" -v -rst
if not "%ERRORLEVEL%"=="0" (
    echo.
    echo [!!] 烧录失败：检查 ST-Link 是否插好、板子是否上电
    pause
    exit /b 1
)

echo.
echo [OK] 编译烧录完成，板子已复位运行
pause
exit /b 0

:notool
echo [!!] 找不到 STM32CubeIDE 工具链，请检查本文件里 IDE= 那一行的路径
pause
exit /b 1
