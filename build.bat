@echo off
setlocal enabledelayedexpansion

:: Get the directory where this script is running
set "PROJECT_DIR=%~dp0"

:: 🧹 CLEANUP: Clear interfering variables from other apps
set CPATH=
set C_INCLUDE_PATH=
set CPLUS_INCLUDE_PATH=
set LIBRARY_PATH=

echo ==============================================
echo   Temporal Disaster Navigator - Auto Build
echo ==============================================

:: --- 1. Check & Install Compiler (w64devkit) ---
set "COMPILER_DIR=%PROJECT_DIR%w64devkit"
set "COMPILER_ZIP=w64devkit.zip"
set "COMPILER_URL=https://github.com/skeeto/w64devkit/releases/download/v1.23.0/w64devkit-1.23.0.zip"

if not exist "%COMPILER_DIR%\bin\gcc.exe" (
    echo [INFO] Compiler not found. Downloading w64devkit...
    powershell -Command "$ProgressPreference = 'SilentlyContinue'; Invoke-WebRequest -Uri '%COMPILER_URL%' -OutFile '%COMPILER_ZIP%'"
    
    echo [INFO] Extracting compiler...
    powershell -Command "$ProgressPreference = 'SilentlyContinue'; Expand-Archive -Path '%COMPILER_ZIP%' -DestinationPath '%PROJECT_DIR%' -Force"
    
    del "%COMPILER_ZIP%"
    echo [SUCCESS] Compiler installed.
) else (
    echo [CHECK] Compiler found.
)

:: --- 2. Check & Install Raylib ---
set "RAYLIB_BASE_DIR=%PROJECT_DIR%raylib"
set "RAYLIB_DIR=%RAYLIB_BASE_DIR%\raylib-5.0_win64_mingw-w64"
set "RAYLIB_ZIP=raylib.zip"
set "RAYLIB_URL=https://github.com/raysan5/raylib/releases/download/5.0/raylib-5.0_win64_mingw-w64.zip"

if not exist "%RAYLIB_DIR%\lib\libraylib.a" (
    echo [INFO] Raylib library not found. Downloading Raylib 5.0...
    if not exist "%RAYLIB_BASE_DIR%" mkdir "%RAYLIB_BASE_DIR%"
    
    powershell -Command "$ProgressPreference = 'SilentlyContinue'; Invoke-WebRequest -Uri '%RAYLIB_URL%' -OutFile '%RAYLIB_ZIP%'"
    
    echo [INFO] Extracting Raylib...
    powershell -Command "$ProgressPreference = 'SilentlyContinue'; Expand-Archive -Path '%RAYLIB_ZIP%' -DestinationPath '%RAYLIB_BASE_DIR%' -Force"
    
    del "%RAYLIB_ZIP%"
    echo [SUCCESS] Raylib installed.
) else (
    echo [CHECK] Raylib found.
)

:: --- 3. Build Application ---
echo.
echo 🔨 Compiling Simulation...

set "PATH=%COMPILER_DIR%\bin;%PATH%"

gcc raylib_viz.c -o raylib_viz.exe ^
    -I "%RAYLIB_DIR%\include" ^
    -L "%RAYLIB_DIR%\lib" ^
    -lraylib -lopengl32 -lgdi32 -lwinmm

if %errorlevel% neq 0 (
    echo.
    echo ❌ FATAL ERROR: Compilation failed.
    echo Check for any error messages above.
    pause
    exit /b %errorlevel%
)

echo.
echo ✅ Build Successful! Starting Simulation...
timeout /t 2 >nul
start raylib_viz.exe
endlocal