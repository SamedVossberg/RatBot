@echo off
REM Build RatBot single-file executable using PyInstaller
REM
REM Prerequisites:
REM   pip install pyinstaller
REM
REM Output:
REM   dist/ratbot.exe

echo ========================================
echo Building RatBot Executable
echo ========================================
echo.

REM Check if PyInstaller is installed
python -c "import PyInstaller" 2>nul
if errorlevel 1 (
    echo ERROR: PyInstaller not found!
    echo Please install: pip install pyinstaller
    echo.
    pause
    exit /b 1
)

REM Clean previous builds
echo Cleaning previous builds...
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

REM Build executable
echo.
echo Building executable...
python -m PyInstaller ratbot_build.spec

REM Check if build succeeded
if exist dist\ratbot.exe (
    echo.
    echo ========================================
    echo Build successful!
    echo ========================================
    echo.
    echo Executable location: dist\ratbot.exe
    echo.
    echo Usage:
    echo   dist\ratbot.exe           - Auto-detect COM port
    echo   dist\ratbot.exe COM3      - Use specific COM port
    echo   dist\ratbot.exe --debug   - Enable debug logging
    echo.
) else (
    echo.
    echo ========================================
    echo Build FAILED!
    echo ========================================
    echo Please check the output above for errors.
    echo.
)

pause
