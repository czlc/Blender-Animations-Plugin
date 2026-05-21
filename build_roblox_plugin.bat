@echo off
setlocal

set "PROJECT_FILE=plugin.project.json"
set "PLUGIN_NAME=BlenderAnimations.rbxm"
set "OUTPUT_FILE=%PLUGIN_NAME%"
set "ROBLOX_PLUGIN_DIR=%LOCALAPPDATA%\Roblox\Plugins"

cd /d "%~dp0"

if not exist "%PROJECT_FILE%" (
    echo ERROR: Could not find %PROJECT_FILE%.
    exit /b 1
)

where rojo >nul 2>nul
if errorlevel 1 (
    echo ERROR: rojo was not found in PATH.
    echo Install it with Aftman, or make sure rojo.exe is available in PATH.
    exit /b 1
)

echo Building Roblox plugin...
echo Project: %PROJECT_FILE%
echo Output:  %OUTPUT_FILE%
echo.

rojo build "%PROJECT_FILE%" --output "%OUTPUT_FILE%"
if errorlevel 1 (
    echo.
    echo ERROR: rojo build failed.
    exit /b 1
)

echo.
echo Built: %OUTPUT_FILE%

if /i "%~1"=="install" (
    if not exist "%ROBLOX_PLUGIN_DIR%" (
        mkdir "%ROBLOX_PLUGIN_DIR%"
        if errorlevel 1 (
            echo ERROR: Failed to create Roblox plugin directory.
            exit /b 1
        )
    )

    copy /Y "%OUTPUT_FILE%" "%ROBLOX_PLUGIN_DIR%\%PLUGIN_NAME%" >nul
    if errorlevel 1 (
        echo ERROR: Failed to install plugin to %ROBLOX_PLUGIN_DIR%.
        exit /b 1
    )

    echo Installed: %ROBLOX_PLUGIN_DIR%\%PLUGIN_NAME%
)

echo.
echo Done.
exit /b 0
