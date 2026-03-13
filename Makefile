# Civitas-X convenience targets for GNU Make on Windows
# Usage examples:
#   make build
#   make run
#   make dev

VSDEVCMD := C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\Tools\VsDevCmd.bat
BUILD_DIR := build-nmake
SHELL := cmd
.SHELLFLAGS := /d /c

.PHONY: help configure build run run-bg dev dev-bg clean

help:
	@echo Available targets:
	@echo   make configure  - configure CMake with NMake + vcpkg toolchain
	@echo   make build      - build civitas_x
	@echo   make run        - run civitas_x
	@echo   make run-bg     - run civitas_x in a new window (non-blocking)
	@echo   make dev        - build then run
	@echo   make dev-bg     - build then run in background
	@echo   make clean      - remove $(BUILD_DIR)

configure:
	call "$(VSDEVCMD)" -arch=x64 -host_arch=x64 && cmake -S . -B $(BUILD_DIR) -G "NMake Makefiles" -DCMAKE_TOOLCHAIN_FILE=%USERPROFILE%\vcpkg\scripts\buildsystems\vcpkg.cmake

build: configure
	call "$(VSDEVCMD)" -arch=x64 -host_arch=x64 && cmake --build $(BUILD_DIR)

run:
	set "PATH=%CD%\$(BUILD_DIR)\vcpkg_installed\x64-windows\bin;%PATH%" && .\$(BUILD_DIR)\civitas_x.exe

run-bg:
	set "PATH=%CD%\$(BUILD_DIR)\vcpkg_installed\x64-windows\bin;%PATH%" && start "" .\$(BUILD_DIR)\civitas_x.exe

dev: build run

dev-bg: build run-bg

clean:
	@if exist "$(BUILD_DIR)" rmdir /s /q "$(BUILD_DIR)"
