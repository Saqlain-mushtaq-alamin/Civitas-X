# Civitas-X

Autonomous City Simulation in 2D using OpenGL + FreeGLUT.

## Vision

**What would a city look like if every car was autonomous?**

Civitas-X models a self-organizing urban ecosystem where:
- all cars are autonomous AI agents
- NPCs live daily routines (sleep, work, travel)
- NPCs earn points from work
- NPCs spend points to rent autonomous cars
- cars spend points/credits on charging or fuel

## Build Plan

### Phase 1 - Core (must finish)
- open world map (2D top-down)
- autonomous driverless car agents
- continuous movement and route choices
- simple rendering + simulation loop

### Phase 2 - Expansion
- NPC agents with schedules
- commute behavior (home <-> work)
- NPC-car interaction (renting and trips)

### Phase 3 - Simulation Layer
- points economy (income, spending)
- car energy model (battery/fuel)
- city-wide emergent behavior metrics

## Tech Stack
- C++20
- OpenGL (2D rendering)
- FreeGLUT (window + input + timing)
- GLM (math)
- CMake

## Quick Start (Windows)

### 1) Install dependencies
Option A (recommended): use `vcpkg`

```powershell
git clone https://github.com/microsoft/vcpkg "$env:USERPROFILE\\vcpkg"
& "$env:USERPROFILE\\vcpkg\\bootstrap-vcpkg.bat"
$env:VCPKG_ROOT = "$env:USERPROFILE\\vcpkg"
& "$env:VCPKG_ROOT\\vcpkg.exe" install freeglut:x64-windows glm:x64-windows
```

### 2) Configure and build
From repository root:

Option A: CMake commands

```powershell
cmd /d /c 'call "%ProgramFiles(x86)%\Microsoft Visual Studio\18\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 && cmake -S . -B build-nmake -G "NMake Makefiles" -DCMAKE_TOOLCHAIN_FILE=%USERPROFILE%\vcpkg\scripts\buildsystems\vcpkg.cmake && cmake --build build-nmake'
```

Option B: Makefile shortcuts

```powershell
make build
```

### 3) Run

```powershell
$env:PATH = "$PWD\build-nmake\vcpkg_installed\x64-windows\bin;$env:PATH"
.\build-nmake\civitas_x.exe
```

Or with Makefile shortcut:

```powershell
make run
```

Run without blocking the current terminal:

```powershell
make run-bg
```

## Repository Layout

```text
Civitas-X/
  include/civitasx/       # Headers
  src/                    # Source files
  docs/                   # Design + phase documentation
  .vscode/                # VS Code tasks and recommendations
  CMakeLists.txt
  vcpkg.json
```

 