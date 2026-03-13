# Windows Environment Setup (OpenGL + FreeGLUT + GLM)

## Prerequisites

- Visual Studio 2022 with C++ Desktop workload
- CMake 3.20+
- Git
- vcpkg (recommended package manager)

## 1) Install dependencies with vcpkg

```powershell
git clone https://github.com/microsoft/vcpkg "$env:USERPROFILE\\vcpkg"
& "$env:USERPROFILE\\vcpkg\\bootstrap-vcpkg.bat"
$env:VCPKG_ROOT = "$env:USERPROFILE\\vcpkg"
& "$env:VCPKG_ROOT\\vcpkg.exe" install freeglut:x64-windows glm:x64-windows
```

Optional persistence:

```powershell
setx VCPKG_ROOT "$env:USERPROFILE\\vcpkg"
```

## 2) Configure project

From the repository root:

```powershell
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT\scripts\buildsystems\vcpkg.cmake"
```

## 3) Build project

```powershell
cmake --build build --config Release
```

## 4) Run

```powershell
.\build\Release\civitas_x.exe
```

## Troubleshooting

- If `GL/freeglut.h` is not found, check that `freeglut` was installed for the same architecture (`x64-windows`).
- If runtime fails with missing `freeglut.dll`, copy it next to the executable or ensure PATH includes the vcpkg installed bin folder.
- If CMake cannot find `glm`, rerun configure with the correct `CMAKE_TOOLCHAIN_FILE` path.
