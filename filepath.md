# Civitas-X

Civitas-X is a 2D autonomous city simulation written in C++20 with OpenGL + FreeGLUT.

The project simulates:
- autonomous cars moving on a road graph with pathfinding
- NPC daily life cycles (home -> work -> park -> home)
- ride-hailing behavior where NPCs request nearby free cars for long trips
- traffic-light-aware driving and simple car-following safety logic

This README is written for lab/project defense. It explains both architecture and implemented logic with exact code references so you can verify each claim directly.

## 1) Tech Stack

- Language: C++20 ([CMakeLists.txt:4](CMakeLists.txt#L4))
- Rendering: OpenGL + FreeGLUT ([CMakeLists.txt:43](CMakeLists.txt#L43), [CMakeLists.txt:44](CMakeLists.txt#L44))
- Math: GLM ([CMakeLists.txt:45](CMakeLists.txt#L45), [CMakeLists.txt:63](CMakeLists.txt#L63))
- Build System: CMake executable target `civitas_x` ([CMakeLists.txt:15](CMakeLists.txt#L15))
- Windows audio (intro music): `winmm` ([CMakeLists.txt:59](CMakeLists.txt#L59))

## 2) How To Build And Run (Windows)

### Dependencies (recommended via vcpkg)

```powershell
git clone https://github.com/microsoft/vcpkg "$env:USERPROFILE\vcpkg"
& "$env:USERPROFILE\vcpkg\bootstrap-vcpkg.bat"
& "$env:USERPROFILE\vcpkg\vcpkg.exe" install freeglut:x64-windows glm:x64-windows
```

### Configure + Build

```powershell
cmd /d /c 'call "%ProgramFiles(x86)%\Microsoft Visual Studio\18\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 && cmake -S . -B build-nmake -G "NMake Makefiles" -DCMAKE_TOOLCHAIN_FILE=%USERPROFILE%\vcpkg\scripts\buildsystems\vcpkg.cmake && cmake --build build-nmake'
```

### Run

```powershell
$env:PATH = "$PWD\build-nmake\vcpkg_installed\x64-windows\bin;$env:PATH"
.\build-nmake\civitas_x.exe
```

## 3) Project Structure (What Each Part Does)

- Entry + app lifecycle
  - [src/main.cpp](src/main.cpp)
  - [src/app.cpp](src/app.cpp)
  - [include/civitasx/app.hpp](include/civitasx/app.hpp)
- Rendering and live simulation update
  - [src/engine/renderer.cpp](src/engine/renderer.cpp)
  - [src/engine/renderer.h](src/engine/renderer.h)
- Input and camera navigation
  - [src/engine/input.cpp](src/engine/input.cpp)
  - [src/engine/input.h](src/engine/input.h)
- Agents
  - [src/agents/car_agent.cpp](src/agents/car_agent.cpp)
  - [src/agents/npc_agent.cpp](src/agents/npc_agent.cpp)
- AI/pathfinding
  - [src/ai/pathfinding.cpp](src/ai/pathfinding.cpp)
  - [src/ai/behavior_system.cpp](src/ai/behavior_system.cpp)
  - [src/ai/decision_logic.cpp](src/ai/decision_logic.cpp)
- Systems
  - [src/systems/traffic_system.cpp](src/systems/traffic_system.cpp)
  - [src/systems/economy_system.cpp](src/systems/economy_system.cpp)
  - [src/systems/simulation_manager.cpp](src/systems/simulation_manager.cpp)
- World/map
  - [src/world/city_map.cpp](src/world/city_map.cpp)
  - [src/world/road_network.cpp](src/world/road_network.cpp)
  - [src/world/tile_system.cpp](src/world/tile_system.cpp)
- Graphics algorithms for lab work
  - [src/graphics/algorithms.cpp](src/graphics/algorithms.cpp)
  - [src/graphics/shape_drawer.cpp](src/graphics/shape_drawer.cpp)

## 4) Runtime Flow (Actual Execution Order)

1. Program starts in `main`, creates `App`, calls `initialize`, then `run`.
   - [src/main.cpp:3](src/main.cpp#L3), [src/main.cpp:6](src/main.cpp#L6), [src/main.cpp:11](src/main.cpp#L11)
2. `App::initialize` sets up GLUT window/callbacks/timer and starts intro state.
   - [src/app.cpp:182](src/app.cpp#L182)
3. `App::onDisplay` runs every frame.
   - If intro not finished: animate loading screen and wait for Enter.
   - Else: call renderer.
   - [src/app.cpp:245](src/app.cpp#L245)
4. `Renderer::render` performs fixed-step simulation updates and draws the scene.
   - [src/engine/renderer.cpp:2469](src/engine/renderer.cpp#L2469)

## 5) Controls

From input handling in [src/engine/input.cpp](src/engine/input.cpp):

- `W/A/S/D` and arrow keys: pan camera ([src/engine/input.cpp:149](src/engine/input.cpp#L149))
- Mouse move near screen edges: pan camera ([src/engine/input.cpp:319](src/engine/input.cpp#L319))
- Mouse wheel or buttons `Z` / `X`: zoom in/out ([src/engine/input.cpp:256](src/engine/input.cpp#L256), [src/engine/input.cpp:294](src/engine/input.cpp#L294))
- `R`: reset camera ([src/engine/input.cpp:95](src/engine/input.cpp#L95))
- `Q`: pause navigation input handling ([src/engine/input.cpp:95](src/engine/input.cpp#L95))
- `Enter`: start simulation from intro screen ([src/engine/input.cpp:95](src/engine/input.cpp#L95), [src/app.cpp:257](src/app.cpp#L257))
- Left click: select NPC for inspector panel ([src/engine/input.cpp:433](src/engine/input.cpp#L433), [src/engine/renderer.cpp:2529](src/engine/renderer.cpp#L2529))

## 6) Core Implemented Logic (With Code Verification)

### 6.1 App and Intro Screen Logic

- Intro loading duration constant `1.35s` ([src/app.cpp:24](src/app.cpp#L24))
- Intro progress uses easing (smooth non-linear loading feel) and is clamped to `[0, 1]` ([src/app.cpp:252](src/app.cpp#L252))
- Simulation only starts when loading is complete and Enter is pressed ([src/app.cpp:257](src/app.cpp#L257))
- Intro music starts/stops on Windows via `PlaySoundA` ([src/app.cpp:272](src/app.cpp#L272), [src/app.cpp:283](src/app.cpp#L283))
- Start screen draws animated car, NPC, moving lane marks, progress bar and prompt ([src/app.cpp:304](src/app.cpp#L304))

### 6.2 City Map and Tile Semantics

- Default map is initialized as a repeated 10x10 layout over active 20x20 grid ([src/world/city_map.cpp:9](src/world/city_map.cpp#L9), [src/world/city_map.cpp:23](src/world/city_map.cpp#L23))
- Tile decoding:
  - `1 -> Road`
  - `2/4/5 -> Building`
  - `3 -> Park`
  - else `Empty`
  - ([src/world/city_map.cpp:60](src/world/city_map.cpp#L60))
- Raw tile value is preserved for role-specific placement logic (home/office) ([src/world/city_map.cpp:83](src/world/city_map.cpp#L83))

### 6.3 Road Graph + Pathfinding (A*)

- Road tiles are converted into graph nodes (`nodeCenters`, adjacency list) ([src/ai/pathfinding.cpp:52](src/ai/pathfinding.cpp#L52))
- Random distinct goal node selection for roaming behavior ([src/ai/pathfinding.cpp:123](src/ai/pathfinding.cpp#L123))
- A* pathfinding with:
  - `gScore`, `fScore`, `cameFrom`
  - Euclidean heuristic (`nodeDistance`)
  - path reconstruction by backtracking
  - ([src/ai/pathfinding.cpp:144](src/ai/pathfinding.cpp#L144))

### 6.4 Car Agent Logic

- Car defaults: id, spawn, speed, battery, wallet, states ([src/agents/car_agent.cpp:9](src/agents/car_agent.cpp#L9))
- Rental/fueling update:
  - rented car consumes battery by traveled distance
  - earns money per second while rented
  - low battery triggers fueling mode
  - fueling recharges battery to full
  - ([src/agents/car_agent.cpp:27](src/agents/car_agent.cpp#L27))

### 6.5 NPC Agent Logic

- NPC defaults include home/work/food targets and state fields ([src/agents/npc_agent.cpp:12](src/agents/npc_agent.cpp#L12))
- Standalone NPC update function supports far-destination rental decision and walking behavior ([src/agents/npc_agent.cpp:33](src/agents/npc_agent.cpp#L33))

### 6.6 Live Simulation in Renderer (Main Active Logic)

This is the primary simulation path used by the app.

- Runtime state containers for cars/NPCs and road graph are defined in renderer file scope ([src/engine/renderer.cpp:70](src/engine/renderer.cpp#L70))
- Car population is initialized from road graph nodes with random speed and start node ([src/engine/renderer.cpp:539](src/engine/renderer.cpp#L539))
- NPC population is initialized from map role spots (home/office/park) ([src/engine/renderer.cpp:951](src/engine/renderer.cpp#L951))
- Ride-request manager assigns nearest free non-fueling car to waiting NPC ([src/engine/renderer.cpp:1062](src/engine/renderer.cpp#L1062))
- Car update pipeline includes:
  - service route recomputation
  - state transitions (`Assigned -> GoToPickup -> WaitForNpc -> Transporting -> Free`)
  - intersection signal reaction (red/yellow speed control)
  - lane-based following distance control
  - acceleration/braking smoothing
  - ([src/engine/renderer.cpp:641](src/engine/renderer.cpp#L641))
- NPC update pipeline includes:
  - dwell timer
  - destination decision by routine stage
  - request-car for far trips
  - walking for short trips
  - pay/earn at arrival and cycle progression
  - ([src/engine/renderer.cpp:1130](src/engine/renderer.cpp#L1130))
- Fixed-step simulation loop (`1/120s`) to stabilize updates regardless of frame rate ([src/engine/renderer.cpp:2489](src/engine/renderer.cpp#L2489), [src/engine/renderer.cpp:2504](src/engine/renderer.cpp#L2504))
- Ordered update sequence each step:
  1) ride handler
  2) cars
  3) NPCs
  - ([src/engine/renderer.cpp:2508](src/engine/renderer.cpp#L2508))

### 6.7 Traffic Signal System

- Determines if a tile is a 4-way signalized intersection ([src/systems/traffic_system.cpp:84](src/systems/traffic_system.cpp#L84))
- Computes adaptive green split from local road corridor strength ([src/systems/traffic_system.cpp:98](src/systems/traffic_system.cpp#L98))
- Uses cycle phases: horizontal green -> yellow -> all-red -> vertical green -> yellow -> all-red ([src/systems/traffic_system.cpp:98](src/systems/traffic_system.cpp#L98))
- Renderer draws dynamic signal heads over road intersections ([src/engine/renderer.cpp:2192](src/engine/renderer.cpp#L2192))

### 6.8 Economy and Operational Cost Logic

- Operational energy drain and wallet penalty on battery depletion ([src/systems/economy_system.cpp:11](src/systems/economy_system.cpp#L11))
- Car rental earnings and recharge behavior handled in car agent update ([src/agents/car_agent.cpp:27](src/agents/car_agent.cpp#L27))

### 6.9 Graphics Algorithms Implemented (Lab-Relevant)

- Bresenham line algorithm implementation ([src/graphics/algorithms.cpp:13](src/graphics/algorithms.cpp#L13))
- Rectangle fill by scanline using Bresenham per row ([src/graphics/algorithms.cpp:51](src/graphics/algorithms.cpp#L51))
- Midpoint circle algorithm to generate boundary points and filled fan vertices ([src/graphics/algorithms.cpp:101](src/graphics/algorithms.cpp#L101))
- Filled circle draw helper based on generated fan vertices ([src/graphics/shape_drawer.cpp:13](src/graphics/shape_drawer.cpp#L13))

### 6.10 Rendering Pipeline Logic

- Static map tiles are compiled into an OpenGL display list cache for performance ([src/engine/renderer.cpp:2110](src/engine/renderer.cpp#L2110))
- Dynamic components (traffic lights, cars, NPCs, inspector overlay) are redrawn each frame ([src/engine/renderer.cpp:2192](src/engine/renderer.cpp#L2192), [src/engine/renderer.cpp:1312](src/engine/renderer.cpp#L1312), [src/engine/renderer.cpp:1487](src/engine/renderer.cpp#L1487), [src/engine/renderer.cpp:463](src/engine/renderer.cpp#L463))
- Camera projection uses panning + smooth zoom with clamped world bounds from input subsystem ([src/engine/input.cpp:319](src/engine/input.cpp#L319), [src/engine/input.cpp:424](src/engine/input.cpp#L424))

## 7) Secondary/Alternative Simulation Module

The file [src/systems/simulation_manager.cpp](src/systems/simulation_manager.cpp) contains another complete simulation manager implementation:

- Initialization and entity spawning ([src/systems/simulation_manager.cpp:22](src/systems/simulation_manager.cpp#L22))
- Update loop with ride assignment, car/NPC progression, economy hooks ([src/systems/simulation_manager.cpp:96](src/systems/simulation_manager.cpp#L96))

Important verification note:
- In current app flow, runtime simulation is driven by renderer path (`Renderer::render`) rather than by direct `SimulationManager` usage.
- Reference for active call path: [src/main.cpp:3](src/main.cpp#L3) -> [src/app.cpp:245](src/app.cpp#L245) -> [src/engine/renderer.cpp:2469](src/engine/renderer.cpp#L2469)


## 8) Current Status

- Implemented and verifiable:
  - autonomous car movement with route planning
  - NPC routine and ride-hailing interaction
  - traffic-light logic and visual signal rendering
  - economy/energy effects
  - custom raster-style graphics helper algorithms
- Next expandable areas:
  - richer economic market rules
  - congestion metrics and analytics
  - serialization/replay of simulation states
