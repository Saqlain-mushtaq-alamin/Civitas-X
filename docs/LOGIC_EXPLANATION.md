# Civitas-X Logic Explanation

This document explains the implemented simulation logic in a simple, presentation-friendly way.

## 1) High-Level Runtime Flow

1. Program starts from [src/main.cpp](../src/main.cpp#L3).
2. App initializes window, callbacks, timer, and intro state in [src/app.cpp](../src/app.cpp#L182).
3. After intro, each frame goes through [src/engine/renderer.cpp](../src/engine/renderer.cpp#L2469).
4. Simulation updates run in fixed steps for stable behavior:
   - fixed step size at [src/engine/renderer.cpp](../src/engine/renderer.cpp#L2489)
   - step loop at [src/engine/renderer.cpp](../src/engine/renderer.cpp#L2504)

Why fixed step matters:
- It keeps movement and state transitions consistent even if FPS changes.

## 2) City and Road Representation

- The tile map is created in [src/world/city_map.cpp](../src/world/city_map.cpp#L9).
- Base pattern layout is declared at [src/world/city_map.cpp](../src/world/city_map.cpp#L23).
- Tile meaning conversion happens in [src/world/city_map.cpp](../src/world/city_map.cpp#L60).

Road graph creation:
- Road tiles are converted to graph nodes in [src/ai/pathfinding.cpp](../src/ai/pathfinding.cpp#L52).
- Adjacency is built from 4-direction road neighbors in the same function.

## 3) Car Movement Logic

Car spawn and route setup:
- Cars are initialized in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L539).
- Every car gets a start node and speed, then receives a route.

Path planning:
- A* is implemented in [src/ai/pathfinding.cpp](../src/ai/pathfinding.cpp#L144).
- Random goal node selection is in [src/ai/pathfinding.cpp](../src/ai/pathfinding.cpp#L123).

Per-step movement update:
- Main car update is in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L641).
- Movement computes direction, angle, speed scaling, and step distance.
- Car states are handled in this flow:
  - Assigned -> GoToPickup -> WaitForNpc -> Transporting -> Free

Traffic-aware driving:
- Signalized intersection check in [src/systems/traffic_system.cpp](../src/systems/traffic_system.cpp#L84).
- Signal timing and color logic in [src/systems/traffic_system.cpp](../src/systems/traffic_system.cpp#L98).
- Cars slow/stop for red or yellow during update in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L826).

Car-following safety:
- Lead vehicle distance check and speed reduction are in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L859).
- This prevents overlap and reduces unrealistic tailgating.

## 4) NPC Logic

NPC creation:
- NPC initialization is in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L951).
- Home/work/park spots are selected from map tile roles.

NPC behavior cycle:
- Main NPC update is in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1130).
- Routine stages:
  - Stage 0: Home -> Work
  - Stage 1: Work -> Park
  - Stage 2: Park -> Home
- Stage labels are defined in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L408).

Decision logic:
- If target is near, NPC walks.
- If target is far, NPC requests a car.
- Arrival updates money and advances cycle stage.

## 5) How Car and NPC Connect (Ride-Hailing Logic)

This is the key interaction logic.

1. NPC decides destination and requests car:
   - Request state flow in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1227).
2. Ride manager processes pending requests:
   - [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1062).
3. Nearest free non-fueling car is selected and assigned:
   - assignment block in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1099).
4. Car drives to pickup location (GoToPickup).
5. When car reaches pickup and NPC is close:
   - NPC enters car and state becomes InCar in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1200).
6. Car state becomes Transporting and drives to NPC target.
7. On drop-off:
   - NPC leaves car, returns to walking/arrived flow.
   - Car returns to Free and receives next route.

## 6) Economy and Energy Logic

- Car operational battery drain and wallet penalty are in [src/systems/economy_system.cpp](../src/systems/economy_system.cpp#L11).
- Rental/fueling behavior is in [src/agents/car_agent.cpp](../src/agents/car_agent.cpp#L27).
- NPC earns/spends at destinations in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1281).

## 7) Rendering + Visual State Feedback

- Full render pipeline entry: [src/engine/renderer.cpp](../src/engine/renderer.cpp#L2469).
- NPC visual rendering by state: [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1312).
- Car visual rendering: [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1487).
- Dynamic traffic lights drawing: [src/engine/renderer.cpp](../src/engine/renderer.cpp#L2192).
- NPC inspector overlay (click-to-inspect): [src/engine/renderer.cpp](../src/engine/renderer.cpp#L463).

## 8) Example Walkthrough (One NPC Trip)

Example scenario:
1. NPC is idle, then decides next target in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1210).
2. Destination is far, so NPC switches to RequestCar then WaitingForCar in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1227).
3. Ride manager picks nearest free car in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1062).
4. Car moves with route logic in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L641).
5. At pickup, NPC enters car in [src/engine/renderer.cpp](../src/engine/renderer.cpp#L1200).
6. Car transports NPC to target, then NPC continues daily cycle.

## 9) Notes for Presentation

When presenting this project, explain logic in this order:
1. Fixed-step loop
2. Road graph + A* pathfinding
3. Car state machine
4. NPC routine state machine
5. Ride-hailing bridge between NPC and car
6. Traffic signals + following model for realism

This order helps examiners clearly see system design, AI behavior, and interaction logic.
