# Architecture Overview

## Runtime Layers

1. Render Layer
- Draws city roads, waypoints, and active autonomous cars.
- Current implementation uses classic OpenGL immediate mode for simplicity.

2. Simulation Layer
- Owns world state and updates autonomous car agents each frame.
- Handles movement, target selection, and basic battery usage.

3. Agent/Economy Layer (planned)
- NPC daily schedules and earning points from work.
- Rental interactions between NPCs and cars.
- Charging/fuel costs and city economy feedback loops.

## Core Modules

- `App`: window lifecycle and frame loop
- `World`: autonomous car simulation state
- `Renderer2D`: OpenGL drawing code

## Future Growth Path

- Add `NpcAgent` and schedule state machine (`sleep`, `commute`, `work`, `eat`, `leisure`)
- Add `TripRequest` and dispatch system
- Add economy ledger for points flow and service pricing
- Add spatial partitioning for larger city maps
