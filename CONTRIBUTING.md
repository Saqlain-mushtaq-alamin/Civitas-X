# Contributing

## Branching

- create short, descriptive branches (`phase1-render-loop`, `phase2-npc-agent`)
- keep pull requests focused on one concern

## Commit Style

Use conventional prefixes:
- `feat:` for new features
- `fix:` for bug fixes
- `docs:` for documentation updates
- `refactor:` for internal changes

## Code Guidelines

- target C++20
- keep modules small and clear
- avoid global mutable state
- prefer deterministic simulation logic for reproducibility

## Definition of a Good PR

- builds locally with CMake
- includes tests or validation notes
- updates docs when behavior changes
- explains simulation assumptions
