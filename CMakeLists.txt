cmake_minimum_required(VERSION 3.20)
project(CivitasX VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(CIVITASX_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(CIVITASX_ENABLE_ASAN "Enable AddressSanitizer for diagnostics" OFF)

add_executable(civitas_x
    src/main.cpp
    src/app.cpp
    src/engine/renderer.cpp
    src/engine/camera.cpp
    src/engine/input.cpp
    src/world/city_map.cpp
    src/world/road_network.cpp
    src/world/tile_system.cpp
    src/agents/car_agent.cpp
    src/agents/npc_agent.cpp
    src/ai/pathfinding.cpp
    src/ai/behavior_system.cpp
    src/ai/decision_logic.cpp
    src/systems/traffic_system.cpp
    src/systems/economy_system.cpp
    src/systems/simulation_manager.cpp
    src/graphics/shape_drawer.cpp
    src/graphics/algorithms.cpp
    src/graphics/transformation.cpp
)

target_include_directories(civitas_x
    PRIVATE
        include
        src
)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(glm CONFIG QUIET)

if(TARGET OpenGL::GL)
    target_link_libraries(civitas_x PRIVATE OpenGL::GL)
endif()

if(TARGET GLUT::GLUT)
    target_link_libraries(civitas_x PRIVATE GLUT::GLUT)
else()
    target_include_directories(civitas_x PRIVATE ${GLUT_INCLUDE_DIRS})
    target_link_libraries(civitas_x PRIVATE ${GLUT_LIBRARIES})
endif()

if(WIN32)
    target_link_libraries(civitas_x PRIVATE winmm)
endif()

if(glm_FOUND)
    target_link_libraries(civitas_x PRIVATE glm::glm)
else()
    message(WARNING "glm not found via package manager. Add include path manually if needed.")
endif()

if(MSVC)
    target_compile_options(civitas_x PRIVATE /W4)
    if(CIVITASX_ENABLE_ASAN)
        target_compile_options(civitas_x PRIVATE /fsanitize=address /Zi)
        target_link_options(civitas_x PRIVATE /fsanitize=address)
    endif()
    if(CIVITASX_WARNINGS_AS_ERRORS)
        target_compile_options(civitas_x PRIVATE /WX)
    endif()
else()
    target_compile_options(civitas_x PRIVATE -Wall -Wextra -Wpedantic)
    if(CIVITASX_WARNINGS_AS_ERRORS)
        target_compile_options(civitas_x PRIVATE -Werror)
    endif()
endif()
