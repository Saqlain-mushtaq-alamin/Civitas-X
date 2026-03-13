cmake_minimum_required(VERSION 3.20)
project(CivitasX VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CIVITASX_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

add_executable(civitas_x
    src/main.cpp
    src/app.cpp
    src/world.cpp
    src/renderer2d.cpp
)

target_include_directories(civitas_x
    PRIVATE
        include
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

if(glm_FOUND)
    target_link_libraries(civitas_x PRIVATE glm::glm)
else()
    message(WARNING "glm not found via package manager. Add include path manually if needed.")
endif()

if(MSVC)
    target_compile_options(civitas_x PRIVATE /W4)
    if(CIVITASX_WARNINGS_AS_ERRORS)
        target_compile_options(civitas_x PRIVATE /WX)
    endif()
else()
    target_compile_options(civitas_x PRIVATE -Wall -Wextra -Wpedantic)
    if(CIVITASX_WARNINGS_AS_ERRORS)
        target_compile_options(civitas_x PRIVATE -Werror)
    endif()
endif()
