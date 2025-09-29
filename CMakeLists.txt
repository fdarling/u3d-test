cmake_minimum_required(VERSION 3.13)

project(u3d-test LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# list(APPEND CMAKE_MODULE_PATH /home/USERNAME/apps/U3D/share/Urho3D/cmake/Modules)
# set(URHO3D_HOME /home/USERNAME/apps/U3D)
include(UrhoCommon)
# find_package (Urho3D REQUIRED) # NOTE: done implicitly by the above inclusion
find_package(assimp REQUIRED)

set(CMAKE_CXX_STANDARD 17) # setting again because UrhoCommon overrode it!

include_directories(
    ${URHO3D_INCLUDE_DIRS}
    src
)

set(TARGET_NAME ${PROJECT_NAME})

# NOTE: cannot use Urho3D's define_source_files() macro, as it looks for *.cpp and *.h in the current directory, not the "src/" directory!
set(SOURCE_FILES
    src/CreateMaterial.cpp
    src/CreatePrimitives.cpp
    src/SceneLoader.cpp
    src/Player.cpp
    src/JumpPad.cpp
    src/Ladder.cpp
    src/Ball.cpp
    src/main.cpp
)

set(ABSOLUTE_PATH_LIBS ${ASSIMP_LIBRARIES})

# Urho3D specific resource embedding step
setup_main_executable()
