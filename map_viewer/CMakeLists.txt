cmake_minimum_required(VERSION 3.4)

project(MapViewer CXX)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../modules/" ${CMAKE_MODULE_PATH})

find_package(Corrade REQUIRED Main)
find_package(Magnum REQUIRED
    GL
    MeshTools
    Primitives
    SceneGraph
    Shaders
    Sdl2Application)
# find_package(OpenCV HINTS /usr/local/opt/opencv3)
find_package(OpenCV REQUIRED)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_viewer src/main.cpp)
target_link_libraries(map_viewer
PRIVATE
    ${OpenCV_LIBS}
    Corrade::Main
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders
    )

# install(TARGETS magnum-mouseinteraction DESTINATION ${MAGNUM_BINARY_INSTALL_DIR})

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT map_viewer)
