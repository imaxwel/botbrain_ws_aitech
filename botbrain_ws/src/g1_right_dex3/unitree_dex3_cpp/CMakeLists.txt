cmake_minimum_required(VERSION 3.15)

# Allow building with plain CMake (without scikit-build).
if(NOT DEFINED SKBUILD_PROJECT_NAME)
  set(SKBUILD_PROJECT_NAME unitree_cpp)
endif()
if(NOT DEFINED SKBUILD_PROJECT_VERSION)
  set(SKBUILD_PROJECT_VERSION 0.0.0)
endif()

project(${SKBUILD_PROJECT_NAME}
  VERSION ${SKBUILD_PROJECT_VERSION}
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)

# Force RPATH (not RUNPATH) so /usr/local/lib takes priority over LD_LIBRARY_PATH.
# This is needed because the system has multiple CycloneDDS versions (apt, ROS Humble,
# ROS2 workspace) but only /usr/local/lib has the matching C++ bindings (ddscxx).
# Force RPATH (old-style) instead of RPATH so that rpath takes priority over LD_LIBRARY_PATH
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--disable-new-dtags")
if(NOT DEFINED SKBUILD)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--disable-new-dtags")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--disable-new-dtags")
endif()

# ======================
# Build options
# ======================
option(UNITREE_CPP_BUILD_PYTHON "Build Python (pybind11) bindings" ON)

# When building wheels via scikit-build, always build the Python extension.
if(DEFINED SKBUILD)
  set(UNITREE_CPP_BUILD_PYTHON ON CACHE BOOL "Build Python (pybind11) bindings" FORCE)
endif()

if(UNITREE_CPP_BUILD_PYTHON)
  find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 CONFIG REQUIRED)
  find_package(pybind11 REQUIRED)
endif()

find_library(UNITREE_SDK2_LIB NAMES unitree_sdk2 PATHS /usr/local/lib /usr/lib)
find_path(UNITREE_SDK2_INCLUDE NAMES unitree/robot/channel/channel_factory.hpp PATHS /usr/local/include /usr/include)

find_library(DDSC_LIB NAMES ddsc PATHS /usr/local/lib /usr/lib NO_DEFAULT_PATH)
find_library(DDSCXX_LIB NAMES ddscxx PATHS /usr/local/lib /usr/lib NO_DEFAULT_PATH)
find_path(DDSCXX_ROOT_INCLUDE dds/dds.hpp PATHS /usr/local/include/ddscxx /usr/include NO_DEFAULT_PATH)

if(NOT UNITREE_SDK2_LIB OR NOT UNITREE_SDK2_INCLUDE)
  message(FATAL_ERROR "Cannot find installed unitree_sdk2 library or headers in /usr/local")
endif()

if(NOT DDSC_LIB OR NOT DDSCXX_LIB OR NOT DDSCXX_ROOT_INCLUDE)
  message(FATAL_ERROR "Cannot find ddscxx or headers")
endif()

message(STATUS "Found unitree_sdk2 lib: ${UNITREE_SDK2_LIB}")
message(STATUS "Found unitree_sdk2 include: ${UNITREE_SDK2_INCLUDE}")
message(STATUS "Found ddsc lib: ${DDSC_LIB}")
message(STATUS "Found ddscxx lib: ${DDSCXX_LIB}")
message(STATUS "Found ddscxx root include: ${DDSCXX_ROOT_INCLUDE}")

if(UNITREE_CPP_BUILD_PYTHON)
  # Add a library using FindPython's tooling (pybind11 also provides a helper like this)
  python_add_library(unitree_cpp MODULE
    src/py_binding.cpp
    src/unitree_controller.cpp
    WITH_SOABI
  )

  target_include_directories(unitree_cpp PRIVATE
    ${UNITREE_SDK2_INCLUDE}
    ${DDSCXX_ROOT_INCLUDE}
  )

  target_link_libraries(unitree_cpp PRIVATE
    pybind11::headers
    ${UNITREE_SDK2_LIB}
    ${DDSC_LIB}
    ${DDSCXX_LIB}
    dl
    Threads::Threads
  )

  # Pass in the version as a define
  target_compile_definitions(unitree_cpp PRIVATE VERSION_INFO=${PROJECT_VERSION})

  # The install directory is the output (wheel) directory
  install(TARGETS unitree_cpp DESTINATION unitree_cpp)

  set_target_properties(unitree_cpp PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "/usr/local/lib"
BUILD_RPATH "/usr/local/lib"
  )
endif()

if(NOT DEFINED SKBUILD)
  # ======================
  # Debug C++ executable
  # ======================
  add_executable(debugcpp
    src/unitree_controller.cpp
  )

  target_compile_definitions(debugcpp PRIVATE
    UNITREE_CPP_DEBUG_MAIN
  )

  target_include_directories(debugcpp PRIVATE
    ${UNITREE_SDK2_INCLUDE}
    ${DDSCXX_ROOT_INCLUDE}
  )

  target_link_libraries(debugcpp PRIVATE
    ${UNITREE_SDK2_LIB}
    ${DDSC_LIB}
    ${DDSCXX_LIB}
    dl
    Threads::Threads
  )

  target_compile_options(debugcpp PRIVATE
    -g -O0
  )

  set_target_properties(debugcpp PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "/usr/local/lib"
BUILD_RPATH "/usr/local/lib"
  )

  # ======================
  # Debug Dex-3 executable
  # ======================
  if(EXISTS "${CMAKE_SOURCE_DIR}/src/debug_dex3.cpp")
    add_executable(debug_dex3
      src/debug_dex3.cpp
      src/unitree_controller.cpp
    )

    target_include_directories(debug_dex3 PRIVATE
      ${UNITREE_SDK2_INCLUDE}
      ${DDSCXX_ROOT_INCLUDE}
    )

    target_link_libraries(debug_dex3 PRIVATE
      ${UNITREE_SDK2_LIB}
      ${DDSC_LIB}
      ${DDSCXX_LIB}
      dl
      Threads::Threads
    )

    target_compile_options(debug_dex3 PRIVATE
      -g -O0
    )

    set_target_properties(debug_dex3 PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "/usr/local/lib"
BUILD_RPATH "/usr/local/lib"
    )
  endif()
endif()