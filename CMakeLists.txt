cmake_minimum_required(VERSION 3.20)
project(WebFlightCpp CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── vcpkg (glad, glfw3, glm, imgui, curl, stb, nlohmann_json) ───────────────
find_package(glad          CONFIG REQUIRED)
find_package(glfw3         CONFIG REQUIRED)
find_package(glm           CONFIG REQUIRED)
find_package(imgui         CONFIG REQUIRED)
find_package(CURL          CONFIG REQUIRED)
find_package(Stb           REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# ── JSBSim — baixa e compila do GitHub ──────────────────────────────────────
include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

set(BUILD_PYTHON_MODULE      OFF CACHE BOOL "" FORCE)
set(JSBSIM_INSTALL           OFF CACHE BOOL "" FORCE)
set(SYSTEM_EXPAT             OFF CACHE BOOL "" FORCE)  # usa expat bundled

FetchContent_Declare(jsbsim
  GIT_REPOSITORY https://github.com/JSBSim-Team/jsbsim.git
  GIT_TAG        master
  GIT_SHALLOW    ON
)
FetchContent_MakeAvailable(jsbsim)

# ── Caminhos dos dados (self-contained em data/) ─────────────────────────────
set(DATA_ROOT "${CMAKE_SOURCE_DIR}/data")
file(TO_CMAKE_PATH "${DATA_ROOT}/aircraft"  _AC_PATH)
file(TO_CMAKE_PATH "${DATA_ROOT}/engine"    _EN_PATH)
file(TO_CMAKE_PATH "${DATA_ROOT}"           _SY_PATH)   # JSBSim procura data/systems/
file(TO_CMAKE_PATH "${DATA_ROOT}/nav"       _DATA_PATH) # airports.csv, runways.csv

# ── Executável ───────────────────────────────────────────────────────────────
add_executable(webflight
  src/main.cpp
  src/FDM.cpp
  src/FlyByWire.cpp
  src/Sky.cpp
  src/Terrain.cpp
  src/TileManager.cpp
  src/Clouds.cpp
  src/AirportManager.cpp
  src/OSMManager.cpp
  src/PostFX.cpp
)

target_include_directories(webflight PRIVATE src ${Stb_INCLUDE_DIR})

target_compile_definitions(webflight PRIVATE
  AIRCRAFT_PATH="${_AC_PATH}"
  ENGINE_PATH="${_EN_PATH}"
  SYSTEMS_PATH="${_SY_PATH}"
  DATA_PATH="${_DATA_PATH}"
)

target_link_libraries(webflight PRIVATE
  libJSBSim
  glad::glad
  glfw
  glm::glm
  imgui::imgui
  CURL::libcurl
  nlohmann_json::nlohmann_json
  opengl32
)

if(MSVC)
  target_compile_options(webflight PRIVATE /W3 /utf-8 /wd4244 /wd4267 /wd4996)
  target_compile_definitions(webflight PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
