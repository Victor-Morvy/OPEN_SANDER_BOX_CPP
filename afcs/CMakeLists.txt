# ── AFCS — Guidance & Autopilot como projeto independente ────────────────────
#
# Compila o núcleo FBW + Guidance (sem NENHUMA dependência externa: só C++17)
# como biblioteca estática + executável de testes offline.
#
#   cmake -B build -S .
#   cmake --build build --config Release
#   build/Release/afcs_test.exe        (ou ctest --test-dir build)
#
# A lib `afcs` é o candidato a exportação para o E195-E2Sim.

cmake_minimum_required(VERSION 3.20)
project(afcs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fontes vivem em ../src — mesma fonte que o webflight usa (sem duplicação).
set(AFCS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_library(afcs STATIC
    ${AFCS_SRC_DIR}/FlyByWire.cpp
    ${AFCS_SRC_DIR}/GuidanceModule.cpp
)
target_include_directories(afcs PUBLIC ${AFCS_SRC_DIR})

# ── Testes offline (modelo cinemático simples, sem JSBSim/OpenGL) ─────────────
add_executable(afcs_test afcs_test.cpp)
target_link_libraries(afcs_test PRIVATE afcs)

enable_testing()
add_test(NAME afcs_smoke COMMAND afcs_test)
