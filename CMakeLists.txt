project(sd_gnuspeech)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")

#aux_source_directory(. SRC_LIST)
#add_executable(${PROJECT_NAME} ${SRC_LIST})

find_package(PkgConfig)

pkg_check_modules(PORTAUDIOCPP portaudiocpp)

link_directories(
    ${PORTAUDIOCPP_LIBRARY_DIRS}
)

include_directories(
    src
    ../gnuspeech_sa/src
    ../gnuspeech_sa/src/trm
    ../gnuspeech_sa/src/trm_control_model
    ${PORTAUDIOCPP_INCLUDE_DIRS}
)

add_executable(sd_gnuspeech
    src/main.cpp
    src/ModuleConfiguration.cpp src/ModuleConfiguration.h
    src/ModuleController.cpp src/ModuleController.h
    src/SynthesizerController.cpp src/SynthesizerController.h
    src/Util.cpp src/Util.h
)
target_link_libraries(sd_gnuspeech
    -l:../gnuspeech_sa-build/libgnuspeechsa.a
    ${PORTAUDIOCPP_LIBRARIES}
)
