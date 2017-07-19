project(gama_tts_speech_dispatcher_module)
cmake_minimum_required(VERSION 2.8)

if(UNIX)
    if(APPLE)
        set(CMAKE_CXX_FLAGS "-std=c++14 -stdlib=libc++")
    else()
        set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra")
    endif()
endif()

find_package(PkgConfig)

include(${CMAKE_SOURCE_DIR}/src/rtaudio/RtAudio.cmake)

set(SOURCE_FILES
    src/main.cpp
    src/ModuleConfiguration.cpp
    src/ModuleConfiguration.h
    src/ModuleController.cpp
    src/ModuleController.h
    src/rtaudio/RtAudio.cpp
    src/rtaudio/RtAudio.h
    src/SynthesizerController.cpp
    src/SynthesizerController.h
    src/Util.cpp
    src/Util.h
)

include_directories(
    src
    src/rtaudio
)

find_file(LOCAL_GAMA_TTS CMakeLists.txt PATHS ${CMAKE_BINARY_DIR}/../gama_tts/ NO_DEFAULT_PATH)
if(LOCAL_GAMA_TTS)
    message(STATUS "Using local GamaTTS")
    include_directories(
        ../gama_tts/src
        ../gama_tts/src/vtm
        ../gama_tts/src/vtm_control_model
    )
    add_executable(sd_gama_tts ${SOURCE_FILES})
    target_link_libraries(sd_gama_tts
        -L../gama_tts-build -lgamatts
        ${RTAUDIO_LINKLIBS}
    )
else()
    message(STATUS "Using system GamaTTS")
    pkg_check_modules(GAMATTS REQUIRED gama_tts)
    include_directories(
        ${GAMATTS_INCLUDE_DIRS}
    )
    add_executable(sd_gama_tts ${SOURCE_FILES})
    target_link_libraries(sd_gama_tts
        ${GAMATTS_LIBRARIES}
        ${RTAUDIO_LINKLIBS}
    )
endif()

if(UNIX AND NOT APPLE)
    install(TARGETS sd_gama_tts
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib${LIB_DIR_SUFFIX}/speech-dispatcher-modules/)
    configure_file(conf/gama_tts.conf.pre gama_tts.conf @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/gama_tts.conf DESTINATION /etc/speech-dispatcher/modules/)
endif()
