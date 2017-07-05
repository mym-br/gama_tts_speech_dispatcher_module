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

pkg_check_modules(PORTAUDIOCPP REQUIRED portaudiocpp)

set(SOURCE_FILES
    src/main.cpp
    src/ModuleConfiguration.cpp
    src/ModuleConfiguration.h
    src/ModuleController.cpp
    src/ModuleController.h
    src/SynthesizerController.cpp
    src/SynthesizerController.h
    src/Util.cpp
    src/Util.h
)

find_file(LOCAL_GAMA_TTS CMakeLists.txt PATHS ${CMAKE_BINARY_DIR}/../gama_tts/ NO_DEFAULT_PATH)
if(LOCAL_GAMA_TTS)
    message(STATUS "Using local GamaTTS")
    include_directories(
        src
        ../gama_tts/src
        ../gama_tts/src/vtm
        ../gama_tts/src/vtm_control_model
        ${PORTAUDIOCPP_INCLUDE_DIRS}
    )
    link_directories(
        ${PORTAUDIOCPP_LIBRARY_DIRS}
    )
    add_executable(sd_gama_tts ${SOURCE_FILES})
    target_link_libraries(sd_gama_tts
        -L../gama_tts-build -lgamatts
        ${PORTAUDIOCPP_LIBRARIES}
    )
else()
    message(STATUS "Using system GamaTTS")
    pkg_check_modules(GAMATTS REQUIRED gama_tts)
    include_directories(
        src
        ${GAMATTS_INCLUDE_DIRS}
        ${PORTAUDIOCPP_INCLUDE_DIRS}
    )
    link_directories(
        ${GAMATTS_LIBRARY_DIRS}
        ${PORTAUDIOCPP_LIBRARY_DIRS}
    )
    add_executable(sd_gama_tts ${SOURCE_FILES})
    target_link_libraries(sd_gama_tts
        ${GAMATTS_LIBRARIES}
        ${PORTAUDIOCPP_LIBRARIES}
    )
endif()

if(UNIX AND NOT APPLE)
    install(TARGETS sd_gama_tts
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib${LIB_DIR_SUFFIX}/speech-dispatcher-modules/)
    configure_file(conf/gama_tts.conf.pre gama_tts.conf @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/gama_tts.conf DESTINATION /etc/speech-dispatcher/modules/)
endif()
