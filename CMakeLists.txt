project(GnuspeechSASpeechDispatcherModule)
cmake_minimum_required(VERSION 2.8)

if(UNIX)
    if(APPLE)
        set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")
    else()
        set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -march=native")
    endif()
endif()

find_package(PkgConfig)

add_executable(sd_gnuspeech
    src/main.cpp
    src/ModuleConfiguration.cpp src/ModuleConfiguration.h
    src/ModuleController.cpp src/ModuleController.h
    src/SynthesizerController.cpp src/SynthesizerController.h
    src/Util.cpp src/Util.h
)

pkg_check_modules(PORTAUDIOCPP portaudiocpp)

find_file(LOCAL_GNUSPEECH_SA CMakeLists.txt PATHS ${CMAKE_BINARY_DIR}/../GnuspeechSA/ NO_DEFAULT_PATH)
if(LOCAL_GNUSPEECH_SA)
    message(STATUS "Using local GnuspeechSA")
    include_directories(
        src
        ../GnuspeechSA/src
        ../GnuspeechSA/src/trm
        ../GnuspeechSA/src/trm_control_model
        ${PORTAUDIOCPP_INCLUDE_DIRS}
    )
    link_directories(
        ${PORTAUDIOCPP_LIBRARY_DIRS}
    )
    target_link_libraries(sd_gnuspeech
        -L../GnuspeechSA-build -lgnuspeechsa
        ${PORTAUDIOCPP_LIBRARIES}
    )
else()
    message(STATUS "Using system GnuspeechSA")
    pkg_check_modules(SYSTEM_GNUSPEECH_SA gnuspeechsa)
    include_directories(
        src
        ${SYSTEM_GNUSPEECH_SA_INCLUDE_DIRS}
        ${PORTAUDIOCPP_INCLUDE_DIRS}
    )
    link_directories(
        ${SYSTEM_GNUSPEECH_SA_LIBRARY_DIRS}
        ${PORTAUDIOCPP_LIBRARY_DIRS}
    )
    target_link_libraries(sd_gnuspeech
        ${SYSTEM_GNUSPEECH_SA_LIBRARIES}
        ${PORTAUDIOCPP_LIBRARIES}
    )
endif()

if(UNIX AND NOT APPLE)
    install(TARGETS sd_gnuspeech
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/lib${LIB_DIR_SUFFIX}/speech-dispatcher-modules/)
    configure_file(conf/gnuspeech.conf.pre gnuspeech.conf @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/gnuspeech.conf DESTINATION /etc/speech-dispatcher/modules/)
    install(FILES ${CMAKE_BINARY_DIR}/gnuspeech.conf DESTINATION ${CMAKE_INSTALL_PREFIX}/share/speech-dispatcher/conf/modules/)
endif()
