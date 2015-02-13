project(sd_gnuspeech)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -march=native")

#aux_source_directory(. SRC_LIST)
#add_executable(${PROJECT_NAME} ${SRC_LIST})

find_package(PkgConfig)

add_executable(sd_gnuspeech
    src/main.cpp
    src/ModuleConfiguration.cpp src/ModuleConfiguration.h
    src/ModuleController.cpp src/ModuleController.h
    src/SynthesizerController.cpp src/SynthesizerController.h
    src/Util.cpp src/Util.h
)

pkg_check_modules(PORTAUDIOCPP portaudiocpp)

find_file(LOCAL_GNUSPEECH_SA CMakeLists.txt PATHS ${CMAKE_BINARY_DIR}/../gnuspeech_sa/ NO_DEFAULT_PATH)
if(LOCAL_GNUSPEECH_SA)
    message(STATUS "Using local Gnuspeech-SA")
    include_directories(
        src
        ../gnuspeech_sa/src
        ../gnuspeech_sa/src/trm
        ../gnuspeech_sa/src/trm_control_model
        ${PORTAUDIOCPP_INCLUDE_DIRS}
    )
    link_directories(
        ${PORTAUDIOCPP_LIBRARY_DIRS}
    )
    target_link_libraries(sd_gnuspeech
        -l:../gnuspeech_sa-build/libgnuspeechsa.so
        ${PORTAUDIOCPP_LIBRARIES}
    )
else(LOCAL_GNUSPEECH_SA)
    message(STATUS "Using system Gnuspeech-SA")
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
endif(LOCAL_GNUSPEECH_SA)

if(UNIX)
    include(GNUInstallDirs)
    install(TARGETS sd_gnuspeech
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/speech-dispatcher-modules/)
    configure_file(conf/gnuspeech.conf.pre gnuspeech.conf @ONLY)
    install(FILES ${CMAKE_BINARY_DIR}/gnuspeech.conf DESTINATION /etc/speech-dispatcher/modules/)
    install(FILES ${CMAKE_BINARY_DIR}/gnuspeech.conf DESTINATION ${CMAKE_INSTALL_PREFIX}/share/speech-dispatcher/conf/modules/)
endif(UNIX)
