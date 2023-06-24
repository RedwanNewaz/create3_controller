cmake_minimum_required(VERSION 3.5)

project(create3_controller_gui LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

add_executable(create3_controller_gui
  main.cpp
  mainwindow.cpp
  mainwindow.h
  mainwindow.ui
  resource.qrc
  processmanager.cpp
  processmanager.h
  ProgramOptions.h
)

target_link_libraries(create3_controller_gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets yaml-cpp)
