

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

target_link_libraries(create3_controller_gui PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

target_include_directories(create3_controller_gui PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>)
target_compile_features(create3_controller_gui PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS create3_controller_gui DESTINATION lib/${PROJECT_NAME})
