
include_directories(
    "${CMAKE_CURRENT_LIST_DIR}/src"
    "${CMAKE_CURRENT_LIST_DIR}/libs"
)

add_executable (
    TestTime
    test/TestTime.cpp
    src/core/Time.cpp
)
set_property(TARGET TestTime PROPERTY CXX_STANDARD 20)

add_executable (
    TestEigen
    test/TestEigen.cpp
    src/core/Time.cpp
)
set_property(TARGET TestEigen PROPERTY CXX_STANDARD 20)

add_executable (
    TestSerialCom
    test/TestSerialCom.cpp
    libs/serialib/Serialib.cpp
    src/comm/SerialStream.cpp
    src/parts/XYZServo.cpp
    src/core/Time.cpp
)
set_property(TARGET TestSerialCom PROPERTY CXX_STANDARD 20)

add_executable (
    CrawlerCPP
    src/CrawlerCPP.cpp
    libs/serialib/Serialib.cpp
    src/comm/SerialStream.cpp
    src/parts/XYZServo.cpp
    src/core/Time.cpp
    src/robot/Robot.cpp
    src/robot/Leg.cpp
    src/robot/Joint.cpp
    src/math/Mathf.cpp
)
set_property(TARGET CrawlerCPP PROPERTY CXX_STANDARD 20)
