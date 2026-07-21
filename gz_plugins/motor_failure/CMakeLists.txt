set(lib_name MotorFailureSystem)
add_library(${lib_name} SHARED MotorFailureSystem.cpp)
target_include_directories(${lib_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${lib_name}
    ${GZ_PLUGIN_TARGET}
    ${GZ_SIM_TARGET}
    ${GZ_TRANSPORT_TARGET}
)
