add_executable(test_vehicle_bridge
    test_can_message.cpp
    test_vehicle_controller.cpp
    test_vehicle_state.cpp
)

target_link_libraries(test_vehicle_bridge PRIVATE vehicle_bridge)

add_test(NAME vehicle_bridge_tests COMMAND test_vehicle_bridge)
