cmake_minimum_required(VERSION 3.10.2)
project(bt_assignment1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find BehaviorTree.CPP (note: versions 4.x use different target names)
find_package(behaviortree_cpp 4.6 REQUIRED)

add_executable(${PROJECT_NAME} bt_demo.cpp)

# New versions use different target names - try these:
target_link_libraries(${PROJECT_NAME} 
    PRIVATE behaviortree_cpp::behaviortree_cpp  # For version 4.x
    # PRIVATE BT::behaviortree_cpp_v3           # For version 3.x
    # PRIVATE BT::behaviortree_cpp              # Alternative
)