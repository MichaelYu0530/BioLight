cmake_minimum_required(VERSION 3.12)
project(GelElectrophoresisBandDetection VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV package
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})

# Common source files
set(COMMON_SOURCES
    src/common/Class.cpp
    src/common/General.cpp
)

# PlanA executable
add_executable(planA_demo
    ${COMMON_SOURCES}
    src/PlanA/algorithm/PlanA.cpp
    src/PlanA/algorithm/SolutionPlanA.cpp
)
target_link_libraries(planA_demo ${OpenCV_LIBS})

# PlanB executable  
add_executable(planB_demo
    ${COMMON_SOURCES}
    src/PlanB/algorithm/PlanB.cpp
    src/PlanB/algorithm/SolutionPlanB.cpp
)
target_link_libraries(planB_demo ${OpenCV_LIBS})

# PlanC executable
add_executable(planC_demo
    ${COMMON_SOURCES}
    src/PlanC/algorithm/PlanC.cpp
    src/PlanC/algorithm/SolutionPlanC.cpp
)
target_link_libraries(planC_demo ${OpenCV_LIBS})

# Note: The original demo.cpp has been removed
# For complete algorithm demonstration, see the Python implementation in ../demo/

# Installation targets (optional)
install(TARGETS planA_demo planB_demo planC_demo
    RUNTIME DESTINATION bin
)

# Output information
message(STATUS "=============================================")
message(STATUS "Build targets created:")
message(STATUS "  planA_demo - Initial exploratory version")
message(STATUS "  planB_demo - Optimized version with geometric modeling")
message(STATUS "  planC_demo - Production-ready optimized version")
message(STATUS "=============================================")
message(STATUS "For runnable demonstration, see ../demo/ Python implementation")