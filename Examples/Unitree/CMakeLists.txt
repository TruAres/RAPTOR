add_library(H1lib SHARED
    src/H1DynamicsConstraints.cpp
    src/H1ConstrainedInverseDynamics.cpp
    src/H1CustomizedConstraints.cpp
    src/H1SingleStepOptimizer.cpp
    src/H1SingleStepPeriodicityConstraints.cpp
    src/H1MultipleStepOptimizer.cpp
    src/H1MultipleStepPeriodicityConstraints.cpp
)

target_include_directories(H1lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/KinematicsDynamics/include
    ${CMAKE_SOURCE_DIR}/Trajectories/include
    ${CMAKE_SOURCE_DIR}/Constraints/include
    ${CMAKE_SOURCE_DIR}/Costs/include
    ${CMAKE_SOURCE_DIR}/Optimization/include
    ${CMAKE_SOURCE_DIR}/Utils/include)

target_link_libraries(H1lib PUBLIC
    trajlib
    IDlib
    Conslib
    Costslib
    Optlib
    ipopt
    coinhsl
    ${Boost_LIBRARIES} 
    pinocchio::pinocchio
)

add_executable(H1_example 
    H1SingleStep.cpp)
    # VisualizationHelper.cpp)

target_link_libraries(H1_example PUBLIC
    H1lib
    yaml-cpp
)
target_compile_options(H1_example PUBLIC
    ${PINOCCHIO_FLAGS})

# add_executable(H1MultipleStep_example 
#     H1MultipleStep.cpp
# )

# target_link_libraries(H1MultipleStep_example PUBLIC
#     H1lib
#     yaml-cpp
# )
# target_compile_options(H1MultipleStep_example PUBLIC
#     ${PINOCCHIO_FLAGS})