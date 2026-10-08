cmake_minimum_required(VERSION 3.16)
project(robot_dynamics_id_ctc VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG TRUE)

option(ROBOT_ID_OFFLINE_CHECKS_ONLY
       "Build Eigen-only algorithm checks without requiring Pinocchio" OFF)

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    find_path(EIGEN3_INCLUDE_DIR Eigen/Core
              PATH_SUFFIXES eigen3
              HINTS ENV EIGEN3_INCLUDE_DIR)
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Eigen3 not found. Install Eigen3 or set EIGEN3_INCLUDE_DIR.")
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()

add_library(robot_id_core
    src/trajectory.cpp
    src/least_squares.cpp
    src/dynamic_parameters.cpp
    src/random.cpp
)
target_include_directories(robot_id_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robot_id_core PUBLIC Eigen3::Eigen)
target_compile_options(robot_id_core PRIVATE -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_trajectory_limits tests/test_trajectory_limits.cpp)
    target_link_libraries(test_trajectory_limits PRIVATE robot_id_core)
    add_test(NAME trajectory_limits COMMAND test_trajectory_limits)

    add_executable(test_scaled_svd tests/test_scaled_svd.cpp)
    target_link_libraries(test_scaled_svd PRIVATE robot_id_core)
    add_test(NAME scaled_svd_rank_deficient COMMAND test_scaled_svd)

    add_executable(test_nominal_mismatch tests/test_nominal_mismatch.cpp)
    target_link_libraries(test_nominal_mismatch PRIVATE robot_id_core)
    add_test(NAME nominal_mismatch_physicality COMMAND test_nominal_mismatch)

    add_executable(test_deterministic_noise tests/test_deterministic_noise.cpp)
    target_link_libraries(test_deterministic_noise PRIVATE robot_id_core)
    add_test(NAME deterministic_noise_sequence COMMAND test_deterministic_noise)
endif()

if(NOT ROBOT_ID_OFFLINE_CHECKS_ONLY)
    find_package(pinocchio 3.4 REQUIRED)
    find_package(tinyxml2 REQUIRED CONFIG)

    if(TARGET pinocchio::pinocchio_default)
        set(ROBOT_ID_PINOCCHIO_TARGET pinocchio::pinocchio_default)
    elseif(TARGET pinocchio::pinocchio)
        set(ROBOT_ID_PINOCCHIO_TARGET pinocchio::pinocchio)
    else()
        message(FATAL_ERROR "No supported Pinocchio CMake target found.")
    endif()

    add_library(robot_id
        src/robot_model.cpp
        src/pinocchio_model_loader.cpp
        src/identification.cpp
        src/experiment.cpp
    )
    target_include_directories(robot_id PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(robot_id PUBLIC robot_id_core ${ROBOT_ID_PINOCCHIO_TARGET} tinyxml2::tinyxml2)
    target_compile_options(robot_id PRIVATE -Wall -Wextra -Wpedantic)

    add_executable(robot_id_demo src/main.cpp)
    target_link_libraries(robot_id_demo PRIVATE robot_id)
    target_compile_options(robot_id_demo PRIVATE -Wall -Wextra -Wpedantic)

    if(BUILD_TESTING)
        add_executable(test_model_contract tests/test_model_contract.cpp)
        target_link_libraries(test_model_contract PRIVATE robot_id)
        add_test(NAME model_contract
                 COMMAND test_model_contract ${CMAKE_CURRENT_SOURCE_DIR}/models/aeroarm6.urdf)

        add_executable(test_regressor_identity tests/test_regressor_identity.cpp)
        target_link_libraries(test_regressor_identity PRIVATE robot_id)
        add_test(NAME regressor_identity
                 COMMAND test_regressor_identity ${CMAKE_CURRENT_SOURCE_DIR}/models/aeroarm6.urdf)

        add_executable(test_identification_pipeline tests/test_identification_pipeline.cpp)
        target_link_libraries(test_identification_pipeline PRIVATE robot_id)
        add_test(NAME identification_pipeline
                 COMMAND test_identification_pipeline ${CMAKE_CURRENT_SOURCE_DIR}/models/aeroarm6.urdf)

        add_executable(test_runtime_guardrails tests/test_runtime_guardrails.cpp)
        target_link_libraries(test_runtime_guardrails PRIVATE robot_id)
        add_test(NAME runtime_guardrails
                 COMMAND test_runtime_guardrails ${CMAKE_CURRENT_SOURCE_DIR}/models/aeroarm6.urdf)

        add_test(NAME cli_contract
                 COMMAND ${CMAKE_COMMAND}
                         -DDEMO=$<TARGET_FILE:robot_id_demo>
                         -DTMP=${CMAKE_CURRENT_BINARY_DIR}/cli_contract_tmp
                         -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/VerifyCliContract.cmake)

        add_test(NAME mvp_acceptance
                 COMMAND ${CMAKE_COMMAND}
                         -DDEMO=$<TARGET_FILE:robot_id_demo>
                         -DURDF=${CMAKE_CURRENT_SOURCE_DIR}/models/aeroarm6.urdf
                         -DOUT=${CMAKE_CURRENT_BINARY_DIR}/mvp_test_results
                         -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/RunMvpAcceptance.cmake)
        set_tests_properties(mvp_acceptance PROPERTIES
            RUN_SERIAL TRUE
            FIXTURES_SETUP mvp_results_fixture)

        add_test(NAME output_contract
                 COMMAND ${CMAKE_COMMAND}
                         -DOUT=${CMAKE_CURRENT_BINARY_DIR}/mvp_test_results
                         -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/VerifyOutputContract.cmake)
        set_tests_properties(output_contract PROPERTIES
            DEPENDS mvp_acceptance
            RUN_SERIAL TRUE
            FIXTURES_REQUIRED mvp_results_fixture)
    endif()
endif()
