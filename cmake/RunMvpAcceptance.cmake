if(NOT DEFINED DEMO OR NOT DEFINED URDF OR NOT DEFINED OUT)
    message(FATAL_ERROR "RunMvpAcceptance.cmake requires DEMO, URDF and OUT")
endif()

# Acceptance must start from a clean result directory so stale artifacts can
# never mask a missing output from the current executable.
file(REMOVE_RECURSE "${OUT}")
file(MAKE_DIRECTORY "${OUT}")

execute_process(
    COMMAND "${DEMO}" --urdf "${URDF}" --out "${OUT}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE stdout_text
    ERROR_VARIABLE stderr_text
)
file(WRITE "${OUT}/demo_stdout.txt" "${stdout_text}")
file(WRITE "${OUT}/demo_stderr.txt" "${stderr_text}")

if(NOT result EQUAL 0)
    message(STATUS "robot_id_demo stdout:\n${stdout_text}")
    message(STATUS "robot_id_demo stderr:\n${stderr_text}")
    message(FATAL_ERROR "robot_id_demo acceptance run failed with exit code ${result}")
endif()
