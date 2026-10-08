if(NOT DEFINED DEMO OR NOT DEFINED TMP)
    message(FATAL_ERROR "VerifyCliContract.cmake requires DEMO and TMP")
endif()
file(REMOVE_RECURSE "${TMP}")
file(MAKE_DIRECTORY "${TMP}")

execute_process(COMMAND "${DEMO}" --help RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT rc EQUAL 0 OR NOT out MATCHES "Usage:")
    message(FATAL_ERROR "--help contract failed: rc=${rc}, out='${out}', err='${err}'")
endif()

execute_process(COMMAND "${DEMO}" --not-a-real-option RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT rc EQUAL 1 OR NOT err MATCHES "Unknown or incomplete argument")
    message(FATAL_ERROR "unknown-option failure contract failed: rc=${rc}, err='${err}'")
endif()

execute_process(COMMAND "${DEMO}" --urdf RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT rc EQUAL 1 OR NOT err MATCHES "Unknown or incomplete argument")
    message(FATAL_ERROR "incomplete --urdf failure contract failed: rc=${rc}, err='${err}'")
endif()

execute_process(COMMAND "${DEMO}" --urdf "${TMP}/missing.urdf" --out "${TMP}/missing_run"
                RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT rc EQUAL 1 OR NOT err MATCHES "ERROR:")
    message(FATAL_ERROR "missing-URDF failure contract failed: rc=${rc}, err='${err}'")
endif()

message(STATUS "CLI help/error behavior PASS")
