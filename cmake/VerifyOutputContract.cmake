if(NOT DEFINED OUT)
    message(FATAL_ERROR "VerifyOutputContract.cmake requires OUT")
endif()

set(required_files
    parameters.csv
    singular_values.csv
    validation_metrics.csv
    control_metrics.csv
    trace_pd_only.csv
    trace_ct_nominal.csv
    trace_ct_identified.csv
    acceptance.csv
)

foreach(name IN LISTS required_files)
    set(path "${OUT}/${name}")
    if(NOT EXISTS "${path}")
        message(FATAL_ERROR "Required FR-07 artifact missing: ${name}")
    endif()
    file(SIZE "${path}" size)
    if(size EQUAL 0)
        message(FATAL_ERROR "Required FR-07 artifact is empty: ${name}")
    endif()
endforeach()

function(expect_header name expected)
    file(STRINGS "${OUT}/${name}" header LIMIT_COUNT 1)
    if(NOT header STREQUAL expected)
        message(FATAL_ERROR "Unexpected CSV header in ${name}: '${header}'")
    endif()
endfunction()

expect_header(parameters.csv "index,link,parameter,theta_true,theta_nominal,theta_identified")
expect_header(singular_values.csv "index,singular_value,relative_to_max,retained")
expect_header(validation_metrics.csv "joint,identified_rmse_nm,identified_nrmse,nominal_rmse_nm,nominal_nrmse")
expect_header(control_metrics.csv "controller,overall_rmse_rad,overall_rmse_deg,max_abs_error_rad,max_abs_error_deg,saturation_fraction,peak_unsaturated_effort_ratio,min_position_limit_margin_rad,peak_velocity_limit_ratio,joint1_rmse_rad,joint2_rmse_rad,joint3_rmse_rad,joint4_rmse_rad,joint5_rmse_rad,joint6_rmse_rad")
expect_header(acceptance.csv "criterion,metric,threshold_or_comparison,pass")
expect_header(trace_pd_only.csv "t,q1,q2,q3,q4,q5,q6,q_des1,q_des2,q_des3,q_des4,q_des5,q_des6,tau1,tau2,tau3,tau4,tau5,tau6")
expect_header(trace_ct_nominal.csv "t,q1,q2,q3,q4,q5,q6,q_des1,q_des2,q_des3,q_des4,q_des5,q_des6,tau1,tau2,tau3,tau4,tau5,tau6")
expect_header(trace_ct_identified.csv "t,q1,q2,q3,q4,q5,q6,q_des1,q_des2,q_des3,q_des4,q_des5,q_des6,tau1,tau2,tau3,tau4,tau5,tau6")

file(STRINGS "${OUT}/validation_metrics.csv" validation_lines)
list(LENGTH validation_lines validation_line_count)
if(NOT validation_line_count EQUAL 8)
    message(FATAL_ERROR "validation_metrics.csv must contain header + 6 joints + overall row; got ${validation_line_count} lines")
endif()
set(has_overall FALSE)
foreach(line IN LISTS validation_lines)
    if(line MATCHES "^overall,")
        set(has_overall TRUE)
    endif()
endforeach()
if(NOT has_overall)
    message(FATAL_ERROR "FR-05 overall validation row is missing")
endif()

file(STRINGS "${OUT}/control_metrics.csv" control_lines)
list(LENGTH control_lines control_line_count)
if(NOT control_line_count EQUAL 4)
    message(FATAL_ERROR "control_metrics.csv must contain header + 3 controllers")
endif()

file(STRINGS "${OUT}/parameters.csv" parameter_lines)
list(LENGTH parameter_lines parameter_line_count)
if(NOT parameter_line_count EQUAL 61)
    message(FATAL_ERROR "parameters.csv must contain header + 60 parameters")
endif()

file(STRINGS "${OUT}/acceptance.csv" acceptance_lines)
list(LENGTH acceptance_lines acceptance_line_count)
if(NOT acceptance_line_count EQUAL 4)
    message(FATAL_ERROR "acceptance.csv must contain header + A0/A1/A2")
endif()
foreach(criterion A0 A1 A2)
    set(found_pass FALSE)
    foreach(line IN LISTS acceptance_lines)
        if(line MATCHES "^${criterion},.*PASS$")
            set(found_pass TRUE)
        endif()
    endforeach()
    if(NOT found_pass)
        message(FATAL_ERROR "${criterion} is missing or not PASS in acceptance.csv")
    endif()
endforeach()

message(STATUS "FR-07 output contract verified: 8 required CSV artifacts, schemas and aggregate validation row PASS")
