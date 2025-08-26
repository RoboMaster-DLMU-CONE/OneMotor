find_package(HyCAN QUIET)

if (NOT HyCAN_FOUND)
    message(STATUS "HyCAN can't be found locally, try fetching from remote...")
    include(FetchContent)
    execute_process(
            COMMAND git ls-remote --heads https://github.com/RoboMaster-DLMU-CONE/HyCAN
            RESULT_VARIABLE hycan_primary_repo_ok
            OUTPUT_QUIET
            ERROR_QUIET
    )

    if (NOT hycan_primary_repo_ok EQUAL 0)
        message(STATUS "Primary fetch failed, falling back to Gitee mirror")
        set(hycan_repo https://gitee.com/dlmu-cone/HyCAN.git)
    else ()
        set(hycan_repo https://github.com/RoboMaster-DLMU-CONE/HyCAN)
    endif ()

    FetchContent_Declare(
            HyCAN_fetched
            GIT_REPOSITORY ${hycan_repo}
            GIT_TAG "main"
    )
    FetchContent_MakeAvailable(HyCAN_fetched)
else ()
    message(STATUS "Found HyCAN v${HyCAN_VERSION}")
endif ()