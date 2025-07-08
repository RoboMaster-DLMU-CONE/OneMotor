find_package(HyCAN QUIET)

if (NOT HyCAN_FOUND)
    message(STATUS "HyCAN can't be found locally, try fetching from Github...")
    include(FetchContent)
    FetchContent_Declare(
            HyCAN_fetched
            GIT_REPOSITORY "https://github.com/RoboMaster-DLMU-CONE/HyCAN"
            GIT_TAG v0.6.2
    )
    FetchContent_MakeAvailable(HyCAN_fetched)
else ()
    message(STATUS "Found HyCAN v${HyCAN_VERSION}")
endif ()