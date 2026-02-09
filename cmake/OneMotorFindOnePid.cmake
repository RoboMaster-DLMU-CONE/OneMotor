find_package(OnePID QUIET)

if (NOT OnePID_FOUND AND NOT OnePID::OnePID OR ZEPHYR_TOOLCHAIN_VARIANT)
    FetchContent_Declare(
            OnePID
            GIT_REPOSITORY https://gitee.com/dlmu-cone/OnePID
            GIT_TAG main
            GIT_SHALLOW true
    )
    FetchContent_MakeAvailable(OnePID)
endif ()
