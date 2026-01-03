if(NOT OnePID::OnePID)
  FetchContent_Declare(
            OnePID
            GIT_REPOSITORY https://gitee.com/dlmu-cone/OnePID
            GIT_TAG main
            GIT_SHALLOW true
    )
  FetchContent_MakeAvailable(OnePID)
endif()
