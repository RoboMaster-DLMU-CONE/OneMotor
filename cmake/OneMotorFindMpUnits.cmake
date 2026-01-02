if(NOT mp-units::mp-units)
  if(BUILD_OM_INSTALL)
    set(MP_UNITS_BUILD_INSTALL ON CACHE BOOL "" FORCE)
  else()
    set(MP_UNITS_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
  endif()
  set(MP_UNITS_API_CONTRACTS "NONE" CACHE STRING "" FORCE)
  set(MP_UNITS_LIB_FORMAT_SUPPORTED ON CACHE BOOL "" FORCE)
  set(MP_UNITS_API_STD_FORMAT ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    mp-units
    GIT_REPOSITORY https://gitee.com/dlmu-cone/mp-units
    GIT_SHALLOW    true
    GIT_TAG        master
    SOURCE_SUBDIR  src
    SYSTEM         TRUE
  )
  FetchContent_MakeAvailable(mp-units)
endif()
