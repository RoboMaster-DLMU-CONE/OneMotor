if (NOT mp-units::mp-units)
  if (BUILD_OM_INSTALL)
    set(MP_UNITS_BUILD_INSTALL ON CACHE BOOL "" FORCE)
  else ()
    set(MP_UNITS_BUILD_INSTALL OFF CACHE BOOL "" FORCE)
  endif ()

  if(ZEPHYR_TOOLCHAIN_VARIANT)
    # Use freestanding mode to disable exceptions (MP_UNITS_THROW -> std::abort())
    # but we'll remove -ffreestanding flag after to not break GCC 14.3.0's libstdc++
    set(MP_UNITS_API_FREESTANDING ON CACHE BOOL "" FORCE)
  else()
    set(MP_UNITS_API_FREESTANDING OFF CACHE BOOL "" FORCE)
  endif ()

  set(MP_UNITS_API_CONTRACTS "NONE" CACHE STRING "" FORCE)
  set(MP_UNITS_LIB_FORMAT_SUPPORTED ON CACHE BOOL "" FORCE)
  set(MP_UNITS_API_STD_FORMAT ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
            mp-units
            GIT_REPOSITORY https://github.com/mpusz/mp-units
            GIT_SHALLOW true
            GIT_TAG master
            SOURCE_SUBDIR src
            SYSTEM TRUE
    )
  FetchContent_MakeAvailable(mp-units)

  if(ZEPHYR_TOOLCHAIN_VARIANT)
    # Remove -ffreestanding flag that mp-units adds when FREESTANDING=ON
    # This flag breaks GCC 14.3.0's libstdc++ hosted headers (__STDC_HOSTED__=0)
    # But we keep MP_UNITS_HOSTED=0 to disable exceptions in mp-units
    get_target_property(_mp_units_compile_opts mp-units-core INTERFACE_COMPILE_OPTIONS)
    if (_mp_units_compile_opts)
      list(REMOVE_ITEM _mp_units_compile_opts "-ffreestanding")
      set_target_properties(mp-units-core PROPERTIES INTERFACE_COMPILE_OPTIONS "${_mp_units_compile_opts}")
    endif()
  endif ()
endif ()
