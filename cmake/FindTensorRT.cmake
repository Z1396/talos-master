# FindTensorRT.cmake -- Locate NVIDIA TensorRT

include(FindPackageHandleStandardArgs)

# TensorRT root
if (DEFINED TensorRT_ROOT)
  list(APPEND _TensorRT_SEARCH_PATHS
    ${TensorRT_ROOT}
    "$ENV{TensorRT_ROOT}"
  )
endif()

# Detect architecture for Debian multiarch paths
set(_TensorRT_ARCH_SUFFIXES "")

# Try CMAKE_SYSTEM_PROCESSOR first
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
  list(APPEND _TensorRT_ARCH_SUFFIXES "aarch64-linux-gnu")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  list(APPEND _TensorRT_ARCH_SUFFIXES "x86_64-linux-gnu")
endif()

# Fallback: detect from compiler name or host system
if (_TensorRT_ARCH_SUFFIXES STREQUAL "")
  if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64")
    list(APPEND _TensorRT_ARCH_SUFFIXES "aarch64-linux-gnu")
  elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    list(APPEND _TensorRT_ARCH_SUFFIXES "x86_64-linux-gnu")
  endif()
endif()

# Last resort: add all common suffixes
if (_TensorRT_ARCH_SUFFIXES STREQUAL "")
  set(_TensorRT_ARCH_SUFFIXES
    "aarch64-linux-gnu"
    "x86_64-linux-gnu"
  )
endif()

list(APPEND _TensorRT_SEARCH_PATHS /usr /usr/local)

# Header - search all architecture suffixes
foreach(_arch IN LISTS _TensorRT_ARCH_SUFFIXES)
  list(APPEND _TensorRT_INCLUDE_SUFFIXES "include/${_arch}")
endforeach()
list(APPEND _TensorRT_INCLUDE_SUFFIXES "include")

find_path(TensorRT_INCLUDE_DIR
  NAMES NvInfer.h
  PATHS ${_TensorRT_SEARCH_PATHS}
  PATH_SUFFIXES ${_TensorRT_INCLUDE_SUFFIXES}
  NO_DEFAULT_PATH
)

# Core library - search all architecture suffixes
foreach(_arch IN LISTS _TensorRT_ARCH_SUFFIXES)
  list(APPEND _TensorRT_LIB_SUFFIXES "lib/${_arch}")
endforeach()
list(APPEND _TensorRT_LIB_SUFFIXES "lib64" "lib" "lib/x64")

find_library(TensorRT_LIBRARY
  NAMES nvinfer
  PATHS ${_TensorRT_SEARCH_PATHS}
  PATH_SUFFIXES ${_TensorRT_LIB_SUFFIXES}
  NO_DEFAULT_PATH
)

find_package_handle_standard_args(TensorRT
  REQUIRED_VARS TensorRT_INCLUDE_DIR TensorRT_LIBRARY
)

if (TensorRT_FOUND)
  set(TensorRT_INCLUDE_DIRS ${TensorRT_INCLUDE_DIR})
  set(TensorRT_LIBRARIES ${TensorRT_LIBRARY})

  # Optional components
  foreach(_comp IN ITEMS nvinfer_plugin nvonnxparser nvparsers)
    find_library(TensorRT_${_comp}_LIBRARY
      NAMES ${_comp}
      PATHS ${_TensorRT_SEARCH_PATHS}
      PATH_SUFFIXES ${_TensorRT_LIB_SUFFIXES}
      NO_DEFAULT_PATH
    )
    if (TensorRT_${_comp}_LIBRARY)
      list(APPEND TensorRT_LIBRARIES ${TensorRT_${_comp}_LIBRARY})
    endif()
  endforeach()

  # Core target
  add_library(TensorRT::TensorRT UNKNOWN IMPORTED)
  set_target_properties(TensorRT::TensorRT PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${TensorRT_INCLUDE_DIRS}"
    IMPORTED_LOCATION "${TensorRT_LIBRARY}"
  )

  # Component targets
  foreach(_comp IN ITEMS nvinfer_plugin nvonnxparser nvparsers)
    if (TensorRT_${_comp}_LIBRARY)
      add_library(TensorRT::${_comp} UNKNOWN IMPORTED)
      set_target_properties(TensorRT::${_comp} PROPERTIES
        IMPORTED_LOCATION "${TensorRT_${_comp}_LIBRARY}"
      )
    endif()
  endforeach()

  message(STATUS "Found TensorRT at ${TensorRT_INCLUDE_DIR}")
endif()
