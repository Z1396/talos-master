include_guard(GLOBAL)

set(ONNXRUNTIME_VERSION "1.26.0" CACHE STRING "ONNX Runtime version")

# Detect ONNX Runtime release platform name.
#
# Common release asset names:
#   onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz
#   onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}.tgz
#   onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz
#   onnxruntime-osx-arm64-${ONNXRUNTIME_VERSION}.tgz

if(APPLE)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(ONNXRUNTIME_PLATFORM "osx-arm64")
  else()
    message(FATAL_ERROR "ONNX Runtime only support osx-arm64")
  endif()

  set(ONNXRUNTIME_ARCHIVE_EXT "tgz")
  set(ONNXRUNTIME_RUNTIME_NAME "libonnxruntime.dylib")

elseif(UNIX)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ONNXRUNTIME_PLATFORM "linux-aarch64")
    message(WARNING "GPU runtime for ONNX Runtime not available.")
  else()
    set(ONNXRUNTIME_PLATFORM "linux-x64-gpu")
  endif()

  set(ONNXRUNTIME_ARCHIVE_EXT "tgz")
  set(ONNXRUNTIME_RUNTIME_NAME "libonnxruntime.so")
else()
  message(FATAL_ERROR "Unsupported platform for ONNX Runtime prebuilt package.")
endif()

set(ONNXRUNTIME_ARCHIVE_NAME
    "onnxruntime-${ONNXRUNTIME_PLATFORM}-${ONNXRUNTIME_VERSION}.${ONNXRUNTIME_ARCHIVE_EXT}")

set(ONNXRUNTIME_URL
    "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ONNXRUNTIME_ARCHIVE_NAME}")

fetch_dependency(NAME onnxruntime
                 ZIP_URL "${ONNXRUNTIME_URL}"
                 ZIP_NAME "${ONNXRUNTIME_ARCHIVE_NAME}")

FetchContent_GetProperties(onnxruntime SOURCE_DIR onnxruntime_SOURCE_DIR)

set(ONNXRUNTIME_INCLUDE_DIR
    "${onnxruntime_SOURCE_DIR}/include")

set(ONNXRUNTIME_LIB_DIR
    "${onnxruntime_SOURCE_DIR}/lib")

if(APPLE)
  set(ONNXRUNTIME_RUNTIME
        "${ONNXRUNTIME_LIB_DIR}/${ONNXRUNTIME_RUNTIME_NAME}")

  if(NOT EXISTS "${ONNXRUNTIME_RUNTIME}")
    message(FATAL_ERROR "ONNX Runtime dylib not found: ${ONNXRUNTIME_RUNTIME}")
  endif()

  add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)

  set_target_properties(onnxruntime::onnxruntime PROPERTIES
        IMPORTED_LOCATION "${ONNXRUNTIME_RUNTIME}"
        INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
    )

else()
  set(ONNXRUNTIME_RUNTIME
        "${ONNXRUNTIME_LIB_DIR}/${ONNXRUNTIME_RUNTIME_NAME}")

  if(NOT EXISTS "${ONNXRUNTIME_RUNTIME}")
    message(FATAL_ERROR "ONNX Runtime shared library not found: ${ONNXRUNTIME_RUNTIME}")
  endif()

  add_library(onnxruntime::onnxruntime SHARED IMPORTED GLOBAL)

  set_target_properties(onnxruntime::onnxruntime PROPERTIES
        IMPORTED_LOCATION "${ONNXRUNTIME_RUNTIME}"
        INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
    )
endif()
