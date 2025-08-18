find_path(CUDNN_INCLUDE_DIR cudnn.h PATHS
  /usr/include/x86_64-linux-gnu
  /usr/local/cuda/include
)

find_library(CUDNN_LIBRARY cudnn PATHS
  /usr/lib/x86_64-linux-gnu
  /usr/local/cuda/lib64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CUDNN DEFAULT_MSG CUDNN_LIBRARY CUDNN_INCLUDE_DIR)

if(CUDNN_FOUND)
  set(CUDNN_INCLUDE_DIRS ${CUDNN_INCLUDE_DIR})
  set(CUDNN_LIBRARIES ${CUDNN_LIBRARY})
endif()

