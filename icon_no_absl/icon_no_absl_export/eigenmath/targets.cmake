# Automatically generated from BUILD in eigenmath

add_library(icon_shared_memory_eigenmath INTERFACE)
target_sources(icon_shared_memory_eigenmath PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/types.h"
)
target_include_directories(icon_shared_memory_eigenmath INTERFACE
  "$<BUILD_INTERFACE:${INSRC_ROOT}>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(icon_shared_memory_eigenmath INTERFACE
  Eigen3::Eigen
)
install(TARGETS icon_shared_memory_eigenmath
        EXPORT icon_shared_memoryTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(FILES
        "${CMAKE_CURRENT_LIST_DIR}/types.h"
        DESTINATION "include/eigenmath"
)
