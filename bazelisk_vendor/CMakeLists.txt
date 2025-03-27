cmake_minimum_required(VERSION 3.10)
project(bazelisk_vendor)

include(GNUInstallDirs)

set(LIBEXEC_DIR "${CMAKE_INSTALL_LIBDIR}/bazelisk_vendor"
    CACHE PATH "Location of program executables" )

function(download_and_install BINARY_NAME)
  set(BAZELISK_VERSION 1.25.0)
  set(BAZELISK_URL "https://github.com/bazelbuild/bazelisk/releases/download/v${BAZELISK_VERSION}")
  file(DOWNLOAD
    "${BAZELISK_URL}/${BINARY_NAME}"
    "${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}"
  )
  install(
    PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}"
    DESTINATION "${LIBEXEC_DIR}"
  )
endfunction()

# Download the binaries for bazelisk.
download_and_install(bazelisk-linux-amd64)
download_and_install(bazelisk-linux-arm64)

function(extract_version_from_xml_tag VERSION_STRING XML_FILE)
  file(READ ${XML_FILE} XML_CONTENT)

  if(XML_CONTENT MATCHES "<version>\\s*([^<]*)\\s*</version>")
    set(VERSION_STRING "${CMAKE_MATCH_1}")
    string(STRIP "${VERSION_STRING}" VERSION_STRING)
    set(VERSION_STRING "${VERSION_STRING}" PARENT_SCOPE)
  else()
    message(FATAL_ERROR "Failed to extract a <version> from file: ${XML_FILE}")
  endif()

endfunction()

extract_version_from_xml_tag(VERSION_STRING "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
message(STATUS "bazelisk_vendor version: ${VERSION_STRING}")

include(CMakePackageConfigHelpers)
configure_package_config_file(bazelisk_vendorConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/bazelisk_vendorConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bazelisk_vendor
  PATH_VARS LIBEXEC_DIR
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/bazelisk_vendorConfigVersion.cmake"
  VERSION "${VERSION_STRING}"
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/bazelisk_vendorConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/bazelisk_vendorConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/bazelisk_vendor
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/bazelisk_vendorConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/bazelisk_vendorConfigVersion.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Install the essential ament_index package resources and the package.xml.
install(CODE "
  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages)
  file(TOUCH ${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages/${PROJECT_NAME})
")
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)
