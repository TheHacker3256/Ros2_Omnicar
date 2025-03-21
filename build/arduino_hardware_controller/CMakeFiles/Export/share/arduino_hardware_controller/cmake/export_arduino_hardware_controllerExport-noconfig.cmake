#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "arduino_hardware_controller::arduino_hardware_controller" for configuration ""
set_property(TARGET arduino_hardware_controller::arduino_hardware_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(arduino_hardware_controller::arduino_hardware_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libarduino_hardware_controller.so"
  IMPORTED_SONAME_NOCONFIG "libarduino_hardware_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS arduino_hardware_controller::arduino_hardware_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_arduino_hardware_controller::arduino_hardware_controller "${_IMPORT_PREFIX}/lib/libarduino_hardware_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
