# CAUTION:
#  - You can specify only one .pyx file for each cython module.
#  - The name of the module and the .pyx file should be identical.
cython_add_module(BLDCControllerClient BLDCControllerClient.pyx ../BLDCControllerClient.cpp)

# The library (libros_cython_example.so) is created in upper CMakeLists.txt
target_link_libraries(BLDCControllerClient blue_hardware_drivers)

# Set the targets to be copied in the python package directry.
set_target_properties(BLDCControllerClient
  PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

#install(TARGETS add_two_ints sub_two_ints
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

