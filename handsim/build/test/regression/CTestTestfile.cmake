# CMake generated Testfile for 
# Source directory: /home/haptix-e15-463/haptix/haptix_controller/handsim/test/regression
# Build directory: /home/haptix-e15-463/haptix/haptix_controller/handsim/build/test/regression
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(REGRESSION_86_motor_values "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test/regression/REGRESSION_86_motor_values" "--gtest_output=xml:/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/REGRESSION_86_motor_values.xml")
set_tests_properties(REGRESSION_86_motor_values PROPERTIES  TIMEOUT "240" _BACKTRACE_TRIPLES "/home/haptix-e15-463/haptix/haptix_controller/handsim/cmake/TestUtils.cmake;40;add_test;/home/haptix-e15-463/haptix/haptix_controller/handsim/test/regression/CMakeLists.txt;16;handsim_build_tests;/home/haptix-e15-463/haptix/haptix_controller/handsim/test/regression/CMakeLists.txt;0;")
add_test(check_REGRESSION_86_motor_values "/home/haptix-e15-463/haptix/haptix_controller/handsim/tools/check_test_ran.py" "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/REGRESSION_86_motor_values.xml")
set_tests_properties(check_REGRESSION_86_motor_values PROPERTIES  _BACKTRACE_TRIPLES "/home/haptix-e15-463/haptix/haptix_controller/handsim/cmake/TestUtils.cmake;47;add_test;/home/haptix-e15-463/haptix/haptix_controller/handsim/test/regression/CMakeLists.txt;16;handsim_build_tests;/home/haptix-e15-463/haptix/haptix_controller/handsim/test/regression/CMakeLists.txt;0;")
