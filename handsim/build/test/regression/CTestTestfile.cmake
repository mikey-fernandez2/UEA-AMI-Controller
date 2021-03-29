# CMake generated Testfile for 
# Source directory: /home/haptix-e15-463/haptix/haptix_controller/handsim/test/regression
# Build directory: /home/haptix-e15-463/haptix/haptix_controller/handsim/build/test/regression
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(REGRESSION_86_motor_values "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test/regression/REGRESSION_86_motor_values" "--gtest_output=xml:/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/REGRESSION_86_motor_values.xml")
SET_TESTS_PROPERTIES(REGRESSION_86_motor_values PROPERTIES  TIMEOUT "240")
ADD_TEST(check_REGRESSION_86_motor_values "/home/haptix-e15-463/haptix/haptix_controller/handsim/tools/check_test_ran.py" "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/REGRESSION_86_motor_values.xml")
