# CMake generated Testfile for 
# Source directory: /home/haptix-e15-463/haptix/haptix_controller/handsim/test/integration
# Build directory: /home/haptix-e15-463/haptix/haptix_controller/handsim/build/test/integration
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(INTEGRATION_physics "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test/integration/INTEGRATION_physics" "--gtest_output=xml:/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/INTEGRATION_physics.xml")
SET_TESTS_PROPERTIES(INTEGRATION_physics PROPERTIES  TIMEOUT "240")
ADD_TEST(check_INTEGRATION_physics "/home/haptix-e15-463/haptix/haptix_controller/handsim/tools/check_test_ran.py" "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/INTEGRATION_physics.xml")
ADD_TEST(INTEGRATION_sim_api "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test/integration/INTEGRATION_sim_api" "--gtest_output=xml:/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/INTEGRATION_sim_api.xml")
SET_TESTS_PROPERTIES(INTEGRATION_sim_api PROPERTIES  TIMEOUT "240")
ADD_TEST(check_INTEGRATION_sim_api "/home/haptix-e15-463/haptix/haptix_controller/handsim/tools/check_test_ran.py" "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/INTEGRATION_sim_api.xml")
ADD_TEST(INTEGRATION_sim_api_client "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test/integration/INTEGRATION_sim_api_client" "--gtest_output=xml:/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/INTEGRATION_sim_api_client.xml")
SET_TESTS_PROPERTIES(INTEGRATION_sim_api_client PROPERTIES  TIMEOUT "240")
ADD_TEST(check_INTEGRATION_sim_api_client "/home/haptix-e15-463/haptix/haptix_controller/handsim/tools/check_test_ran.py" "/home/haptix-e15-463/haptix/haptix_controller/handsim/build/test_results/INTEGRATION_sim_api_client.xml")
