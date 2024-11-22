cmake_minimum_required(VERSION 3.8)
project(claw_api)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)
 
# define a library target called claw_api
add_library(claw_api src/claw_api.cpp)
 

target_include_directories(claw_api INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/claw_api>
  $<INSTALL_INTERFACE:include/claw_api>
)


# this line to exports the library
ament_export_targets(claw_api HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/claw_api
  DESTINATION include
)
 
install(
  TARGETS claw_api
  EXPORT claw_api
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)

# Helper function to add gtest with common settings
function(add_gtest_with_includes test_name source_file)
  ament_add_gtest(${test_name}
    ${source_file}
    $<TARGET_OBJECTS:claw_api>
  )
  
  target_include_directories(${test_name} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${libserial_INCLUDE_DIRS}
  )

target_link_libraries(${test_name} serial)
endfunction()

# add_gtest_with_includes(${PROJECT_NAME}_diff_drive_test test/diff_drive_test.cpp)
add_gtest_with_includes(${PROJECT_NAME}_api_test test/api_test.cpp)

endif()

ament_package()
