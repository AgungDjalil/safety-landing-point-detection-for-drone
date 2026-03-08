# CMake >= 3.16 supaya bisa set CMP0074
cmake_minimum_required(VERSION 3.16)
project(segmentation_node)

cmake_policy(SET CMP0074 NEW)

# Warnings & C++17
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters features search io)  # <- lengkap
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenMP)

# Global PCL defs (pilih salah satu gaya: global atau per-target)
add_definitions(${PCL_DEFINITIONS})

# ---- cylinder_crop ----
add_executable(cylinder_crop src/cylinder_crop.cpp)
ament_target_dependencies(cylinder_crop rclcpp sensor_msgs pcl_conversions tf2_ros)
target_include_directories(cylinder_crop PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cylinder_crop ${PCL_LIBRARIES})

# ---- plane_segmentation_gng ----
add_executable(plane_segmentation_gng src/plane_segmentation_gng.cpp)
ament_target_dependencies(plane_segmentation_gng rclcpp sensor_msgs pcl_conversions visualization_msgs)
target_include_directories(plane_segmentation_gng PUBLIC 
      ${PCL_INCLUDE_DIRS}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
target_link_libraries(plane_segmentation_gng ${PCL_LIBRARIES} Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(plane_segmentation_gng OpenMP::OpenMP_CXX)
  # JANGAN definisikan _OPENMP manual: hapus baris sebelumnya
endif()

# ---- plane_segmentation_ransac ----
add_executable(plane_segmentation_ransac src/plane_segmentation_ransac.cpp)
ament_target_dependencies(plane_segmentation_ransac rclcpp sensor_msgs pcl_conversions)
target_include_directories(plane_segmentation_ransac PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(plane_segmentation_ransac ${PCL_LIBRARIES} Eigen3::Eigen)

# ---- ddgng_node ----
add_executable(ddgng_node src/ddgng_node.cpp)
ament_target_dependencies(ddgng_node  rclcpp sensor_msgs pcl_conversions visualization_msgs)
target_link_libraries(ddgng_node Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(ddgng_node OpenMP::OpenMP_CXX)
  target_compile_definitions(ddgng_node PRIVATE DDGNG_OPENMP)  # ini makro custom kamu, aman
endif()

# ---- landing_circle ----
add_executable(landing_circle src/landing_circle.cpp)
ament_target_dependencies(landing_circle rclcpp sensor_msgs pcl_conversions tf2 tf2_ros tf2_eigen)
target_include_directories(landing_circle PUBLIC 
    ${PCL_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
target_link_libraries(landing_circle ${PCL_LIBRARIES} Eigen3::Eigen)

add_executable(cloud_size_node src/cloud_size_node.cpp)
ament_target_dependencies(cloud_size_node
  rclcpp
  sensor_msgs
)


# ---- install ----
install(TARGETS
  landing_circle
  cylinder_crop
  plane_segmentation_gng
  plane_segmentation_ransac
  ddgng_node
  cloud_size_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME})

# ---- lint (opsional) ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
