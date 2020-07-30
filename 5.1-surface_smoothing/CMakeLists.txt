cmake_minimum_required(VERSION 2.8 FATAL_ERROR) 
# rgb_depth_saver为工程的名字 
project(test) # PCL 
find_package(PCL 1.8 REQUIRED) 
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


include_directories(${PCL_INCLUDE_DIRS}) 
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) # 运行主程序 
add_executable(test main.cpp) # 链接库 
target_link_libraries(test ${PCL_LIBRARIES})

