cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

# add boost
find_package(Boost COMPONENTS serialization iostreams REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(/include)
include_directories(${PROJECT_SOURCE_DIR}/msg_gen/cpp/include/tf_tunnel)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add_message_files(DIRECTORY msg FILES CompressedTF.msg)
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread) 
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(show_tf_tree src/show_tf_tree.cpp src/tf_tree.cpp)
target_link_libraries( show_tf_tree ${Boost_LIBRARIES} )

rosbuild_add_executable(show_tf_roots src/show_tf_roots.cpp src/tf_tree.cpp)
target_link_libraries( show_tf_roots ${Boost_LIBRARIES} )

rosbuild_add_executable(tf_encoder src/tf_encoder.cpp src/tf_tree.cpp src/tf_compression.cpp)
target_link_libraries( tf_encoder ${Boost_LIBRARIES} )

rosbuild_add_executable(tf_decoder src/tf_decoder.cpp src/tf_tree.cpp src/tf_compression.cpp)
target_link_libraries( tf_decoder ${Boost_LIBRARIES} )


