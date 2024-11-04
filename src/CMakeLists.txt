# Top-level CMakeLists.txt for the corgi_ros_ws catkin workspace

cmake_minimum_required(VERSION 3.0.2)
project(corgi_ros_ws)

# Indicate that this is the top-level CMakeLists.txt for a catkin workspace
set(CATKIN_TOPLEVEL TRUE)

# --------------------------------------------------------------------------------------
# Attempt to find the 'catkin' package within the workspace using 'catkin_find_pkg'
# --------------------------------------------------------------------------------------

# Construct the command to find the 'catkin' package
set(find_catkin_cmd "catkin_find_pkg" "catkin" "${CMAKE_SOURCE_DIR}")

# Execute the command and capture the result
execute_process(
  COMMAND ${find_catkin_cmd}
  RESULT_VARIABLE find_catkin_result
  OUTPUT_VARIABLE find_catkin_output
  ERROR_VARIABLE find_catkin_error
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)

# Check if an error occurred during the search
if(NOT find_catkin_result EQUAL 0 AND NOT find_catkin_result EQUAL 2)
  # An error occurred while searching for 'catkin'
  string(REPLACE ";" " " find_catkin_cmd_str "${find_catkin_cmd}")
  message(FATAL_ERROR "Search for 'catkin' in workspace failed (${find_catkin_cmd_str}): ${find_catkin_error}")
endif()

# --------------------------------------------------------------------------------------
# Include 'catkin' from the workspace or find it using 'find_package()'
# --------------------------------------------------------------------------------------

if(find_catkin_result EQUAL 0)
  # 'catkin' package found in the workspace

  # Set the path to catkin's 'cmake' directory
  set(catkin_extras_dir "${CMAKE_SOURCE_DIR}/${find_catkin_output}/cmake")

  # Include 'all.cmake' from catkin without creating a new policy scope
  include("${catkin_extras_dir}/all.cmake" NO_POLICY_SCOPE)

  # Add the 'catkin' package as a subdirectory to the build
  add_subdirectory("${find_catkin_output}")

else()
  # 'catkin' package not found in the workspace

  # Ensure CMAKE_PREFIX_PATH is set, either from command line or environment variable
  if(NOT DEFINED CMAKE_PREFIX_PATH)
    if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      if(NOT WIN32)
        # Convert colon-separated list to semicolon-separated list (Unix-like systems)
        string(REPLACE ":" ";" CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
      else()
        # Use the environment variable as is (Windows systems)
        set(CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
      endif()
    endif()
  endif()

  # Build a list of catkin workspaces by checking for '.catkin' marker file in each path
  set(catkin_search_paths "")
  foreach(path ${CMAKE_PREFIX_PATH})
    if(EXISTS "${path}/.catkin")
      list(APPEND catkin_search_paths "${path}")
    endif()
  endforeach()

  # Remove duplicate paths to ensure each workspace is searched only once
  list(REMOVE_DUPLICATES catkin_search_paths)

  # Attempt to find the 'catkin' package using 'find_package' in the catkin workspaces
  set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
  find_package(catkin QUIET
    NO_POLICY_SCOPE
    PATHS ${catkin_search_paths}
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH)
  unset(CATKIN_TOPLEVEL_FIND_PACKAGE)

  # Check if 'catkin' was found
  if(NOT catkin_FOUND)
    message(FATAL_ERROR "find_package(catkin) failed. 'catkin' was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before.")
  endif()
endif()

# --------------------------------------------------------------------------------------
# Declare this directory as a catkin workspace
# --------------------------------------------------------------------------------------

catkin_workspace()

file(GLOB SUBDIRS "src/corgi_control_packages/*")

foreach(subdir ${SUBDIRS})
  if(IS_DIRECTORY ${subdir})
    add_subdirectory(${subdir})
  endif()
endforeach()