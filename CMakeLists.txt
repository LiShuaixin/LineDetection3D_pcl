# set project's name
PROJECT(LineFromPointCloud)

###############################################################################
# CMake settings
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

set( CMAKE_BUILD_TYPE "Release" )
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# OpenMP
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
	OPTION(WITH_OPENMP "Whether to use parallel processing capabilities of OPENMP. ON/OFF" ON)
ENDIF(OPENMP_FOUND)

# 搜索pcl
find_package( PCL REQUIRED )
if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )
  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )
endif()

IF(OPENMP_FOUND AND WITH_OPENMP)
	MESSAGE(STATUS "With OpenMP ")
	SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=${PROCESSOR_COUNT} -DOPENMP_NUM_THREADS=${PROCESSOR_COUNT} ${OpenMP_CXX_FLAGS} -DOPENMP")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMAX_OPENMP_NUM_THREADS=${PROCESSOR_COUNT} -DOPENMP_NUM_THREADS=${PROCESSOR_COUNT} ${OpenMP_CXX_FLAGS} ${OpenMP_C_FLAGS} -DOPENMP")
ELSE(OPENMP_FOUND AND WITH_OPENMP)
	MESSAGE(STATUS "Without OpenMP")
	SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")
	SET (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DMAX_OPENMP_NUM_THREADS=1 -DOPENMP_NUM_THREADS=1")
ENDIF(OPENMP_FOUND AND WITH_OPENMP)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

include_directories( 
${OPENCV_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
${TBB_INCLUDE_DIRS}
)

ADD_SUBDIRECTORY(src)
