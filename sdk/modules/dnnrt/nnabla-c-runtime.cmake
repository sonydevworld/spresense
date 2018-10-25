set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

include(CMakeForceCompiler)
include_directories(${project_root}/../../../../../../nuttx/include
 ${project_root}/include)

# specify the cross compiler
CMAKE_FORCE_C_COMPILER(${ARM_CC_PATH} GNU)
CMAKE_FORCE_CXX_COMPILER(${ARM_CXX_PATH} GNU)
set(CMAKE_AR ${CMAKE_AR_TOOL})
set(CMAKE_C_LINK_EXECUTABLE ${CMAKE_LINKER})
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "-s -specs=rdimon.specs")
set(CMAKE_C_FLAGS "${CMAKE_CFLAGS} -D'fminf(a,b)=((a)<(b)?(a):(b))' -D'fmaxf(a,b)=((a)>(b)?(a):(b))'")