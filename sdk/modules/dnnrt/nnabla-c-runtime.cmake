set(CMAKE_SYSTEM_NAME GNU)
set(CMAKE_SYSTEM_PROCESSOR arm)

include_directories(${project_root}/../../../../../../nuttx/include
 ${project_root}/include)

# skip compiler check
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

# specify the cross compiler
set(CMAKE_C_COMPILER ${ARM_CC_PATH})
set(CMAKE_CXX_COMPILER ${ARM_CXX_PATH})
set(CMAKE_AR ${CMAKE_AR_TOOL})
set(CMAKE_C_LINK_EXECUTABLE ${CMAKE_LINKER})
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "-s -specs=rdimon.specs")
set(CMAKE_C_FLAGS "${CMAKE_CFLAGS} -D'fminf(a,b)=((a)<(b)?(a):(b))' -D'fmaxf(a,b)=((a)>(b)?(a):(b))'")
