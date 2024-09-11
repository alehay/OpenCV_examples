cmake_minimum_required(VERSION 3.10)
project(CompileEachFile)

set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")
file(GLOB SOURCE_FILES "${SOURCE_DIR}/affine_transform/*.cpp")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV 4.9.0 REQUIRED)
find_package(Boost)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

# Function to set compile options
function(set_strict_compile_options target)
    target_compile_options(${target} PRIVATE
        -pedantic-errors
        -Wall
        -Wextra
        -pedantic
        -Wpedantic
        -Wfloat-equal
        -Wpointer-arith
        -Wstrict-overflow=5
        -Wwrite-strings
        -Waggregate-return
        -Wswitch-default
        -Wswitch-enum
        -Wconversion
        -Wunreachable-code
    )
endfunction()


foreach(SOURCE_FILE ${SOURCE_FILES})
    get_filename_component(FILENAME ${SOURCE_FILE} NAME_WE)
    add_executable(${FILENAME} ${SOURCE_FILE})
    target_include_directories(${FILENAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${FILENAME} ${OpenCV_LIBS} )
    set_strict_compile_options(${FILENAME})
endforeach()

#add_executable(${PROJECT_NAME} main.cpp)
#target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
#set_strict_compile_options(${PROJECT_NAME})

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

IF(tsan)
# https://clang.llvm.org/docs/ThreadSanitizer.html
message("TREAD sanitazer ON") 
add_compile_options(  -fsanitize=thread  ) 
ENDIF()
IF(asan) 
# https://clang.llvm.org/docs/AddressSanitizer.html
message("ADRESS sanitazer ON")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address ")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address  ")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address ")
add_compile_options( -fsanitize=address  ) 
ENDIF()

IF(msan) 
#https://clang.llvm.org/docs/MemorySanitizer.html
message("MEMORY sanitazer ON")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=memory  -fno-omit-frame-pointer ")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=memory  -fno-omit-frame-pointer  ")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=memory   -fno-omit-frame-pointer ")
add_compile_options( -fsanitize=memory  -fno-omit-frame-pointer  -fno-optimize-sibling-calls -fno-sanitize-memory-use-after-dtor -fsanitize-recover=all -O1 ) 
ENDIF()

IF(ubsan) 
#https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html
message("UB san  sanitazer ON")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=undefined")
add_compile_options(-fsanitize=undefined  -fsanitize=signed-integer-overflow,null,alignment -fsanitize-recover=all) 
ENDIF()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message(" \n ——————————► DEBUG ◄——————————")
    add_compile_options(-g3 -O0)
endif()

# Release configuration
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Print build type and OpenCV version
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

