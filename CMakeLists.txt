cmake_minimum_required(VERSION 3.16)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project("surfive")

set (CMAKE_CXX_STANDARD 23)

# SDL
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

add_subdirectory(vendored/SDL EXCLUDE_FROM_ALL)

if(EMSCRIPTEN)
  set(CMAKE_EXECUTABLE_SUFFIX ".html" CACHE INTERNAL "")
endif()

add_executable (surfive WIN32 "surfive.cpp" )
target_link_libraries(surfive PRIVATE SDL3::SDL3)

