cmake_minimum_required(VERSION 3.16)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project("surfive")

set (CMAKE_CXX_STANDARD 23)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

if(EMSCRIPTEN)
  add_subdirectory(vendored/SDL EXCLUDE_FROM_ALL)
  add_executable (surfive WIN32 "client.cpp" )
  set(CMAKE_EXECUTABLE_SUFFIX ".html" CACHE INTERNAL "")
  set_target_properties(surfive PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sample.bmp)
  set_target_properties(surfive PROPERTIES LINK_FLAGS "--preload-file ${CMAKE_CURRENT_SOURCE_DIR}/sample.bmp@./sample.bmp -sALLOW_MEMORY_GROWTH ")
  target_link_libraries(surfive PRIVATE SDL3::SDL3)
else()
  add_subdirectory(vendored/SDL EXCLUDE_FROM_ALL)
  add_subdirectory(vendored/SDL_net)
  add_executable(server server.cpp)
  target_link_libraries(server PRIVATE SDL3::SDL3 SDL3_net::SDL3_net)
endif()


