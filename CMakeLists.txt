cmake_minimum_required(VERSION 3.22)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project(surfive)

set (CMAKE_CXX_STANDARD 23)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

if(EMSCRIPTEN)
  add_subdirectory(vendored/SDL EXCLUDE_FROM_ALL)
  add_executable(surfive WIN32 "client.cpp")
  set_target_properties(surfive PROPERTIES SUFFIX ".html")
  set_target_properties(surfive PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sample.bmp)
  set_target_properties(surfive PROPERTIES LINK_FLAGS
    "-sASYNCIFY=1 \
-sASSERTIONS=2 \
-sWEBSOCKET_DEBUG=1 \
--use-port=emdawnwebgpu \
--shell-file ${CMAKE_SOURCE_DIR}/index.html \
-lwebsocket.js \
--preload-file ${CMAKE_CURRENT_SOURCE_DIR}/sample.bmp@./sample.bmp
-sALLOW_MEMORY_GROWTH
-g -gsource-map=inline")
  set_target_properties(surfive PROPERTIES COMPILE_WARNING_AS_ERROR ON)
  target_link_libraries(surfive PUBLIC SDL3::SDL3)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --use-port=emdawnwebgpu -g -gsource-map=inline")
else()
  add_executable(server server.cpp)
  target_include_directories(server PRIVATE
    vendored/websocketpp
    vendored/asio/asio/include
  )

  target_compile_definitions(server PRIVATE
      ASIO_STANDALONE
      _WEBSOCKETPP_CPP11_INTERNAL_
  )
endif()


