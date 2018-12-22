cmake_minimum_required(VERSION 3.12)
project(kevin-tablet-mode)

add_executable(kevin-tablet-mode kevin-tablet-mode.c)
target_link_libraries(kevin-tablet-mode m)
install(
  TARGETS kevin-tablet-mode
  RUNTIME DESTINATION bin
)
