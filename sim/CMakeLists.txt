add_library(sim INTERFACE)
target_include_directories(sim INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(sim INTERFACE math_lib)
