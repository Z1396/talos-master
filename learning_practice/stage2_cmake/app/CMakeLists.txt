# app 的 CMakeLists：顶层应用，依赖 math（间接依赖 io）
add_executable(app src/main.cpp)
target_link_libraries(app PRIVATE math)
