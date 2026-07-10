# math crate 的 CMakeLists：依赖 io crate
# crate() 宏自动处理 PUBLIC 链接，依赖传递给上层
crate(math DEPENDENCIES io)
