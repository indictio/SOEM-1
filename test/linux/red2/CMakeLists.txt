
set(SOURCES red2.c)
add_executable(red2 ${SOURCES})
target_link_libraries(red2 soem m)
install(TARGETS red2 DESTINATION bin)
