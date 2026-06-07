idf_component_register(
    SRCS "main.c" "pid.c"
    INCLUDE_DIRS "."
)
target_link_libraries(${COMPONENT_LIB} PUBLIC m)

