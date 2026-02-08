file(GLOB_RECURSE U8G2_SOURCES
    csrc/*.c
)

idf_component_register(
    SRCS ${U8G2_SOURCES}
    INCLUDE_DIRS csrc
)
