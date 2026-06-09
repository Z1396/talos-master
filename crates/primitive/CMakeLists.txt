crate(
    NAME primitive
    TYPE STATIC
    DEPS
        fmt::fmt
)

set_target_properties(primitive PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
