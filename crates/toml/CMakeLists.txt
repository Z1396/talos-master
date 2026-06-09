crate(
    NAME toml_std
    TYPE INTERFACE
    DEPS
        toml_impl
        fmt::fmt
        Boost::pfr
        magic_enum
        Eigen3::Eigen
)
