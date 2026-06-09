crate(
    NAME chiral
    TYPE INTERFACE
)

if(TALOS_BUILD_TESTING)
    # Both tests exercise the same named SHM endpoints and must not run concurrently.
    set_tests_properties(
        chiral_endpoint_test
        triple_buffer_simple_test
        PROPERTIES RESOURCE_LOCK chiral_shm
    )
endif()
