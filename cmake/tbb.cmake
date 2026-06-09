function(sanitize_tbb)

    if(NOT TARGET TBB::tbb)
        return()
    endif()

    get_target_property(_incs
        TBB::tbb
        INTERFACE_INCLUDE_DIRECTORIES)

    if(NOT _incs)
        return()
    endif()

    set(_clean "")

    foreach(path IN LISTS _incs)

        #
        # Homebrew-style polluted root:
        #
        #   /opt/homebrew/include
        #
        # rewrite into:
        #
        #   /opt/homebrew/include/oneapi
        #

        if(EXISTS "${path}/oneapi/tbb/parallel_for.h")

            set(_fixed "${path}/oneapi")

            message(STATUS
                "[sanitize_tbb] rewrite:\n"
                "    ${path}\n"
                " -> ${_fixed}")

            list(APPEND _clean "${_fixed}")
            continue()

        endif()

        #
        # Legacy TBB layout
        #
        if(EXISTS "${path}/tbb/parallel_for.h")

            #
            # already narrow enough
            #
            list(APPEND _clean "${path}")
            continue()

        endif()

        #
        # keep unknown entries
        #
        list(APPEND _clean "${path}")

    endforeach()

    list(REMOVE_DUPLICATES _clean)

    set_target_properties(TBB::tbb PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_clean}"
    )

    get_target_property(_final
        TBB::tbb
        INTERFACE_INCLUDE_DIRECTORIES)

    message(STATUS
        "[sanitize_tbb] final = ${_final}")

endfunction()
