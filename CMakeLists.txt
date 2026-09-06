cmake_minimum_required(VERSION 3.7)
project(Filters)

include(filters.cmake)

add_definitions(-fpic)

SET(LIBSBC_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/dec_sbc.c
)

SET(LIBSBC_LIBS
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsbc.a
)

SET(LIBSBC_INC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_filter(libsbc
        "${LIBSBC_SRC}"
        "${LIBSBC_LIBS}"
        []
        ""
        "${LIBSBC_INC}"
        ""
        "1")
