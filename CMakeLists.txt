add_library(motor STATIC
    Motor.h
    Motor.cpp
)

# pico_generate_pio_header(encoder ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)

# This will link all the required libraries from RP2040's SDK
# in case it has already been linked from another proprietary
# library, the guard clauses in the preprocesor directives
# will avoid duplicate code
target_link_libraries(motor PUBLIC
    pico_stdlib
)

target_include_directories(motor PUBLIC ${CMAKE_CURRENT_LIST_DIR})


