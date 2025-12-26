idf_component_register(SRCS "machine_state.c" "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES bsp freertos rs485_comm)