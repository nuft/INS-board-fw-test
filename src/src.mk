
SRC_FILES = main.c sensors.c runtime.c system_stm32f4xx.c uart.c i2c.c mpu60X0.c
CSOURCES += $(addprefix src/, $(SRC_FILES))
