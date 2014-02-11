#
# Assorted tests and the like
#

MODULE_COMMAND		 = tests
MODULE_STACKSIZE	 = 12000
MAXOPTIMIZATION		 = -Os

SRCS			 = test_adc.c \
			   test_bson.c \
			   test_float.c \
			   test_hott_telemetry.c \
			   test_hrt.c \
			   test_int.c \
			   test_jig_voltages.c \
			   test_led.c \
			   test_sensors.c \
			   test_servo.c \
			   test_sleep.c \
			   test_time.c \
			   test_uart_baudchange.c \
			   test_uart_console.c \
			   test_uart_loopback.c \
			   test_uart_send.c \
			   test_mixer.cpp \
			   test_file.c \
			   tests_main.c \
			   test_param.c \
			   test_ppm_loopback.c \
			   test_rc.c \
			   test_conv.cpp \
			   test_mount.c \
			   test_mtd.c

ifeq ($(CONFIG_BOARD),PX4FMU_V1)
SRCS		   += test_gpio.c
endi