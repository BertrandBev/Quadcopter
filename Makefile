#
# Copyright 2014, NICTA
#
# This software may be distributed and modified according to the terms of
# the BSD 2-Clause license. Note that NO WARRANTY is provided.
# See "LICENSE_BSD2.txt" for details.
#
# @TAG(NICTA_BSD)
#

TARGETS := $(notdir ${SOURCE_DIR}).cdl
ADL := quadcopter.camkes
    
spi_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/spi/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
spi_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/spi/src/*.c))

gpio_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/gpio/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
gpio_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/gpio/src/*.c))

i2c_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/i2c/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
i2c_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/i2c/src/*.c))

flight_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/flight/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
flight_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/flight/src/*.c))

spi_sensors_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/spi_sensors/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
spi_sensors_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/spi_sensors/src/*.c))

planner_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/planner/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
planner_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/planner/src/*.c))

motors_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/motors/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
motors_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/motors/src/*.c))
    
timer_HFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/timer/include/*.h)) \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/include/*.h))
timer_CFILES := \
    $(patsubst ${SOURCE_DIR}/%,%,$(wildcard ${SOURCE_DIR}/components/timer/src/*.c))


uart_LIBS += platsupport
gpio_LIBS += platsupport
i2c_LIBS += platsupport
spi_LIBS += platsupport
timer_LIBS += platsupport
planner_LIBS += platsupport

include ${PWD}/tools/camkes/camkes.mk