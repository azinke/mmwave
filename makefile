# Compiler
CC = gcc
CCP = g++
FLAGS = -o

ODIR = output

# Root directory
ROOT_DIR = ti

# MMWavelink
MMWLINK_IDIR = ${ROOT_DIR}/mmwavelink/src
MMWLINK_ODIR = ${ODIR}/mmwavelink
MMWLIKN_OBJS = rl_controller.o rl_device.o rl_driver.o rl_monitoring.o rl_sensor.o
MMWLINK_SO = mmwavelink.so

mmwlink:
	${CC} -c -Wall -fpic ${MMWLINK_IDIR}/*.c
	${CC} -shared -o ${MMWLINK_SO} ${MMWLIKN_OBJS}
	@mkdir -p ${MMWLINK_ODIR}
	@mv ${MMWLIKN_OBJS} ${MMWLINK_ODIR}
	@mv ${MMWLINK_SO} ${MMWLINK_ODIR}


# MMWave Ethernet
MMWETH_IDIR = ${ROOT_DIR}/ethernet/src
MMWETH_ODIR = ${ODIR}/ethernet
MMWETH_OBJS = mmwl_port_ethernet.o
MMWETH_SO = mmwl_port_ethernet.so

mmwethernet:
	${CC} -c -fpic ${MMWETH_IDIR}/*.c
	${CC} -shared -o ${MMWETH_SO} ${MMWETH_OBJS}
	@mkdir -p ${MMWETH_ODIR}
	@mv ${MMWETH_OBJS} ${MMWETH_ODIR}
	@mv ${MMWETH_SO} ${MMWETH_ODIR}


mmwave:
	${CCP} ${FLAGS} mmwave mmwave.cpp

# Build all
all: mmwlink mmwethernet mmwave

clean:
	rm -r ${ODIR}
