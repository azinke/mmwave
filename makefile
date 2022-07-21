# Compiler
CC = gcc
FLAGS = -o

ODIR = output

# Root directory
ROOT_DIR = ti

# MMWavelink
MMWLINK_IDIR = ${ROOT_DIR}/mmwavelink/src

mmwlink:
  ${CC} -c -Wall ${MMWLINK_IDIR}/*.c


# MMWave Ethernet
MMWETH_IDIR = ${ROOT_DIR}/ethernet/src

mmwethernet:
  ${CC} -c ${MMWETH_IDIR}/*.c


mmwave: mmwlink mmwethernet
  ${CC} -c ${ROOT_DIR}/mmwave

# Build all
all: mmwlink mmwethernet mmwave
  ${CC} -c *.c
  ${CC} -o app *.o

build: all

clean:
  @rm *.o
  @rm app
