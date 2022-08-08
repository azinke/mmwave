# Compiler
CC = gcc
CFLAGS = -o

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
	${CC} -c ${ROOT_DIR}/mmwave/*.c

cliopt:
	${CC} -c opt/*.c

# Build all
all: mmwlink mmwethernet mmwave cliopt
	${CC} -c *.c
	${CC} ${CFLAGS} mmwave *.o -lpthread
	@rm -f *.o

build: all

clean:
	@rm -f *.o
	@rm -f mmwave
