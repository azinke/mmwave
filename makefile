# Compiler
CC = gcc
CFLAGS = -o
FLAGS = -c -w

ODIR = output

# Root directory
ROOT_DIR = ti

# MMWavelink
MMWLINK_IDIR = ${ROOT_DIR}/mmwavelink/src

mmwlink:
	@${CC} ${FLAGS} ${MMWLINK_IDIR}/*.c


# MMWave Ethernet
MMWETH_IDIR = ${ROOT_DIR}/ethernet/src

mmwethernet:
	@${CC} ${FLAGS} ${MMWETH_IDIR}/*.c


mmwave: mmwlink mmwethernet
	@${CC} ${FLAGS} ${ROOT_DIR}/mmwave/*.c

cliopt:
	@${CC} ${FLAGS} opt/*.c

tomlconfig:
	@${CC} ${FLAGS} toml/*.c

# Build all
all: mmwlink mmwethernet mmwave cliopt tomlconfig
	@${CC} ${FLAGS} *.c
	@${CC} ${CFLAGS} mmwave *.o -lpthread -lm
	@rm -f *.o

build: all

install: all
	@sudo cp -f ./mmwave /usr/bin
	@rm -f ./mmwave

clean:
	@rm -f *.o
	@rm -f mmwave
