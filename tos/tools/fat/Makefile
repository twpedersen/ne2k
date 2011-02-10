
include ../../MakeVars

CC_HOST_OPT := $(CC_HOST_OPT) -DFS_STANDALONE -Wall

BIN = fatdir fatformat fatmd fatcopy fatdel fatsys
OBJ = fatdir.o fatformat.o fatmd.o fatcopy.o fatdel.o fatsys.o fat.o

all: $(BIN)

fatdir: fat.o fatdir.o
	$(LD_HOST) $(LD_HOST_OPT) -o fatdir fatdir.o fat.o

fatformat: fat.o fatformat.o
	$(LD_HOST) $(LD_HOST_OPT) -o fatformat fatformat.o fat.o

fatmd: fat.o fatmd.o
	$(LD_HOST) $(LD_HOST_OPT) -o fatmd fatmd.o fat.o

fatcopy: fat.o fatcopy.o
	$(LD_HOST) $(LD_HOST_OPT) -o fatcopy fatcopy.o fat.o

fatdel: fat.o fatdel.o
	$(LD_HOST) $(LD_HOST_OPT) -o fatdel fatdel.o fat.o

fatsys: fatsys.o
	$(LD_HOST) $(LD_HOST_OPT) -o fatsys fatsys.o

%.o: %.c
	$(CC_HOST) $(CC_HOST_OPT) -c $<

.PHONY: clean
clean:
	rm -f *~ $(BIN) $(OBJ)
