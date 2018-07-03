BASE = ..

target: bin

bin_paths = $(shell find . -maxdepth 2 -name 'Makefile' -printf "%h ")

bin: $(bin_paths:%=inPath_make/%)

clean: $(bin_paths:%=inPath_clean/%) cleanLocks

include $(BASE)/build/generic.mk
