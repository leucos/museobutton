# Name: Makefile
# Project: museobutton
# Location : daemon/
# Tabsize: 2
# License: GNU GPL v2
# This Revision: $Id: Makefile 277 2007-03-20 10:53:33Z cs $

COMPILE = gcc -Wall
.PHONY: clean

# symbolic targets:
all: museobuttond

museobuttond: museobuttond.o
	gcc -ggdb -o museobuttond -lusb -llo -lpthread museobuttond.o `pkg-config --libs glib-2.0`

#museobuttond.o: museobuttond.c
#	gcc -c museobuttond.c -o museobuttond.o

.c.o:
	$(COMPILE) `pkg-config --cflags glib-2.0` -ggdb -c $< -o $@


clean:
	@rm -f *.o *~ museobuttond