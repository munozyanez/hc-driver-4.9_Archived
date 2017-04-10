CFLAGS=-Wall
PROGRAMS=example apitest hcantool abuse
all: $(PROGRAMS)

# Setting the unknown_hw flag makes sense only with some prototype boards
CFLAGS+=$(shell test -e ./unknown_hw && echo -DALLOW_UNKNOWN_HW)

%: %.c  hico_api.h
	$(CC) $(CFLAGS) $< -o $@ 

abuse : abuse.c  hico_api.h
	$(CC) $(CFLAGS) -pthread $< -o $@ 

clean:
	rm -f $(PROGRAMS) *.o

test: $(PROGRAMS)
	./example /dev/can0 /dev/can1 &&\
	./apitest /dev/can0 /dev/can1 &&\
	./hcantool.sh /dev/can0 /dev/can1 hcantool.c
