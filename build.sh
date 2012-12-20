gcc -DTERM0_CONS -DTIMER_FREQ=1 \
-o mach mach.c timer_1ms.c \
-L. -L/usr/local/lib -lfame -lwinmm -lzmq && \
./mach $1
