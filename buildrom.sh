file=$1

r68 -m0 ${file}.a -o=${file}.r -u=$MWOS/OS9/SRC/DEFS
l68 -r=f00000 ${file}.r -o=${file}.rom
#cp ${file}.rom rlm-sys-1.rom

gcc -DTERM0_CONS -DTIMER_FREQ=2000 -o mach mach.c timer_1ms.c\
 -L. -L/usr/local/lib -lfame -lwinmm -lzmq &&\
 ./mach ${file}.rom
