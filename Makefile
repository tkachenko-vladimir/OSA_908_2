# MPLAB IDE generated this makefile for use with GNU make.
# Project: OSA_908_2.mcp
# Date: Thu Jun 04 18:31:47 2015

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

OSA_908_2.hex : OSA_908_2.cof
	$(HX) "OSA_908_2.cof"

OSA_908_2.cof : osa.o main.o DEE\ Emulation\ 16-bit.o Flash\ Operations.o
	$(CC) -mcpu=24FJ64GA004 "osa.o" "main.o" "DEE Emulation 16-bit.o" "Flash Operations.o" "C:\Program Files\Microchip\mplabc30\v3.30c\lib\PIC24F\libpPIC24Fxxx-coff.a" -o"OSA_908_2.cof" -Wl,-Tp24FJ64GA004.gld,--defsym=__MPLAB_BUILD=1,-Map="OSA_908_2.map",--report-mem

osa.o : ../osa/kernel/system/osa_tasks.c ../osa/kernel/system/osa_system.c ../osa/kernel/events/osa_bsem.c ../osa/kernel/events/osa_csem.c ../osa/kernel/events/osa_queue.c ../osa/kernel/events/osa_squeue.c ../osa/kernel/timers/osa_ttimer.c ../osa/kernel/timers/osa_stimer.c ../osa/kernel/timers/osa_qtimer.c ../osa/kernel/timers/osa_dtimer.c ../osa/port/pic24/osa_pic24_mplabc.c ../osa/port/osa_include.c ../osa/kernel/timers/osa_timer.h ../osa/kernel/timers/osa_ttimer.h ../osa/kernel/timers/osa_qtimer.h ../osa/kernel/timers/osa_dtimer.h ../osa/kernel/timers/osa_stimer_old.h ../osa/kernel/timers/osa_stimer.h ../osa/kernel/events/osa_squeue.h ../osa/kernel/events/osa_smsg.h ../osa/kernel/events/osa_queue.h ../osa/kernel/events/osa_msg.h ../osa/kernel/events/osa_flag.h ../osa/kernel/events/osa_csem.h ../osa/kernel/events/osa_bsem.h ../osa/kernel/system/osa_tasks.h ../osa/kernel/system/osa_system.h ../osa/kernel/osa_oldnames.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24FJ64GA004.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24Fxxxx.h ../osa/port/pic24/osa_pic24_mplabc.h ../osa/port/osa_include.h OSAcfg.h ../osa/OSA.h ../osa/osa.c
	$(CC) -mcpu=24FJ64GA004 -x c -c "D:\Dropbox\osa\osa.c" -o"osa.o" -I"D:\Dropbox\OSA_908_2" -I"D:\Dropbox\osa" -g -Wall

main.o : ../osa/kernel/timers/osa_timer.h ../osa/kernel/timers/osa_ttimer.h ../osa/kernel/timers/osa_qtimer.h ../osa/kernel/timers/osa_dtimer.h ../osa/kernel/timers/osa_stimer_old.h ../osa/kernel/timers/osa_stimer.h ../osa/kernel/events/osa_squeue.h ../osa/kernel/events/osa_smsg.h ../osa/kernel/events/osa_queue.h ../osa/kernel/events/osa_msg.h ../osa/kernel/events/osa_flag.h ../osa/kernel/events/osa_csem.h ../osa/kernel/events/osa_bsem.h ../osa/kernel/system/osa_tasks.h ../osa/kernel/system/osa_system.h ../osa/kernel/osa_oldnames.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24FJ64GA004.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24Fxxxx.h ../osa/port/pic24/osa_pic24_mplabc.h ../osa/port/osa_include.h OSAcfg.h ../osa/osa.h c:/program\ files/microchip/mplabc30/v3.30c/support/peripheral_24F/PIC24F_periph_features.h c:/program\ files/microchip/mplabc30/v3.30c/support/peripheral_24F/spi.h c:/program\ files/microchip/mplabc30/v3.30c/support/peripheral_24F/GenericTypeDefs.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24FJ64GA004.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24Fxxxx.h c:/program\ files/microchip/mplabc30/v3.30c/support/peripheral_24F/PIC24F_periph_features.h i2c.h DEE\ Emulation\ 16-bit.h c:/program\ files/microchip/mplabc30/v3.30c/include/time.h c:/program\ files/microchip/mplabc30/v3.30c/include/math.h c:/program\ files/microchip/mplabc30/v3.30c/include/ctype.h c:/program\ files/microchip/mplabc30/v3.30c/include/stdlib.h c:/program\ files/microchip/mplabc30/v3.30c/include/stdarg.h c:/program\ files/microchip/mplabc30/v3.30c/include/stdio.h c:/program\ files/microchip/mplabc30/v3.30c/include/stddef.h c:/program\ files/microchip/mplabc30/v3.30c/include/string.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24FJ64GA004.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24Fxxxx.h main.c
	$(CC) -mcpu=24FJ64GA004 -x c -c "main.c" -o"main.o" -I"D:\Dropbox\OSA_908_2" -I"D:\Dropbox\osa" -g -Wall

DEE\ Emulation\ 16-bit.o : DEE\ Emulation\ 16-bit.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24FJ64GA004.h c:/program\ files/microchip/mplabc30/v3.30c/support/PIC24F/h/p24Fxxxx.h DEE\ Emulation\ 16-bit.c
	$(CC) -mcpu=24FJ64GA004 -x c -c "DEE Emulation 16-bit.c" -o"DEE Emulation 16-bit.o" -I"D:\Dropbox\OSA_908_2" -I"D:\Dropbox\osa" -g -Wall

Flash\ Operations.o : Flash\ Operations.s
	$(CC) -mcpu=24FJ64GA004 -c -I"D:\Dropbox\OSA_908_2" -I"D:\Dropbox\osa" "Flash Operations.s" -o"Flash Operations.o" -Wa,-g

clean : 
	$(RM) "osa.o" "main.o" "DEE Emulation 16-bit.o" "Flash Operations.o" "OSA_908_2.cof" "OSA_908_2.hex"

