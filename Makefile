
TGT=arm
PRU_INC=/usr/include 
PRU_LIB=/usr/lib
PASM=/home/root/AM335x_PRU_BeagleBone/GPIO_PWM_PRU/utils/pasm
PASM=pasm

all	:
	echo target is $(TGT)
ifeq ($(TGT),arm)
	make $(TGT)/mcf.o
	make $(TGT)/pructl
	#make $(TGT)/pructl_spi
	make $(TGT)/pru00.bin
	make $(TGT)/pru01.bin
	make $(TGT)/spi_awg.bin
	#make $(TGT)/pru01_spi.bin
	make $(TGT)/follower
	#make $(TGT)/follower_zero_crossing
	make $(TGT)/follower_fft
	make $(TGT)/follower_rms
	#make $(TGT)/SiEstimate.o
	#make $(TGT)/TstEst
	make $(TGT)/mio.o
#	make $(TGT)/spi_awg
endif

$(TGT)/mio.o	: mio.h mio.c | $(TGT)
	g++ $(CPPFLAGS) -c mio.c -o $(TGT)/mio.o


$(TGT)/mcf.o	: mcf.h mcf.cpp | $(TGT)
	g++ $(CPPFLAGS) -c mcf.cpp -o $(TGT)/mcf.o

$(TGT)/pructl   : pructl.c pructl.h pru.hp | $(TGT)
	g++ -I$(PRU_INC) -L$(PRU_LIB) pructl.c \
		-o $(TGT)/pructl -lprussdrv -lpthread 
#$(TGT)/spi_awg   : spi.c | $(TGT)
#	g++ -I$(PRU_INC) -L$(PRU_LIB) spi.c SimpleGPIO.cpp $(TGT)/mio.o \
#		-o $(TGT)/spi_awg -lprussdrv -lpthread 

#$(TGT)/pructl_spi   : pructl_spi.c | $(TGT)
#	gcc -I$(PRU_INC) -L$(PRU_LIB) pructl_spi.c \
#		-o $(TGT)/pructl_spi -lprussdrv -lpthread 

$(TGT)/follower   : follower.c pructl.h  | $(TGT)
	g++ -O3 -I$(PRU_INC) -L$(PRU_LIB) follower.c \
                 $(TGT)/mcf.o \
		-o $(TGT)/follower \
                 -lprussdrv -lpthread 
#$(TGT)/follower_zero_crossing   : follower_zero_crossing.c pructl.h  | $(TGT)
#	g++ -O3 -I$(PRU_INC) -L$(PRU_LIB) follower_zero_crossing.c \
#		-o $(TGT)/follower_zero_crossing \
#                 -lprussdrv -lpthread -lfftw3 
$(TGT)/follower_fft   : follower_fft.c pructl.h  | $(TGT)
	g++ -O3 -I$(PRU_INC) -L$(PRU_LIB) follower_fft.c \
		-o $(TGT)/follower_fft \
                 -lprussdrv -lpthread -lm -lfftw3 

$(TGT)/follower_rms   : follower_rms.c pructl.h  | $(TGT)
	g++ -O3 -I$(PRU_INC) -L$(PRU_LIB) follower_rms.c \
		-o $(TGT)/follower_rms \
                 -lprussdrv -lpthread -lm -lfftw3 
$(TGT)/pru00.bin : pru00.p pructl.h pru.hp | $(TGT)
	${PASM} -V3 -b pru00.p
	mv pru00.bin $(TGT)/pru00.bin
$(TGT)/spi_awg.bin : spi_awg.p pructl.h pru.hp | $(TGT)
	${PASM} -V3 -b spi_awg.p
	mv spi_awg.bin $(TGT)/spi_awg.bin

$(TGT)/pru01.bin : pru01.p pructl.h pru.hp | $(TGT)
	${PASM} -V3 -b pru01.p
	mv pru01.bin $(TGT)/pru01.bin
#$(TGT)/pru01_spi.bin : pru01_spi.p pructl.h pru.hp | $(TGT)
#	${PASM} -V3 -b pru01_spi.p
#	mv pru01_spi.bin $(TGT)/pru01_spi.bin

$(TGT)/get_line.o : get_line.s | $(TGT)
	as get_line.s -o $(TGT)/get_line.o

CPPFLAGS=-g


#$(TGT)/SiEstimate.o : SiEstimate.cpp SiEstimate.h | $(TGT)
#	g++ -O3 -g -c SiEstimate.cpp -o $(TGT)/SiEstimate.o
#
#$(TGT)/TstEst   : TstEst.cpp $(TGT)/SiEstimate.o  | $(TGT)
#	g++ -O3 -g TstEst.cpp $(TGT)/SiEstimate.o \
#                $(UTIL_OBJS) \
#                -lm -lfftw3 -lpthread -o $(TGT)/TstEst

clean	:
	rm -f $(TGT)/*
	rm -rf $(TGT)
	rm -f *~
	rm -f tmp
