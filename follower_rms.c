// This code is the same as the follower.c, however it
// performs a root-mean-square (RMS) calculation on blocks 
// of 'samples_to_rms' size


#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include "pructl.h"
#include <math.h>
#include <time.h>


int xyz;
float a1[10000];
float a2[10000];
float a3[10000];
float a4[10000];

static double *channel1 = NULL;
static double *channel2 = NULL;
static double *channel3 = NULL;
static double *channel4 = NULL;
static double *channel5 = NULL;
static double *channel6 = NULL;
static double *channel7 = NULL;
static double *channel8 = NULL;

int samples_to_rms = 2048;

//----------------------------------------------------------------------------
#define MAX_PATTERN_ERRS 10
static void *ddrMem;
static void *pru0DataMem;

//----------------------------------------------------------------------------
/**
 * This method copies 512 bytes from the source location to the destination
 * location using ldmi/stmia instructions to move 32 bytes at a time.
 */
void
copy512( unsigned short *src, unsigned short *dst )
{
	// NOTE: By specifying the clobber set, gcc will emit the
	// asm(" stmfd	sp!, { r3, r4, r5, r6, r7, r8, r9, r10 } ");
	// asm(" ldmfd	sp!, { r3, r4, r5, r6, r7, r8, r9, r10 } ");
	// surrounding the operation so we do not have to

	asm volatile (
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 32
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 64
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 96
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 128
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 160
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 192
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 224
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 256
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 288
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 320
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 352
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 384
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 416
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 448
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 480
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"	// 512
	  " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
	  : [src] "+r" (src), [dst] "+r" (dst)		   
	  :
	  : "r3","r4","r5","r6","r7","r8","r9","r10"   );
}

//----------------------------------------------------------------------------
/**
  * This method reads a number of samples from the head into the 
  * buffer provided.  The tailOffset value should be retained 
  * across invokations to maintain a stream or set to current value
  * of headOffset to flush the stream and obtain the most
  * current samples.
  *
  * The number of samples is returned.
  * The buffers should be at least page aligned.
  */
int counter = 0;

int get_sample_block( 
	volatile unsigned int *headOffsetPtr,
	unsigned int		  *tailOffsetPtr, 
	unsigned short		*dst )
{
	unsigned long long *llsrc,*lldst;
	unsigned int		mask;
	unsigned int		hoff,toff;
	int				 bytes_in_block = 4096;

	// Create a mask for the number of bytes to be read
	mask = ~(bytes_in_block-1);

	// Wait until head != tail
	do{
	   hoff = (*headOffsetPtr)&mask;
	   toff = (*tailOffsetPtr)&mask;
	}while( hoff==toff );

	// Copy from current tail to provided destination
	lldst = (unsigned long long*)dst;
	llsrc = (unsigned long long*)( (unsigned char*)ddrMem + toff );

	// copy using 32 byte copies
	int idx;
	for(idx=0;idx<(bytes_in_block/512);idx++){
	  copy512( (unsigned short*)llsrc, (unsigned short*)lldst);
	  llsrc+=64;
	  lldst+=64;
	}

	// Move tail forward
	*tailOffsetPtr=((*tailOffsetPtr)+bytes_in_block )%(2*PRU_MAX_SHORT_SAMPLES);

	return( bytes_in_block/sizeof(short) );
}

//----------------------------------------------------------------------------
/**
  * This method continuously follows the stream of PRU data checking
  * the pattern.
  *
  * It returns in once MAX_PATTERN_ERRS pattern errors have been encountered.
  */
void follow_stream( unsigned int usDelay )
{
	volatile unsigned int *headByteOffsetPtr;
	unsigned int		   tailByteOffset;
	int index = 0;
	unsigned short		 sample_bf[131072];
	
	int					ns;
	int					idx;
	int output;
	float output2;
	time_t sec;


	// setup gnuplot pipe
	FILE * gnuplotPipe = popen("gnuplot -persistant","w");
	fprintf(gnuplotPipe,"set term x11 \n");
	// fprintf(gnuplotPipe,"set xrange [0:%d] \n", numberOfSamplers - 1);
	//fprintf(gnuplotPipe,"set yrange [0:1.5] \n");


	// setup rms 
	channel1 = (double*)malloc(sizeof(double) * (samples_to_rms)); 
	channel2 = (double*)malloc(sizeof(double) * (samples_to_rms)); 
	channel3 = (double*)malloc(sizeof(double) * (samples_to_rms)); 
	channel4 = (double*)malloc(sizeof(double) * (samples_to_rms)); 
	memset(channel1, 0, (samples_to_rms)*sizeof(double));
	memset(channel2, 0, (samples_to_rms)*sizeof(double));
	memset(channel3, 0, (samples_to_rms)*sizeof(double));
	memset(channel4, 0, (samples_to_rms)*sizeof(double));

	// output variables
	double p1, p2, peak_sum, min_sum, last_peak, last_min;
	int count_peak, count_min, count_period, period_sum, start, first_peak, first_min, last_period;

	// Setup basic operating variables
	headByteOffsetPtr = (unsigned int*)( 
						   (unsigned char*)pru0DataMem+PRU0_OFFSET_DRAM_HEAD );
	tailByteOffset = *headByteOffsetPtr;

	int quit = 0, c = 0;
	double sum_1 = 0, sum_2 = 0, sum_3 = 0, sum_4 = 0, temp, temp2;
	while( !quit ){

		// Get a block of samples
		ns=get_sample_block( headByteOffsetPtr, &tailByteOffset, sample_bf );

		// convert samples, and sort into channels	
		for( idx=0;idx<ns;idx=idx+2 ){
			output = ((sample_bf[idx+1]&0x00FF)<<16)+sample_bf[idx];
			output2 = ((2.5*output)/8388608);
			if(sample_bf[idx+1]&0x0080){ output2 = output2-5;  }

		if((sample_bf[idx+1]&0xFF00) == 0x00){
			if(index < samples_to_rms){
				channel1[index] = output2;	
				index++;
			}
		}
		else if((sample_bf[idx+1]&0xFF00) == 0x2000){
			if(index < samples_to_rms){
				channel2[index] = output2;	
			}
		}
		else if((sample_bf[idx+1]&0xFF00) == 0x4000){
			if(index < samples_to_rms){
				channel3[index] = output2;	
			}
		}
		else if((sample_bf[idx+1]&0xFF00) == 0x6000){
			if(index < samples_to_rms){
				channel4[index] = output2;	
			}
		}
	}
	
	if(index==samples_to_rms){
		sum_1 = 0;
		sum_2 = 0;
		sum_3 = 0;
		sum_4 = 0;
		
		
		fprintf(gnuplotPipe,"plot '-' using 1:2 with lines, '-' using 1:3 with lines, '-' using 1:4 with lines, '-' using 1:5 with lines \n");
			
		for(index=0;index<samples_to_rms;index++){
			sum_1 = sum_1 + pow(channel1[index],2);
			sum_2 = sum_2 + pow(channel2[index],2);
			sum_3 = sum_3 + pow(channel3[index],2);
			sum_4 = sum_4 + pow(channel4[index],2);
			// fprintf(gnuplotPipe, "%lf %lf %lf\n", (double)index, temp,temp2);
		}
		sum_1 = sqrt((sum_1)/samples_to_rms);
		sum_2 = sqrt((sum_2)/samples_to_rms);
		sum_3 = sqrt((sum_3)/samples_to_rms);
		sum_4 = sqrt((sum_4)/samples_to_rms);
		a1[counter] = sum_1;
		a2[counter] = sum_2;
		a3[counter] = sum_3;
		a4[counter] = sum_4;
		counter++;
		printf("RMS calc %f %f %f %f \n",sum_1, sum_2, sum_3, sum_4);
		for(xyz = 0;xyz<counter;xyz++){
		fprintf(gnuplotPipe,"%d %f %f %f %f \n",xyz, a1[xyz], a2[xyz], a3[xyz], a4[xyz]);
		}
		sec = time(NULL);
		fprintf(gnuplotPipe,"e\n");
		fflush(gnuplotPipe);
		index = 0;
	}


	} // End of main loop 

}

//----------------------------------------------------------------------------
int main(int argc, char *argv[]  )
{
	unsigned int ret;
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	void *vptr;		 
	void *pptr;		 

	int usDelay  = 0;

	/* Initialize the PRU */
	printf("\tINFO: pruss init\n");
	prussdrv_init ();		   
	
	/* Open PRU Interrupt */
	printf("\tINFO: pruss open\n");
	ret = prussdrv_open(PRU_EVTOUT_0);
	if (ret)
	{
		printf("prussdrv_open open failed\n");
		return (ret);
	}

	/* Get the interrupt initialized */
	printf("\tINFO: pruss intc init\n");
	prussdrv_pruintc_init(&pruss_intc_initdata);

	/* Initialize example */
	printf("\tINFO: mapping memory \n");
	prussdrv_map_prumem (PRUSS0_PRU0_DATARAM, &pru0DataMem);

	/* Setup DRAM memory */
	printf("\tINFO: setup mem \n");
	// line 215 extram_phys_base is phys and read from sysfs
	// line 231 extram_base	  is virt and mmapped
	
	prussdrv_map_extmem( &vptr ); 
	printf("V extram_base = 0x%08x\n",(unsigned int) vptr);
	ddrMem = vptr;

	follow_stream( usDelay );

	return(0);

}
