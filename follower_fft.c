// This code is the same as the follower.c, however it runs an FFT on the
// data from the ADC in real-time, and displays the output on a gnuplot
// graph


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
#include <fftw3.h>


// prepare FFT

static fftw_plan FFTPlan;
static fftw_plan FFTPlan_inverse;
static double *channel1 = NULL;
static double *channel2 = NULL;
static double *channel3 = NULL;
static double *channel4 = NULL;

int samples_to_fft = 1024;
int interpolation_factor = 32;
int time_domain_zero_padding = 4096;



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
	fprintf(gnuplotPipe,"set yrange [0:2.6] \n");


	// setup fft
	channel1 = (double*)fftw_malloc(sizeof(double) * (samples_to_fft+time_domain_zero_padding)); 
	channel2 = (double*)fftw_malloc(sizeof(double) * (samples_to_fft+time_domain_zero_padding)); 
	channel3 = (double*)fftw_malloc(sizeof(double) * (samples_to_fft+time_domain_zero_padding)); 
	channel4 = (double*)fftw_malloc(sizeof(double) * (samples_to_fft+time_domain_zero_padding)); 

	memset(channel1, 0, (samples_to_fft+time_domain_zero_padding)*sizeof(double));
	memset(channel2, 0, (samples_to_fft+time_domain_zero_padding)*sizeof(double));
	memset(channel3, 0, (samples_to_fft+time_domain_zero_padding)*sizeof(double));
	memset(channel4, 0, (samples_to_fft+time_domain_zero_padding)*sizeof(double));

	int interpolation_size = samples_to_fft * interpolation_factor;

	fftw_complex * tmp_fd = (fftw_complex*)fftw_malloc((interpolation_size/2+1)*sizeof(fftw_complex));
	fftw_complex * tmp_fd2 = (fftw_complex*)fftw_malloc((interpolation_size/2+1)*sizeof(fftw_complex));
	fftw_complex * tmp_fd3 = (fftw_complex*)fftw_malloc((interpolation_size/2+1)*sizeof(fftw_complex));
	fftw_complex * tmp_fd4 = (fftw_complex*)fftw_malloc((interpolation_size/2+1)*sizeof(fftw_complex));

	double * result = (double*)fftw_malloc(interpolation_size*sizeof(double));
	double * result2 = (double*)fftw_malloc(interpolation_size*sizeof(double));
	double * result3 = (double*)fftw_malloc(interpolation_size*sizeof(double));
	double * result4 = (double*)fftw_malloc(interpolation_size*sizeof(double));

	fftw_plan fft_plan = fftw_plan_dft_r2c_1d(samples_to_fft+time_domain_zero_padding, channel1, tmp_fd, FFTW_ESTIMATE | FFTW_DESTROY_INPUT ); 
	fftw_plan fft_plan2 = fftw_plan_dft_r2c_1d(samples_to_fft+time_domain_zero_padding, channel2, tmp_fd2, FFTW_ESTIMATE | FFTW_DESTROY_INPUT ); 
	fftw_plan fft_plan3 = fftw_plan_dft_r2c_1d(samples_to_fft+time_domain_zero_padding, channel3, tmp_fd3, FFTW_ESTIMATE | FFTW_DESTROY_INPUT ); 
	fftw_plan fft_plan4 = fftw_plan_dft_r2c_1d(samples_to_fft+time_domain_zero_padding, channel4, tmp_fd4, FFTW_ESTIMATE | FFTW_DESTROY_INPUT ); 

	fftw_plan ifft_plan = fftw_plan_dft_c2r_1d(interpolation_size, tmp_fd, result, FFTW_ESTIMATE | FFTW_DESTROY_INPUT );
	fftw_plan ifft_plan2 = fftw_plan_dft_c2r_1d(interpolation_size, tmp_fd2, result2, FFTW_ESTIMATE | FFTW_DESTROY_INPUT );
	fftw_plan ifft_plan3 = fftw_plan_dft_c2r_1d(interpolation_size, tmp_fd3, result3, FFTW_ESTIMATE | FFTW_DESTROY_INPUT );
	fftw_plan ifft_plan4 = fftw_plan_dft_c2r_1d(interpolation_size, tmp_fd4, result4, FFTW_ESTIMATE | FFTW_DESTROY_INPUT );

	// output variables
	double p1, p2, peak_sum, min_sum, last_peak, last_min;
	int count_peak, count_min, count_period, period_sum, start, first_peak, first_min, last_period;

	// Setup basic operating variables
	headByteOffsetPtr = (unsigned int*)( 
						   (unsigned char*)pru0DataMem+PRU0_OFFSET_DRAM_HEAD );
	tailByteOffset = *headByteOffsetPtr;

	int quit = 0, c = 0;
	double max = 0, max2 = 0, max3 = 0, max4 = 0, temp, temp2,temp3,temp4;
	while( !quit ){

		// Get a block of samples
		ns=get_sample_block( headByteOffsetPtr, &tailByteOffset, sample_bf );

		// convert samples, and sort into channels	
		for( idx=0;idx<ns;idx=idx+2 ){
			output = ((sample_bf[idx+1]&0x00FF)<<16)+sample_bf[idx];
			output2 = ((2.5*output)/8388608);
			if(sample_bf[idx+1]&0x0080){ output2 = output2-5;  }

		if((sample_bf[idx+1]&0xFF00) == 0x00){
			if(index < samples_to_fft){
				channel1[index] = output2;	
				index++;
			}
		}
		else if((sample_bf[idx+1]&0xFF00) == 0x2000){
			if(index < samples_to_fft){
				channel2[index] = output2;	
			}
		}
		else if((sample_bf[idx+1]&0xFF00) == 0x4000){
			if(index < samples_to_fft){
				channel3[index] = output2;	
			}
		}
		else if((sample_bf[idx+1]&0xFF00) == 0x6000){
			if(index < samples_to_fft){
				channel4[index] = output2;	
			}
		}
	}
	
	if(index==samples_to_fft){
		max = 0;
		max2 = 0;
		max3 = 0;
		max4 = 0;
		memset(tmp_fd, 0, (interpolation_size/2+1)*sizeof(fftw_complex));
		memset(tmp_fd2, 0, (interpolation_size/2+1)*sizeof(fftw_complex));
		memset(tmp_fd3, 0, (interpolation_size/2+1)*sizeof(fftw_complex));
		memset(tmp_fd4, 0, (interpolation_size/2+1)*sizeof(fftw_complex));

		//r(index=0;index<samples_to_fft;index++){
		//hannel2[index] *= channel1[index];
		//
		fftw_execute_dft_r2c(fft_plan, channel1, tmp_fd);
		fftw_execute_dft_r2c(fft_plan2, channel2, tmp_fd2);
		fftw_execute_dft_r2c(fft_plan3, channel3, tmp_fd3);
		fftw_execute_dft_r2c(fft_plan4, channel4, tmp_fd4);

		fprintf(gnuplotPipe,"plot '-' using 1:2 with lines, '-' using 1:3 with lines, '-' using 1:4 with lines, '-' using 1:5 with lines \n");
	
		for(index=0;index<samples_to_fft;index++){
			temp = sqrt(pow(tmp_fd[index][0],2)+pow(tmp_fd[index][1],2))/samples_to_fft;
			temp2 = sqrt(pow(tmp_fd2[index][0],2)+pow(tmp_fd2[index][1],2))/samples_to_fft;
			temp3 = sqrt(pow(tmp_fd3[index][0],2)+pow(tmp_fd3[index][1],2))/samples_to_fft;
			temp4 = sqrt(pow(tmp_fd4[index][0],2)+pow(tmp_fd4[index][1],2))/samples_to_fft;
			fprintf(gnuplotPipe, "%lf %lf %lf %lf %lf\n", (double)index, temp,temp2,temp3,temp4);
			if(temp > max) { max = temp; }
			if(temp2 > max2) { max2 = temp2; }
			if(temp3 > max3) { max3 = temp3; }
			if(temp4 > max4) { max4 = temp4; }
		}
		sec = time(NULL);
		printf("T=%ld C1 %f C2 %f C3 %f C4 %f\n",sec,max,max2,max3,max4);
		fprintf(gnuplotPipe,"e\n");
		fflush(gnuplotPipe);
		/*
		fftw_execute_dft_c2r(ifft_plan, tmp_fd, result);

			// find peaks and period
		p1 = -10.0; p2 = -10.0; peak_sum = 0.0; min_sum = 0.0; last_peak = 0.0; last_min = 0.0; 
		count_peak = -1; count_min = -1; count_period = -1; period_sum = 0; start = 0; first_peak = 1; first_min = 1; last_period = 0;

		for(index=0;index<samples_to_fft*interpolation_factor;index++){
			if((p1 < p2) && (p2 > result[index])){ 
				if(first_peak==0){ peak_sum += p2/samples_to_fft; count_peak++; last_peak = p2/samples_to_fft; }	
				else{ first_peak = 0; }
			}
			if((p1 > p2) && (p2 < result[index])){ 
				if(first_min == 0){ min_sum += p2/samples_to_fft; count_min++; last_min = p2/samples_to_fft; }
				else{ first_min = 0; }
			}
			if((p2 > 0) && (result[index] < 0)){
				if ((start != 0)&&(first_peak != 1) && (first_min != 1)){
					period_sum += (index-start); count_period++; last_period = (index-start);
				}
				start = index;
			}
			p1 = p2; p2 = result[index];
		}
		sec = time(NULL);
		printf("T=%ld Pk-Pk %f, Period %d\n",sec,(peak_sum-last_peak)/count_peak-(min_sum-last_min)/count_min, (period_sum-last_period)/count_period);
		*/	
		index = 0;
		memset(channel1, 0, (samples_to_fft+time_domain_zero_padding)*sizeof(double));
		memset(channel2, 0, (samples_to_fft+time_domain_zero_padding)*sizeof(double));
		memset(channel3, 0, (samples_to_fft+time_domain_zero_padding)*sizeof(double));
		memset(channel4, 0, (samples_to_fft+time_domain_zero_padding)*sizeof(double));
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
