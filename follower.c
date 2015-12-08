// This program listens to the output of the ADC (via PRU0 and PRU1)
// And dumps the raw output to the stdout.



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

#include "mcf.h"

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
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 32
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 64
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 96
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 128
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 160
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 192
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 224
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 256
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 288
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 320
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 352
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 384
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 416
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 448
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 480
      " stmia	%[dst]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t" 
      " ldmia	%[src]!, { r3, r4, r5, r6, r7, r8, r9, r10 } \n\t"    // 512
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
    unsigned int          *tailOffsetPtr, 
    unsigned short        *dst )
{
    unsigned long long *llsrc,*lldst;
    unsigned int        mask;
    unsigned int        hoff,toff;
    int                 bytes_in_block = 4096;

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
    unsigned int           tailByteOffset;

    unsigned short         sample_bf[131072];
    unsigned short         r0,r2;
    unsigned short         actual;
    
    int r1;
    int                    ns;
    int                    errCount;
    int                    idx;
    int                    synchronized;
   int output;
	float output2;
    // Setup basic operating variables
    errCount       = 0;
    synchronized   = 0;
    headByteOffsetPtr = (unsigned int*)( 
                           (unsigned char*)pru0DataMem+PRU0_OFFSET_DRAM_HEAD );
    tailByteOffset = *headByteOffsetPtr;
    r1 = 0;
    while( errCount < MAX_PATTERN_ERRS ){
        // if( usDelay!=0 ) us_sleep( usDelay );

        // Get a block of samples
        ns=get_sample_block( headByteOffsetPtr, &tailByteOffset, sample_bf );
        // Loop over all of the samples available in this block and verify

	float out1,out2,out3,out4;
        for( idx=0;idx<ns;idx=idx+2 ){
            output = ((sample_bf[idx+1]&0x00FF)<<16)+sample_bf[idx];
            output2 = ((2.5*output)/8388608);
            if(sample_bf[idx+1]&0x0080){ output2 = output2-5; }
		if((sample_bf[idx+1]&0xFF00) == 0x00) out1 = output2;
		else if((sample_bf[idx+1]&0xFF00) == 0x2000) out2 = output2;
		else if((sample_bf[idx+1]&0xFF00) == 0x4000) out3 = output2;
		else if((sample_bf[idx+1]&0xFF00) == 0x6000) {
			out4 = output2; 
			printf("%f %f %f %f\n",out1,out2,out3,out4);
	   	} 
            // printf("%f\n",output2);
        }// End of loop over sample block
  
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
    // line 231 extram_base      is virt and mmapped
	
    prussdrv_map_extmem( &vptr ); 
    printf("V extram_base = 0x%08x\n",(unsigned int) vptr);
    ddrMem = vptr;

    follow_stream( usDelay );

    return(0);

}
