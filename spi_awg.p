// This routine is the assembly code for bit-banging an SPI port to talk to the AWG

.origin 0
.entrypoint START

#define PRU0_ARM_INTERRUPT 19

#define GPIO1 			0x4804c000		// The adress of the GPIO1 
#define GPIO_DATAOUT 	0x13c				// This is the register for settign data
#define LEN_ADDR		0x00000000		// Adress of the abort command			
#define PIN_OFFSET		0x00000004  		// Offset for the pins (reserve the first adress for abort)

#define data r2
#define bits r9
#define delay r10
#define rSamp0       r13


START:	
    // MOV r30.b0, 0x40 // 8 4 2 1 8 4 2 1 
	SET r30.t6 
    MOV rSamp0, 0x0000
    MOV r10, 0xFF000000
    // MOV r30.b1, 0x00
    MOV  r0, 0x0120
    MOV  r1, 0x00022028
    SBBO r0, r1, 0, 4
    MOV  r1, 0x00024028


    SBBO r0, r1, 0, 4
    LBCO r0, C4, 4, 4					// clear that bit
    CLR  r0, r0, 4					// No, really, clear it!
    SBCO r0, C4, 4, 4					// Actually i have no idea what this does
    MOV  r0, 0						// The first register contains the loop count
    LBBO data, r0, 0, 4					// Load r1 with first data value

    MOV bits, 0x00000000
    AND r1, data, r10
    QBEQ WRITE, r1, 0 					// if r1 == 0 go to write mode 
    QBA READ
WRITE:
    // LBBO data, r0, 0, 4  // copy data over
    MOV bits, 32
    QBA SEND
READ:
    // LBBO data, r0, 0, 2 // copy data over
    MOV bits, 16

SEND:
    MOV r15, 0x80000000

    loop_start:
        MOV delay, 34000 // 34000
        // prepare data out
        AND r13, data, r15
	lsr r13, r13, 31
        lsl data, data, 1
	AND r13, r13, 0x01
	MOV r30.b1, r13
	// MOV r30.b1, 0xFF
        SUB bits, bits, 1

        delay_hi:
                SUB delay, delay, 1
                QBNE delay_hi, delay, 0
	MOV delay, 34000
        // or r30.b1, r30.b1, 2
	OR r30.b1, r30.b1, 0x02 // clk high
        // AND r30.b1, r13, 0xFD // clk low
	// MOV r30.b1, 0x00
        delay_low:
                SUB delay, delay, 1
                QBNE delay_low, delay, 0
        QBNE loop_start, bits, 0

	
     QBNE READ2, r1, 0
     QBA FINISH
READ2:
     MOV bits, 16
		MOV rSamp0, 0x00000000
	
	SPICLK_BIT:                     // loop for each of the 24 bits
		MOV r30.b1, 0x02 // 0x03
		MOV delay, 34000
		CLKH2:	
			SUB delay, delay, 1
			QBNE CLKH2, delay, 0
		MOV r30.b1, 0x00
		MOV delay, 34000
		CLKL2:
			SUB delay, delay, 1
			QBNE CLKL2, delay, 0

		// This is the 24 bit SPI loop 
		SUB bits, bits, 1        // count down through the bits
		QBBC DATAINLOW, r31.t10
			OR rSamp0, rSamp0, 0x00000001
			QBA NEXT
		DATAINLOW:
			OR rSamp0, rSamp0, 0x00000000
			QBA NEXT
		NEXT:
		LSL rSamp0, rSamp0, 1
		QBNE SPICLK_BIT, bits, 0
		LSR rSamp0, rSamp0, 1

FINISH:
    MOV r30.b1, 0x01
   // MOV r30.b0, 0x40 // 8 4 2 1 8 4 2 1 
SET r30.t6
    MOV r0, 0
    // MOV rSamp0, 0xFFFF	
    SBCO rSamp0, c28, 0, 4                  // Place sequence in local PRU 0 ram    

    // MOV data, 0x12312312
    // MOV r0, data               // r0 = other knwon, strange sequence
    // SBCO r0, c25, 0, 4                   // Place sequence in local PRU 0 ram    

    // MOV r0, bits            	 // r0 = known, strange sequence
    // SBCO r0, c28, 0, 4                   // Place the sequence in shared ram 




    MOV R31.b0, PRU0_ARM_INTERRUPT+16   // Send notification to Host for program completion
HALT
