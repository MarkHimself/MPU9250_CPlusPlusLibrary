#include "SPI_ArduinoDue.h"

/*
SPI0 is instance i.d. 24
Master Mode.
8 bits per transfer (SAM3x8e supports from 8 to 16 bits - in case you want/need to use more)
chip select controlled through software - The SAM3X8E does offer SPI controlled nCS pins but that
requires the use of interrupts/DMA to control SPI transactions.
*/

void setup_SPI0_Master(uint8_t spi_mode){
	uint32_t temp = 0;
	
	PMC->PMC_PCER0 = PMC_PCER0_PID24;	// clock for SPI0						pg. 679, 38, 542
	SPI0->SPI_WPMR = 0x535049;			// unlock write protect					pg. 706
	temp = SPI0->SPI_WPSR;				// clear status reg.					pg. 707
	SPI0->SPI_CR = SPI_CR_SWRST;		// reset SPI0							pg. 693
	
	
	if ((spi_mode == 2) | (spi_mode == 3)) temp = SPI_CSR_CPOL;
	if ((spi_mode == 0) | (spi_mode == 2)) temp |= SPI_CSR_NCPHA;
	
	// i'm going to control the chip select pin through software so this is generic. but important for the spi mode.
	SPI0->SPI_CSR[0] = 0			// chip select register						pg. 703
		| temp						// spi mode
		| SPI_CSR_BITS_8_BIT		// 8 bits
		| SPI_CSR_SCBR(84)			// 84MHz / 84 = 1 MHz baud rate for SPI clock
	;
	
	SPI0->SPI_MR = 0				// mode register							pg. 694
		| SPI_MR_MSTR				// master mode
		| SPI_MR_MODFDIS			// mode fault disable
	;
	
	temp = SPI0->SPI_SR;			// clear status register					pg. 698
	
	SPI0->SPI_CR = SPI_CR_SPIEN;		// enable SPI0
}

// if frequency is 0, fail.
// frequency shall be between 329 KHz and 84000 KHz
bool SPI0_Clock_Rate_khz(uint16_t f){
	
	uint8_t scbr = 0;
	
	if (f == 0) return false;
	
	// verify that no data is being transmitted.
	if (!(SPI0->SPI_SR & SPI_SR_TXEMPTY)){	// if transmitter buffers are not empty		pg. 698
		return false;
	}
	
	scbr = 84000 / f;
	if (scbr == 0) {
		return false;
	}
	
	SPI0->SPI_CSR[0] = SPI0->SPI_CSR[0] & (~SPI_CSR_SCBR(0xFF));	// clear the baud rate	pg. 703
	SPI0->SPI_CSR[0] |= SPI_CSR_SCBR(scbr);	// set the new baud rate			pg. 703
	return true;
}

// MISO is Peripheral A
// Arduino Digital Pin D74
void setup_PIOA25_as_SPI0_MISO(){
	// PIOA is instance i.d. 11													pg. 38
	PMC->PMC_PCER0 = PMC_PCER0_PID11;	// enable clock for PIOA				pg. 542
	PIOA->PIO_WPMR = 0x50494F;			// unlock write protect					pg. 674
	
	PIOA->PIO_PUDR = PIO_PUDR_P25;		// disable pull up						pg. 653
	PIOA->PIO_ABSR = PIOA->PIO_ABSR & (~PIO_ABSR_P25);	// 0 = Peripheral A		pg. 656
	PIOA->PIO_PDR = PIO_PDR_P25;	// disable PIO control of pin. (Peripheral controls it)	pg. 634
}

// MOSI is Peripheral A
// Arduino Digital Pin D75
void setup_PIOA26_as_SPI0_MOSI(){
	// PIOA is instance i.d. 11													pg. 38
	PMC->PMC_PCER0 = PMC_PCER0_PID11;	// enable clock for PIOA				pg. 542
	PIOA->PIO_WPMR = 0x50494F;			// unlock write protect					pg. 674
	
	PIOA->PIO_PUDR = PIO_PUDR_P26;		// disable pull up						pg. 653
	PIOA->PIO_ABSR = PIOA->PIO_ABSR & (~PIO_ABSR_P26);	// 0 = Peripheral A		pg. 656
	PIOA->PIO_PDR = PIO_PDR_P26;	// disable PIO control of pin. (Peripheral controls it)	pg. 634
}

// SCK is Peripheral A
// Arduino Digital Pin D76
void setup_PIOA27_as_SPI0_SCK(){
	// PIOA is instance i.d. 11													pg. 38
	PMC->PMC_PCER0 = PMC_PCER0_PID11;	// enable clock for PIOA				pg. 542
	PIOA->PIO_WPMR = 0x50494F;			// unlock write protect					pg. 674
	
	PIOA->PIO_PUDR = PIO_PUDR_P27;		// disable pull up						pg. 653
	PIOA->PIO_ABSR = PIOA->PIO_ABSR & (~PIO_ABSR_P27);	// 0 = Peripheral A		pg. 656
	PIOA->PIO_PDR = PIO_PDR_P27;	// disable PIO control of pin. (Peripheral controls it)	pg. 634
}

void setup_PIOA28_as_cs_digital(){
	// PIOA is instance i.d. 11													pg. 38
	PMC->PMC_PCER0 = PMC_PCER0_PID11;	// enable clock for PIOA				pg. 542
	PIOA->PIO_WPMR = 0x50494F;			// unlock write protect					pg. 674
	
	PIOA->PIO_PUDR = PIO_PUDR_P28;		// disable pull up						pg. 653
	PIOA->PIO_OER = PIO_OER_P28;		// set to output
	PIOA->PIO_PER = PIO_PER_P28;		// enable PIO control of pin. 			pg. 633
	PIOA->PIO_SODR = PIO_SODR_P28;		// set HIGH
}


uint8_t write_SPI0_blocking(uint8_t val){
	
	SPI0->SPI_TDR = val;			// transmit data through SPI				pg. 697
	
	// if SPI is not enabled, return 0 (avoid waiting forever for transaction to finish)
	if (!(SPI0->SPI_SR & SPI_SR_SPIENS)){	// check if SPI0 is not enabled.	pg. 698
		return 2;
	}
	
	// wait until the receive register is not empty (wait until it's full)
	while (!(SPI0->SPI_SR & SPI_SR_RDRF)){	// 									pg. 698
		// wait.
	}
	
	return SPI0->SPI_RDR;			// return the read data						pg. 696
}

// this is the chip select for SPI.
// it's just digital output pin 10 on Arduino Due but i'm using low level registers to set it up.
void spi_cs_imu_due(bool high){
	if (high){
		PIOA->PIO_SODR = PIO_SODR_P28;
	}
	else{
		PIOA->PIO_CODR = PIO_CODR_P28;
	}
}






