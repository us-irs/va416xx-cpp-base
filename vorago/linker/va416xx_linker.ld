/******************************************************************************
 * This linker file was developed from the example file included in the GCC download.
 *  <..>\GCC_install\gcc-arm-none-eabi-7-2017-q4-major-win32\share\gcc-arm-none-eabi\samples\ldscripts
 * 
 * The author is not responsible for any
 * damage or liability that this file might cause.
******************************************************************************/
 
 /*   Modified for VA416xx MCU: 
            Cortex M4 based  
            256k byte of Instruction space RAM at 0x0000:0000  
            32k byte of Data space RAM at 0x1FFF:8000
            
            Part boots from external SPI NVM.  
            For code development the debugger loads RAM directly. 
*/
 
/* Entry Point */
ENTRY(Reset_Handler)
 
/* Specify the memory areas */
MEMORY
{
	ROM (rx)        : ORIGIN = 0x00000000, LENGTH = 256K
	RAM (xrw)       : ORIGIN = 0x1FFF8000, LENGTH = 32K
	SRAM1 (xrw)     : ORIGIN = 0x20000000, LENGTH = 32K
}
 
/* define stack size and heap size here */
stack_size = 1024;  
heap_size = 512;
 
/* define beginning and ending of stack */
__StackTop = ORIGIN(RAM) + LENGTH(RAM) - 4;
_stack_end  =__StackTop - stack_size;
 
/* Define output sections */
SECTIONS
{
 	.text :
	{
		. = ALIGN(4); 
		KEEP(*(.vectors))
		*(.text*)
		
		
		KEEP(*(.init))
		KEEP(*(.fini))
		
		*(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
		KEEP(*(.eh_frame*))
	} > ROM

	.preinit_array ALIGN(4): {
        __preinit_array_start = .;
        KEEP(*(.preinit_array))
        __preinit_array_end = .;
    	} >ROM

    .init_array ALIGN(4): {
        __init_array_start = .;
        KEEP(*(.init_array))
        __init_array_end = .;
    } >ROM

    .fini_array ALIGN(4): {
        __fini_array_start = .;
        KEEP(*(.fini_array))
        __fini_array_end = .;
    } >ROM
    
 	/* used for unwinding, can be useful for debugging  */  
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >ROM
   
    /* used for unwinding, can be useful for debugging  */
	__exidx_start = .;
    .ARM.exidx (NOLOAD) :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } >ROM
    __exidx_end = .;

    . = ALIGN(4);
    __etext = .;  
 
  	/* can be used by the startup to initialize data */
 
    _sidata = .;   
	
	 .RTTSection : AT ( _sidata ) {
		. = ALIGN(4);
		_sdata = .;        		/* create a global symbol at data start */
		/* Place SEGGER-RTT first at 0x1fff8000 */
	/*	KEEP(*(.RTTSection)); */
	} >RAM

  	/* Initialized data sections goes into RAM, load LMA copy after code */
    .data : AT ( _sidata + SIZEOF(.RTTSection) )
  	{
    	. = ALIGN(4);
    	*(.data)           		/* .data sections */
    	*(.data*)          		/* .data* sections */
    	. = ALIGN(4);
    	_edata = .;        		/* define a global symbol at data end */
  	} >RAM  
 	
    /* Uninitialized data section */
    . = ALIGN(4);
    .bss :  /*  basic service set:  used for statically-allocated variables  */  
    {
        /*  Used by the startup in order to initialize the .bss section */
        _sbss = .;         /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
 
        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end */
        __bss_end__ = _ebss;
    } >RAM
 
	/* Heap section */
    . = ALIGN(4);
    .heap :
    {
    	_sheap_ = .;
    	_heap_start = .;
    	end = _heap_start;
    	_end = end;
    	__end = end; 	
    } > RAM
    
    /* Place symbol at heap end */
    _eheap_ = 0x1fff5000;
    _sstack = 0x20000000;
    /* This symbol is used by the start-up file ! */
    __StackTop = _sstack;

    .dma_sec (NOLOAD) :
    {
        . = ALIGN(4);
        /* Place DMA block at start of SRAM1 */
        KEEP(*(.dma_blk));
    } >SRAM1

    /* Remove information from the standard libraries */

    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
