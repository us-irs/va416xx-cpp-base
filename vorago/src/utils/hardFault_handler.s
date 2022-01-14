;/***************************************************************************/
;/***************************************************************************/

              AREA    |.text|, CODE, READONLY
                  
;// HardFault handler wrapper in assembly language.
;// It extracts the location of stack frame and passes it to the handler written
;// in C as a pointer. We also extract the LR value as second parameter.
hfhandler_asm PROC
              EXPORT  hfhandler_asm             [WEAK]
              IMPORT  HardFault_Handler_C
	          TST    LR, #4
	          ITE    EQ
	          MRSEQ  R0, MSP
	          MRSNE  R0, PSP
	          MOV    R1, LR
	          B      HardFault_Handler_C
              ENDP
                  
              END