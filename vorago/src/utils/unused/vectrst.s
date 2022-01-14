;/***************************************************************************/
;/***************************************************************************/
;/* vector_reset()
; * Jump to reset vector
; */

              AREA    |.text|, CODE, READONLY

vector_reset  PROC
              EXPORT  vector_reset             [WEAK]
              LDR R0,=0xE000ED08 ; Set R0 to VTOR address
              LDR R1,[R0] ; Load VTOR
              LDR R0,[R1] ; Load initial MSP value
              MOV SP, R0 ; Set SP value (assume MSP is selected)
              LDR R0,[R1, #4] ; Load reset vector
              BX R0 ; Branch to reset handler
              ENDP
                  
              END