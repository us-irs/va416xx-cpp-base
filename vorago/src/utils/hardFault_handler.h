#ifndef VORAGO_SRC_UTILS_HARDFAULT_HANDLER_H_
#define VORAGO_SRC_UTILS_HARDFAULT_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__)

extern void VOREXC_HardFault_Handler(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* VORAGO_SRC_UTILS_HARDFAULT_HANDLER_H_ */
