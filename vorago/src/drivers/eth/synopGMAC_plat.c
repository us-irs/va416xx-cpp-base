/* ===================================================================================
 * Copyright (c) <2009> Synopsys, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software annotated with this license and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * =================================================================================== */

/**\file
 *  This file defines the wrapper for the platform/OS related functions
 *  The function definitions needs to be modified according to the platform
 *  and the Operating system used.
 *  This file should be handled with greatest care while porting the driver
 *  to a different platform running different operating system other than
 *  Linux 2.6.xx.
 * \internal
 * ----------------------------REVISION HISTORY-----------------------------
 * Synopsys			01/Aug/2007			Created
 */

#include "drivers/eth/synopGMAC_plat.h"

/**
 * This is a wrapper function for Memory allocation routine. In linux Kernel
 * it it kmalloc function
 * @param[in] bytes in bytes to allocate
 */

void *plat_alloc_memory(u32 bytes) {
  // return kmalloc((size_t)bytes, GFP_KERNEL);
  return malloc((size_t)bytes);
}

/**
 * This is a wrapper function for consistent dma-able Memory allocation routine.
 * In linux Kernel, it depends on pci dev structure
 * @param[in] bytes in bytes to allocate
 */

void *plat_alloc_consistent_dmaable_memory(struct pci_dev *pcidev, u32 size, u32 *addr) {
  // return (pci_alloc_consistent (pcidev,size,addr)); // TA
  return malloc((size_t)size);
}

/**
 * This is a wrapper function for freeing consistent dma-able Memory.
 * In linux Kernel, it depends on pci dev structure
 * @param[in] bytes in bytes to allocate
 */

void plat_free_consistent_dmaable_memory(struct pci_dev *pcidev, u32 size, void *addr,
                                         u32 dma_addr) {
  // pci_free_consistent (pcidev,size,addr,dma_addr); // TA
  free(addr);
  return;
}

/**
 * This is a wrapper function for Memory free routine. In linux Kernel
 * it it kfree function
 * @param[in] buffer pointer to be freed
 */
void plat_free_memory(void *buffer) {
  // kfree(buffer); //TA
  free(buffer);
  return;
}

/**
 * This is a wrapper function for platform dependent delay
 * Take care while passing the argument to this function
 * @param[in] buffer pointer to be freed
 */
void plat_delay(u32 delay) {
  while (delay--)
    ;
  return;
}
