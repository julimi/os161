#ifndef _COREMAP_H_
#define _COREMAP_H_

#include "opt-A3.h"

#if OPT_A3
#include <types.h>
#include <lib.h>
#include <vm.h>
#include <spinlock.h>
#include <synch.h>

// pack a physical page's info
struct coremap_entry {
	// where is page mapped
	paddr_t paddr;
	paddr_t kvaddr;
	uint32_t sl;		// segment length
	bool isDirty;		// pages status
};

void coremap_bootstrap(void);
paddr_t coremap_getppages(unsigned long npages);
void coremap_free_kpages(vaddr_t addr);
#endif /* OPT_A3 */
#endif /* _COREMAP_H_ */
