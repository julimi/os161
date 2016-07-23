#include "opt-A3.h"

#if OPT_A3

#include <coremap.h>

// use an array of struct coremap_entry to keep all physical pages info
static struct coremap_entry *coremap_array;
static struct spinlock stealmem_lock = SPINLOCK_INITIALIZER;
// protect every access to coremap using lock
static struct spinlock coremap_lock = SPINLOCK_INITIALIZER;
static uint32_t firstpage;
static uint32_t lastpage;
static bool vm_got = false;
static paddr_t firstaddr;
static paddr_t lastaddr;

void
coremap_bootstrap(void)
{
	uint32_t npages, size;
	
	/*
	 * because ram_getsize will destroy its firstaddr and lastaddr before return,
	 * we should initialize all other data structures before we call ram_getsize
	 */
	// find out how many physical pages in system
	ram_getsize(&firstaddr, &lastaddr);
	npages = (lastaddr - firstaddr) / PAGE_SIZE;
	
	size = ROUNDUP(npages*sizeof(struct coremap_entry), PAGE_SIZE);

	coremap_array = (struct coremap_entry *)PADDR_TO_KVADDR(firstaddr);
	
	// current addr and number of pages
	firstaddr += size;
	firstpage = firstaddr / PAGE_SIZE;
	lastpage = lastaddr / PAGE_SIZE;
	npages = lastpage - firstpage;
	// initiate the coremap

	for (uint32_t i = 0; i < npages; i++) {
		coremap_array[i].paddr = firstpage*PAGE_SIZE + i*PAGE_SIZE;
		coremap_array[i].kvaddr = PADDR_TO_KVADDR(coremap_array[i].paddr);
		coremap_array[i].sl = 0;
		coremap_array[i].isDirty = false;
	}

	// initialization is done
	vm_got = true;
	return;
}

paddr_t
coremap_getppages(unsigned long npages)
{
	paddr_t addr;
	if (!vm_got) {
		spinlock_acquire(&stealmem_lock);
		
		addr = ram_stealmem(npages);

		spinlock_release(&stealmem_lock);
	} else {
		spinlock_acquire(&coremap_lock);

		uint32_t begin = 0;
		uint32_t index = 0;
		// is the page status is fixed
		bool isFix = false;

		for (uint32_t i = 0; i < lastpage-firstpage; i++) {
			if (coremap_array[i].isDirty) {
				begin = index = 0;
				isFix = false;
			} else {
				index++;
				
				if (!isFix) {
					begin = i;
					isFix = true;
				}
				
				// get pages we need
				if (index == npages) {
					for (uint32_t j = begin; j < begin+npages; j++) {
						coremap_array[j].sl = npages - j + begin;
						coremap_array[j].isDirty = true;
					}

					// got it
					spinlock_release(&coremap_lock);
					return coremap_array[begin].paddr;
				}
			}
		}

		// now here, it means fail to get free pages
		spinlock_release(&coremap_lock);
		panic("No free pages\n");
	
	}

	return addr;
}

void
coremap_free_kpages(vaddr_t addr)
{
	if (vm_got) {
		spinlock_acquire(&coremap_lock);

		uint32_t index, size;
		index = (addr - MIPS_KSEG0) / PAGE_SIZE - firstpage;
		size = index + coremap_array[index].sl;

		// unmap the coremap
		for (uint32_t i = index; i < size; i++) {
			coremap_array[i].sl = 0;
			coremap_array[i].isDirty = false;
		}

		spinlock_release(&coremap_lock);
	}

}
#endif /* OPT_A3 */
