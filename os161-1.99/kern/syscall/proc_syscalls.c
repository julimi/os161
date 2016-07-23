#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>

#include "opt-A2.h"
#if OPT_A2
#include <limits.h>
#include <array.h>
#include <mips/trapframe.h>
#include <synch.h>
#include <pid.h>
#include <vfs.h>
#include <kern/fcntl.h>
#endif /* OPT_A2 */

#include "opt-A3.h"

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */
void sys__exit(int exitcode) {
  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;

#if OPT_A2
  pid_exit(p, exitcode);
#endif /* OPT_A2 */

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  *retval = 1;

#if OPT_A2
  *retval = curproc->pid;
#endif /* OPT_A2 */

  return(0);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */
  
#if OPT_A2
  // check if option valid
#endif /* OPT_A2 */
  if (options != 0) {
    return(EINVAL);
  }

#if OPT_A2
  // check if the status pointer is a valid pointer anyway
  if (status == NULL) {
    return(EFAULT);
  }

  // other constraint checking
  bool b = pid_wait(curproc, pid, &exitstatus, &result);
  if (b) {
    return(result);
  }
#else
  /* for now, just pretend the exitstatus is 0 */
  exitstatus = 0;
#endif /* OPT_A2 */
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}

#if OPT_A2
int
sys_fork(struct trapframe *tf, pid_t *retval)
{
	return pid_fork(tf, retval, curproc);
}

/*
 * 1st arg: progname
 * 2nd arg: uargs, an array of pointers, each pointer points to a user space string,
 * and the last pointer is NULL.
 */
int
sys_execv(char *progname, char **uargs)
{
	struct addrspace *as;
	struct vnode *v;
	vaddr_t entrypoint, stackptr;
	int result;
	int argc = 0;			// count of arguments in uargs

	/*
	 * count how many arguments in uargs,
	 */
	char **cuargs;
	for (int i = 0; uargs[i] != NULL; i++) {
		result = copyin((const_userptr_t)uargs[i],(void *)&cuargs,(strlen(uargs[i])+1)*sizeof(char));
		if (result) {
			return result;
		}
		argc++;
	}
	argc++;				// include the last NULL pointer
	if (argc > ARG_MAX) {
		return E2BIG;		// too many arguments
	}

	/*
	 * copy arguments into kernel buffer
	 */
	char **kargv = kmalloc(argc * sizeof(char *));
	for (int i = 0; i < argc-1; i++) {
		int klen = strlen(uargs[i]) + 1;
		kargv[i] = kmalloc(klen * sizeof(char));
		result = copyinstr((const_userptr_t)uargs[i],kargv[i],klen * sizeof(char),NULL);
		if (result) {
			return result;
		}
	}
	kargv[argc-1] = NULL;		// also put a NULL as terminator
	
	/*
	 * copy program into the kernel buffer
	 */
	int kplen = strlen(progname)+1;
	char *kp = kmalloc(kplen * sizeof(char));
	result = copyinstr((const_userptr_t)progname,kp,kplen*sizeof(char),NULL);
	if (result) {
		return result;
	}


	// Open the executable, create a new as and load elf into the kernel buffer
	/* Open the file. */
	result = vfs_open(kp, O_RDONLY, 0, &v);
	if (result) {
		return result;
	}

	/* Create a new address space. */
	as = as_create();
	if (as ==NULL) {
		vfs_close(v);
		return ENOMEM;
	}

	/* Switch to it and activate it. */
	curproc_setas(as);
	as_activate();

	/* Load the executable. */
	result = load_elf(v, &entrypoint);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		vfs_close(v);
		return result;
	}

	/* Done with the file now. */
	vfs_close(v);


	// copy the arguments from kernel buffer into user stack
	/* use as_define_stack to get the value of initial stack pointer */
	result = as_define_stack(as, &stackptr);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		return result;
	}

	userptr_t cptr[argc];			// store new stackptr
	// fill kargs[i] with actual user space pointer
	for (int i = 0; i < argc-1; i++) {
		// padding, require alignment are 8 bypes long
		int nlen = ROUNDUP(strlen(kargv[i])+1,8);
		stackptr -= nlen;
		cptr[i] = (userptr_t)stackptr;
		result = copyoutstr(kargv[i],(userptr_t)stackptr,nlen * sizeof(char),NULL);
		if (result) {
			return result;
		}
	}
	cptr[argc-1] = NULL;

	// minus stackptr by the length of kargv
	int spsize = ROUNDUP(argc*sizeof(char **),8);
	stackptr -= spsize;
	// return the stackptr to user
	result = copyout(cptr, (userptr_t)stackptr, spsize);
	if (result) {
		return result;
	}

	// return user mode using enter_new_process
	/* Warp to user mode. */
	enter_new_process(argc-1,(userptr_t)stackptr,stackptr, entrypoint);
	panic("enter_new_process returned\n");
	return EINVAL;
}
#endif /* OPT_A2 */
