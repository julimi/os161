
#include <pid.h>

#include "opt-A3.h"
//#include <synch.h>

struct semaphore *pid_sem;
struct lock *pid_lock;
struct cv *pid_cv;
struct array *proc_array;	// array of proc
struct array *pid_array;	// array of pid
pid_t nextpid;			//  next new pid

struct
pidinfo *pidinfo_create(pid_t pid, pid_t ppid)
{
	struct pidinfo *pidinfo;
	pidinfo = kmalloc(sizeof(*pidinfo));
	pidinfo->pid = pid;
	pidinfo->ppid = ppid;
	pidinfo->exitcode = 0;
	pidinfo->cexited = false;
	pidinfo->pexited = false;

	return pidinfo;
}

void
pid_bootstrap(void)
{
	pid_sem = sem_create("pid_sem", 1);
	if (pid_sem == NULL) {
		panic("could not create pid_sem semaphore\n");
	}

	pid_lock = lock_create("pid_lock");
	if (pid_lock == NULL) {
		panic("could not create pid_lock lock\n");
	}

	pid_cv = cv_create("pid_cv");
	if (pid_cv == NULL) {
		panic("could not create pid_cv condition variable");
	}

	proc_array = array_create();
	pid_array = array_create();

	// leave pid = 0 alone, for MACROS, WAIT_MYPGRP(0)
	nextpid = 1;
}

void
pid_check_pid(struct proc *proc)
{
	P(pid_sem);

	if (array_num(pid_array) == 0) {
		// allocate a new pid and set it for this process
		nextpid++;
		proc->pid = nextpid;

	}else{
		// exist pid, reuse it, then remove it from the array
		proc->pid = *((pid_t *) array_get(pid_array, 0));
		kfree(array_get(pid_array, 0));
		array_remove(pid_array, 0);
	}

	V(pid_sem);
}

bool
pid_wait(struct proc *proc, pid_t pid, int *exitstatus, int *result)
{
	bool b;
	lock_acquire(pid_lock);

	unsigned np = array_num(proc_array);
	b = false;
	
	for (unsigned i = 0; i < np; i++) {
		struct pidinfo *pidinfo = (struct pidinfo *) array_get(proc_array, i);
		
		// check does the waited pid exist
		if (pidinfo->pid == pid) {
			// if valid, check if we are allowed to wait it
			if (proc->pid == pidinfo->ppid) {
				// wait for children processing
				while (!pidinfo->cexited) {
					cv_wait(pid_cv, pid_lock);
				}

				KASSERT(pidinfo->cexited);
				*exitstatus = pidinfo->exitcode;
				b = true;
				break;
			} else {
				lock_release(pid_lock);
				*result = ECHILD;
				return true;
			}
		}
	}
	
	lock_release(pid_lock);
	
	// fail to search
	if (!b) {
		*result = ESRCH;
		return true;
	}

	return false;

}

void
pid_exit(struct proc *proc, int exitcode)
{
	lock_acquire(pid_lock);
	
	unsigned np = array_num(proc_array);
	for (unsigned i = 0; i < np; i++) {
		struct pidinfo *pidinfo = (struct pidinfo *) array_get(proc_array, i);

		// check if this is a parent process
		if (proc->pid == pidinfo->ppid) {
			// set exit
			// we don't need to fill the  exitcode here,
			// because no one cares
			pidinfo->pexited = true;
			// children exit as well, reuse its pid
			if (pidinfo->cexited) {
				P(pid_sem);

				pid_t *newpid = kmalloc(sizeof(pid_t));
				*newpid = pidinfo->pid;
				array_add(pid_array, (void *) newpid, NULL);

				V(pid_sem);

				// free the original one
				kfree(pidinfo);

				array_remove(proc_array, i);
				np--;				
				// this index will be replaced by other, so decrement
				i--;

			}
			
		}
		// check if this is a children process
		else if (proc->pid == pidinfo->pid) {
			pidinfo->cexited = true;
			// its parent also exit
			if (pidinfo->pexited) {
				P(pid_sem);

				pid_t *newpid = kmalloc(sizeof(pid_t));
				*newpid = pidinfo->pid;
				array_add(pid_array, (void *) newpid, NULL);
			
				V(pid_sem);

				kfree(pidinfo);

				array_remove(proc_array, i);
				np--;
				i--;
			} 
			// its parent still processing
			else {
				// set the real exitcode
#if OPT_A3
				pidinfo->exitcode = exitcode;
#else
				pidinfo->exitcode = _MKWAIT_EXIT(exitcode);
#endif /* OPT_A3 */
				cv_broadcast(pid_cv, pid_lock);
			}
		}
	}

	lock_release(pid_lock);
}

int
pid_fork(struct trapframe *tf, pid_t *retval, struct proc *proc)
{
	// create a new address space for new process
	struct addrspace *nas;
	// create a new process
	struct proc *nproc = proc_create_runprogram(proc->p_name);
	
	// check if new process is valid
	if (nproc == NULL) {
		return(ENOMEM);
	}

	// check if pid proceed its maximum
	if (nproc->pid > PID_MAX) {
		proc_destroy(nproc);
		P(pid_sem);
		
		nextpid--;
		
		V(pid_sem);
		
		// coz now doing system call 
		return(ENPROC);
	}

	// copy parent's trap frame, and pass it to children
	struct trapframe *ntf = kmalloc(sizeof(*ntf));
	// check the new trapframe is valid
	if (ntf == NULL) {
		P(pid_sem);
		
		nextpid--;

		V(pid_sem);
		proc_destroy(nproc);
		return(ENOMEM);
	}
	memcpy(ntf, tf, sizeof(*ntf));

	// copy parent's address space
	// for match the error number
	int errno = as_copy(proc->p_addrspace, &nas);
	if (errno == ENOMEM) {
		P(pid_sem);
		
		nextpid--;
		
		V(pid_sem);
		proc_destroy(nproc);
		return(errno);
	}
	nproc->p_addrspace = nas;

	// add the new process with pid into the proc_array
	lock_acquire(pid_lock);

	struct pidinfo *pidinfo = pidinfo_create(nproc->pid, proc->pid);
	array_add(proc_array, pidinfo, NULL);

	lock_release(pid_lock);

	// create child thread
	errno = thread_fork("thread_fork", nproc, enter_forked_process, (void *) ntf, (int) nproc->pid);
	// check if forked thread is valid
	if (errno == ENOMEM) {
		P(pid_sem);
		
		nextpid--;

		V(pid_sem);
		proc_destroy(nproc);
		return(errno);
	}

	// parent returns with child's pid immediately
	*retval = nproc->pid;

	// child returns with 0
	return(0);
}
