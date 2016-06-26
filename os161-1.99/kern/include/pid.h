#ifndef _PID_H_
#define _PID_H_

#include "opt-A2.h"
#if OPT_A2
#include <types.h>
#include <kern/errno.h>
#include <limits.h>
#include <kern/wait.h>
#include <array.h>
#include <proc.h>
#include <mips/trapframe.h>
#include <synch.h>
#include <addrspace.h>
#include <syscall.h>

/*struct semaphore *pid_sem;
struct lock *pid_lock;
struct cv *pid_cv;
struct array *proc_array;	// array of proc
struct array *pid_array;	// array of pid
pid_t nextpid;			//  next new pid
*/
struct
pidinfo
{
	pid_t pid;	// children pid
	pid_t ppid;	// parent pid
	int exitcode;
	bool cexited;	// if children exited
	bool pexited;	// if parent exited
	//struct cv *pid_cv;
};

struct pidinfo *pidinfo_create(pid_t pid, pid_t ppid);
void pid_bootstrap(void);
void pid_check_pid(struct proc *proc);
bool pid_wait(struct proc *proc, pid_t pid, int *exitstatus, int *result);
void pid_exit(struct proc *proc, int exitcode);
int pid_fork(struct trapframe *tf, pid_t *retval, struct proc *proc);

#endif /* OPT_A2 */
#endif /* _PID_H_ */
