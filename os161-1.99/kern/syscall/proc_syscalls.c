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
#include <synch.h>
#include <../arch/mips/include/trapframe.h>
#include <vfs.h>
#include <kern/fcntl.h>

#if OPT_A2
const int PROC_MAX = 64;
volatile int proc_count = 0;
struct lock *proc_count_lk = NULL;
#endif //OPT_A2


/* this implementation of sys__exit does not do anything with the exit code */
/* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

#if OPT_A2
	lock_acquire(curproc->p_lk);
	struct addrspace *as;
  	struct proc *p = curproc;

  	KASSERT(curproc->p_addrspace != NULL);
  	as_deactivate();

  	as = curproc_setas(NULL);
  	as_destroy(as);

  	/* detach this thread from its process */
  	/* note: curproc cannot be used after this call */
  	proc_remthread(curthread);

    // store exit code	
    p->exitcode = exitcode;

  	// empty children array; 
  	int size_children = array_num(p->children);
  	for(int i=size_children-1; i>=0; --i) {
  		struct proc *c = array_get(p->children, i);
  		lock_acquire(c->p_lk);
  		if(c->isZombie) {
  			// fully delete zombie procs
  			proc_cleanup(c);
  		} else {
			// make living children orphans
  			c->parent = NULL;
  			lock_release(c->p_lk);
  		}
  		remove_child(p, i);
  	}

 	// if parent is NULL, can fully delete
 	if(p->parent == NULL) {
 		proc_cleanup(p);
 		thread_exit();
 	} else {
 	// if parent is alive we turn into a zombie, but do not cleanup now
		p->isZombie = true;
 		cv_signal(p->has_exited, p->p_lk);
 		lock_release(p->p_lk);
 		thread_exit();
 	}

#else
  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;

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
#endif // OPT_A2
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
#if OPT_A2
	pid_t pid = curproc->pid;
	*retval = pid;
	return(0);
#else
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  *retval = 1;
  return(0);
#endif // OPT_A2

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

  if (options != 0) {
    return(EINVAL);
  }

#if OPT_A2
  	// check if proc is non existent 
  	lock_acquire(proc_count_lk);
  	if(pid > proc_count) { return(ESRCH); }
  	lock_release(proc_count_lk);

	// check if parent is interested in child's pid 
 	lock_acquire(curproc->p_lk);
 	int size = array_num(curproc->children);
 	for(int i=0; i < size; ++i) {
 		struct proc *child = array_get(curproc->children, i);
 		lock_acquire(child->p_lk);	
 		if(child->pid == pid) { 
 			// child found; parent is interested
 			remove_child(curproc, i);
 			lock_release(curproc->p_lk);
 			while(1) {
 				if(child->isZombie) {
 					// retrieve exit code, destroy child and return
 					exitstatus = _MKWAIT_EXIT(child->exitcode);
 					proc_cleanup(child);
 					result = copyout((void *)&exitstatus,status,sizeof(int));
  					if (result) { return(result); }
 					break;
 				} else { 
 					// block parent until child proc has exited
 					cv_wait(child->has_exited, child->p_lk);
 				}
 			}
  			*retval = pid;
  			return(0);
 		}
 		lock_release(child->p_lk);
 	} // child not found 
 	lock_release(curproc->p_lk);
 	return(ECHILD);

#else
  /* for now, just pretend the exitstatus is 0 */
  exitstatus = 0;
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
#endif /* OPT_A2 */
}

#if OPT_A2
int sys_fork(struct trapframe *tf, int32_t *retval) {
	/* make a deep copy the parent's trapframe onto the heap */
	struct trapframe *tf_kheap = (struct trapframe*) kmalloc(sizeof(*tf));
	*tf_kheap = *tf;

	/* make sure we haven't exceeded the proc limit */
	if(proc_count_lk == NULL) {
		proc_count_lk = lock_create("proc_count_lk");
	}
	lock_acquire(proc_count_lk);
	if(proc_count > PROC_MAX) {
		return(EMPROC);
	} else { ++proc_count; }
	lock_release(proc_count_lk);

	/* create child proc structure */
	struct proc *childproc = NULL;
	childproc = proc_create_runprogram("child_proc");
	if(childproc == NULL) {
		return(EMPROC);
	}

	/* copy address space */
	struct addrspace *newas;
	int result = as_copy(curproc->p_addrspace, &newas);
	if(result != 0) {
		return(ENOMEM);
	}
	/* set child's address space */
	spinlock_acquire(&childproc->p_lock);
	childproc->p_addrspace = newas;
	spinlock_release(&childproc->p_lock);

	/* create parent-child relationship */
	lock_acquire(curproc->p_lk);
	add_child_proc(childproc, curproc);
	lock_release(curproc->p_lk);
	lock_acquire(childproc->p_lk);
	childproc->parent = curproc;
	
	/* assign pid */
	pid_t pid_childproc = pid_counter;
	++pid_counter;
	childproc->pid = pid_childproc;
	lock_release(childproc->p_lk);

	/* set return value for parent */
	*retval = pid_childproc;

	/* create a thread for the child proc */
	int r = thread_fork("childproc_thread", childproc, enter_forked_process, tf_kheap, 0);
	return (r);
}

int sys_execv(const_userptr_t program, const_userptr_t argv) {
	(void)argv;
	struct addrspace *as;
	struct vnode *v;
	vaddr_t entrypoint, stackptr;
	int result;

	// copy program path to kernel space
	char* k_program  = (char *) kmalloc(64 * sizeof(char));
	size_t got = 0;
	int err = copyinstr(program, (void*)k_program, (size_t)64, &got);
    if(err) { return err; }

    // count arguments
    char **k_argv_ptr = (char **) kmalloc(64 * sizeof(char*));
    err = copyin(argv, (void*)k_argv_ptr, 64 * sizeof(char*));
    if(err) { return err; }
    int num_args = 0;
    while(k_argv_ptr[num_args] != NULL) {
    	++num_args;
    }

    // copy arguments to kernel space
    char **k_argv = (char **) kmalloc((num_args + 1) * sizeof(char*));
    k_argv[num_args + 1] = NULL;
    for(int i=0; i<num_args; ++i) {
    	k_argv[i] = (char*) kmalloc(64);
    	err = copyinstr((const_userptr_t)k_argv_ptr[i], (void*)k_argv[i], (size_t)64, &got);
    	if(err) { return err; }
    }

	/* Open the file. */
	result = vfs_open(k_program, O_RDONLY, 0, &v);
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
	struct addrspace *old_as = curproc_setas(as);
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

	/* Define the user stack in the address space */
	if(k_argv == NULL) {
		result = as_define_stack(as, &stackptr);
	} else {
		result = as_define_stack_args(as, &stackptr, k_argv, num_args);
	}
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		return result;
	}

	// safe to free memory now
	kfree(k_program);
	for(int i=0; i<num_args; ++i) {
    	kfree(k_argv[i]);
    }
    kfree(k_argv_ptr);
    kfree(old_as);

	/* Warp to user mode. */
	if(num_args == 0) {
		enter_new_process(num_args /*argc*/, NULL /*userspace addr of argv*/,
			  	stackptr, entrypoint);
	} else {
		enter_new_process(num_args, (userptr_t)stackptr, stackptr, entrypoint);
	}
	
	/* enter_new_process does not return. */
	panic("enter_new_process returned\n");
	return EINVAL;	
}


#endif // OPT_A2



