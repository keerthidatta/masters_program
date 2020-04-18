
#include <kernel/kernel.h>


#define OS_MAGIC_STACK_WORD		(0xA55A)	///< the magic word for the task stack preparation


/*==== Kernel variables as defined in the C sources ====*/
extern	uint16_t	os_kernel_mode;			///< indicates if we are in kernel mode (1) or in task mode (0)
extern	Task_t*		os_runningTask;			///< pointer to the TCB of the currently running task
extern	TaskId_t	os_readyQueue;			///< ID of the first task in the priority queue holding the ready tasks. The head is the currently running task.
extern	TaskId_t    os_timeoutQueue;		///< ID of the first task in the system-wide timeout queue.
extern	uint8_t		os_timeoutQueueDirty;	///< 1 indicates that the head of the timout queue has changed during the das modification on the queue

/*==== Kernel variables as provided by the linker ====*/
extern	Task_t			os_tasks[];				///< The array of consecutive TCBs. The array has os_taskCount elements
extern	uint8_t 		const os_taskCount;		///< The total number of declared tasks (including the idle task).


/**
 * This function will be executed once at system start, right before the kernel invokes the scheduler for the first time.
 * 	It initializes the system timeline and the corresponding hardware timer:
 *
 * 	- At system start the OS timeline (os_timeline) has to start from 0.
 * 	- The OS timeline will be driven by the hardware TimerA0 which is in turn driven by the SMCLK:
 *		Use the function sysclk_getSMCLK() from sysclk.h to calculate suitable dividers for generating
 *		a timer frequency of 1MHz (resulting in a timeline resolution of 1us).
 *
 * 	Since the OS scheduler needs to be executed at certain points in time
 * 	(which basically depend on the task execution flow and the timer bit width)...
 * 	... the Timer is used in UpMode and CompareMode, using TA0CCR0 as hard overflow value.
 *  ... the Timer triggers an interrupt whenever it reaches the soft overflow value stored in TA0CCR1.
 *
 *	The function also makes sure that the TimerA Interrupt Enable and the general interrupts (use __EINT())
 *	bits are activated after the timer initialization.
 *
 *	The function also configures pin P6.0 as general purpose output pin, for test purposes
 *
 *	@return 1 on success (timer running at 1MHz),
 *			0 on failure (e.g., the target frequency of 1MHz could not be set)
 */
extern Result_t os_initSystemTimer(void);


/** Inserts the specified task into the given memberlist.
 *  The memberlist is a priority queue, and thus sorted descending by task priorities.
 *  The successor of a task t in the same queue is indicated by t->next in t's TCB.
 *  If the new task has the same priority as another task in the queue, the new task will be inserted *after* the other one.
 *  If the task already belongs to the queue, the function simply returns.
 *
 *  @asserts on NULL pointer for the list
 *  @param task_id is the ID of the task to be inserted
 *  @param list is a pointer to the ID of the queue's head-task, if  (*list == NIL_ID) the specified task will become the head.
 */
extern void insert_into_memberlist(TaskId_t task_id, TaskId_t *list);

/** Removes the specified task from its current memberlist.
 *  The memberlist remains sorted descending by task-priorities.
 *  Removing the idle task (task_id = 0) is not allowed and will cause a kernel panic.
 *  If the specified task does not belong to any memberlist, the function simply returns.
 *
 *  @asserts on the attempt to remove the idle task from its memberlist
 *  @param task_id is the ID of the task to be removed
 */
extern void remove_from_memberlist(TaskId_t task_id);

/** Inserts the specified task into the timeout-queue.
 *  The timeout queue is global for the whole system and always sorted ascending by task-timeouts.
 *  If the head of the timeout queue changes, timeout_queue_dirty will be set to 1.
 *
 *  The system shall panic in case the task is already in a timeout queue, since this is not allowed to happen.
 */
extern void insert_into_timeout_queue(TaskId_t task_id);

/** Removes the specified task from the timeout-queue.
 * 	If the specified task is not found within the timeout-queue, the function simply returns.
 *  The timeout queue is global for the whole system and always sorted ascending by task-timeouts.
 *  If the head of the timeout queue changes, timeout_queue_dirty will be set to 1.
 */
extern void remove_from_timeout_queue(TaskId_t task_id);

/** Initializes the tasks:
 *
 * 	- Init all task stacks by
 * 		- preparing them for the dispatcher
 * 		- filling the rest of the stack with the OS_MAGIC_STACK_WORD.
 *
 * 	- Init all task control blocks by
 * 		- assigning a continuous task ID. 0 for the idle task, 1+ for the remaining tasks
 * 		- put the task-specific stack pointer into the reg_context
 *
 * 	- Insert all tasks into the ready queue
 *
 * 	@return SUCCESS on success, FAILURE otherwise
 */
extern Result_t os_initTasks(void);