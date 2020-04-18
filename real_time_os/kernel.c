/*
 * kernel.c
 *
 *  Created on: 21-April-2017
 *      Author: Group_23
 */
#include<msp430f5529.h>
#include<kernel/kernel.h>
#include<stdio.h>
#include"kernel_intern.h"
#include<libs/printfx.h>


uint16_t os_irq_sysc;
uint16_t os_SP;

uint16_t const No_resources;
Resource_t  os_resources[];

//void  syscall_mystery(void);

//idle task declaration
__OS_DECLARE_TASK(TaskIdle, 30, 0, ".os_idletask");
OS_TASKENTRY(TaskIdle) {
    while (1) {
    }

}
/*
void insert_into_timeout_queue(TaskId_t task_id)
{
    TaskId_t curr_taskid, next_taskid, temp, *list;
    temp = os_timeoutQueue;
    list = &os_timeoutQueue;
    // sort in ascending order by timeouts and insert
    if (*list == NIL_ID) // the task becomes the head.
    {
        os_timeoutQueueDirty = 1;
        *list = task_id;
        os_tasks[task_id].id = task_id;
        os_tasks[task_id].next_timeout = NIL_ID;
        os_timeoutQueue = task_id;
        return;
    }
    curr_taskid = *list;
    next_taskid = os_tasks[curr_taskid].next_timeout;
    if ((os_tasks[curr_taskid].timeout  > os_tasks[task_id].timeout) \
        && (curr_taskid != task_id))
    {
        *list = task_id;// move it to head
        os_timeoutQueue = task_id;
        if (temp != os_timeoutQueue)
        {
            os_timeoutQueueDirty = 1;
        }
        return;
    }
    while (next_taskid != NIL_ID)
    {
        //prevtask_id = curr_taskid;
        if (curr_taskid == task_id)
        {
            // task id already exists
            PANIC();
            if (temp != os_timeoutQueue)
            {
                os_timeoutQueueDirty = 1;
            }
            return;
        }
        else if ((os_tasks[curr_taskid].timeout < os_tasks[task_id].timeout) \
            && (os_tasks[next_taskid].timeout > os_tasks[task_id].timeout) && (next_taskid != NIL_ID))
        {
            os_tasks[curr_taskid].next_timeout = task_id;
            return;
        }
        else if ((os_tasks[curr_taskid].timeout == os_tasks[task_id].timeout)  \
            && (os_tasks[next_taskid].timeout != os_tasks[task_id].timeout) && (next_taskid != NIL_ID))
        {
            os_tasks[curr_taskid].next_timeout = task_id;
            return;
        }
        curr_taskid = next_taskid;
        next_taskid = os_tasks[curr_taskid].next_timeout;
    }
    os_tasks[curr_taskid].next_timeout = task_id;
    // after sort if there is change in head
    if (temp != os_timeoutQueue)
    {
        os_timeoutQueueDirty = 1;
    }
}
*/
void remove_from_timeout_queue(TaskId_t task_id)
{
    TaskId_t current, prev, next_taskid;
    current = os_timeoutQueue;
    if ((current == NIL_ID) || (task_id == NIL_ID))
    {
        return;
    }
    prev = current;
    next_taskid = os_tasks[current].next_timeout;
    if ((os_tasks[current].id == task_id))
    {
        os_timeoutQueue = next_taskid; // point head to next
        os_tasks[current].next_timeout = NIL_ID;
        os_timeoutQueueDirty = 1;
        next_taskid = NIL_ID;
        return;
    }
    while (next_taskid != NIL_ID)
    {
        prev = current;
        if ((os_tasks[next_taskid].id == task_id))
        {
            os_tasks[prev].next_timeout = os_tasks[next_taskid].next_timeout;
            os_tasks[next_taskid].next_timeout = NIL_ID;
            return;
        }
        current = next_taskid;
        next_taskid = os_tasks[current].next_timeout;
    }
}

Result_t os_initTasks(void)
{

    os_timeoutQueue = NIL_ID;
    os_timeoutQueueDirty = 0;
    os_readyQueue = NIL_ID;
    for(int i=0; i <os_taskCount; i++ )
    {
        *(os_tasks[i].stackHA) = (int)os_tasks[i].entry;
        *(os_tasks[i].stackHA - 1) = GIE;
        os_tasks[i].id = i;
        os_tasks[i].next_timeout = NIL_ID;
        //fill stack with magic word
        for(int *ptr = (int*)os_tasks[i].stackLA; ptr < (int*)os_tasks[i].stackHA-2; ptr++)
        {
            *ptr = OS_MAGIC_STACK_WORD;
        }
        os_tasks[i].reg_context[0] = (int) (os_tasks[i].stackHA - 1);

        insert_into_memberlist(i, &os_readyQueue);

        for(int Reg=1; Reg<13; Reg++)
        {
            os_tasks[i].reg_context[Reg] = 0;
        }
        os_runningTask = &os_tasks[os_readyQueue];
    }

    return 1;
}

void panic(void)
{
    while (1) {
            printfx("%d\n", os_runningTask->id);
        }
}

void os_schedule(void)
{
    Time_t time;
    getCurrentTime(&time);
    TaskId_t id = os_timeoutQueue;
    if (id != NIL_ID )
    {
        while(os_tasks[id].timeout -25 <= time)
        {
            // FIXME member list is not a pointer its an id so use NIL_ID=255 and not NULL=0
            if(os_tasks[id].memberlist != NIL_ID)
            {
                remove_from_memberlist(id);
                //os_tasks[id].reg_context[9] = 0;
            }

            // FIXME:
            //TaskId_t next = os_tasks[id].next_timeout;

            remove_from_timeout_queue(id);
            insert_into_memberlist(id, &os_readyQueue);
            if( os_timeoutQueue == NIL_ID )
                break;
            else
                id = os_timeoutQueue;
        }
    }
    if(os_timeoutQueue == NIL_ID )
    {
        TA0CCR1 = 0xF000;
    }
    else
    {
        os_runningTask = &os_tasks[os_readyQueue];
        if((os_tasks[os_timeoutQueue].timeout - time) < 0xF000)
        {
            // FIXME
            TA0CCR1 = os_tasks[os_timeoutQueue].timeout - time;
        }
        else
        {
            TA0CCR1 = 0xF000;
        }
    }
    os_timeoutQueueDirty = 0;
    os_dispatcher();
}

void __syscall_mystery(void)
{
    remove_from_memberlist(os_runningTask->id);
    insert_into_memberlist(os_runningTask->id, &os_readyQueue);
    os_runningTask = &os_tasks[os_readyQueue];
}

Result_t __syscall_CAS(int * p, int oldVal, int newVal)
{
    if (*p == oldVal)
    {
        *p = newVal;
        // FIXME we will save the return values in core.s
        // os_runningTask->reg_context[9] = 1;
        return 1;
    }
    // FIXME we will save the return values in core.s
    //os_runningTask->reg_context[9] = 0;
    return 0;
}


void updateHead() {
    os_runningTask = &os_tasks[os_readyQueue];
}

Result_t __syscall_waitEventUntil(Event_t *event, const Time_t *deadline){
    Time_t time1,time2;
    TaskId_t id = os_runningTask->id;
    if(os_runningTask->nResources !=0)
    {
        return 0;
    }
    if(event != NULL)
    {
        if(event->value == 1)
        {
            event->value = 0;
            return 1;
        }
    }
    if (id != NIL_ID )
    {
        if(deadline != __DEADLINE_INFINITE)
        {
            getCurrentTime(&time1);
            if(deadline == __DEADLINE_PRESET)
                time2 = os_tasks[id].timeout;
            else
                time2 = *deadline;

            if(time2 <= time1)
            {
                return 0;
            }
        }
        remove_from_memberlist(id);
        if(deadline != __DEADLINE_PRESET)
        {
            os_tasks[id].timeout = *deadline;
        }
        if(event != NULL)
        {
            insert_into_memberlist(id, &event->waitqueue);
        }

        if(deadline != __DEADLINE_INFINITE)
        {
            insert_into_timeout_queue(id);
        }
    }
    return 0;
}

void __syscall_setEvent(Event_t *event)
{
    if(event->waitqueue == NIL_ID)
    {
        event->value = 1;
    }
    else
    {
       TaskId_t id = event->waitqueue;
       remove_from_memberlist(id);
       remove_from_timeout_queue(id);
       insert_into_memberlist(id, &os_readyQueue);
       os_tasks[id].reg_context[9] = 1;
    }
}

Result_t waitEventFor(Event_t *event, Delay_t timeout){
    Result_t result;
    Time_t time1;
    getCurrentTime(&time1);
    TaskId_t id = os_runningTask->id;
    //const Time_t  deadline = __DEADLINE_PRESET;
    os_tasks[id].timeout = time1 + timeout;
    result = syscall_waitEventUntil(event, __DEADLINE_PRESET);
    return result;
}

Result_t waitEventUntil(Event_t *event, const Time_t *deadline)
{
    Result_t result;
    result = syscall_waitEventUntil(event, deadline);
    return result;
}


Result_t waitEvent(Event_t *event)
{
    Result_t result = 0;
    if(event != NULL)
    {
        //const Time_t deadline = __DEADLINE_INFINITE;
        result = syscall_waitEventUntil(event, __DEADLINE_INFINITE);
    }
    return result;
}

void setEvent(Event_t *event)
{
    if(event != NULL)
    {
        __syscall_setEvent(event);
    }
}

void clearEvent(Event_t *event)
{
    if(event != NULL)
    {
        event->value = 0;
    }
}

Result_t checkEvent(Event_t* event)
{
    if(event != NULL)
    {
        if(event->value == 1)
            return 1;
    }
    return 0;
}

void sleep(Delay_t timeout)
{
    waitEventFor(NULL, timeout);
}

void sleepUntil(Time_t *deadline)
{
    if(deadline != NULL)
    {
        syscall_waitEventUntil(NULL, deadline);
    }
}

void getResource(Resource_t *resource)
{
    syscall_getResource(resource);
}

void releaseResource(Resource_t *resource)
{
    syscall_releaseResource(resource);
}

void __syscall_getResource(Resource_t *resource)
{
    TaskId_t id = os_runningTask->id;
    if(resource->counter == 0)
    {
        if( resource->prioCeiling > os_tasks[id].prioActive )
        {
            os_tasks[id].prioActive = resource->prioCeiling;
            TaskId_t *memberlist = os_tasks[id].memberlist;
            os_tasks[id].prioActive ++;
            remove_from_memberlist(id);
            insert_into_memberlist(id, memberlist);
            os_tasks[id].prioActive --;
        }
        resource->owner = id;
    }
    os_tasks[resource->owner].nResources ++;
    resource->counter ++;
}

void __syscall_releaseResource(Resource_t *resource)
{
    uint8_t priority = 0;
    TaskId_t id = os_runningTask->id;
    if(resource->owner == id)
    {
        os_tasks[id].nResources --;
        resource->counter --;
        if(resource->counter == 0)
        {
            resource->owner = NIL_ID;
        }
    }
    if(os_tasks[id].nResources > 0)
    {
        for(int i = 0;  i< No_resources; i++)
        {
            if(os_resources[i].owner == id)
            {
                if(os_resources[i].prioCeiling > priority)
                    priority = os_resources[i].prioCeiling;
            }
        }
        if(priority > os_tasks[id].prioBase)
        {
            os_tasks[id].prioActive = priority;//Priority changed
            os_tasks[id].prioActive ++;
            remove_from_memberlist(id);
            insert_into_memberlist(id, os_tasks[id].memberlist);
            os_tasks[id].prioActive --;
        }
    }
}
