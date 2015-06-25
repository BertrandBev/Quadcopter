/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <timer.h>
#include <platsupport/timer.h>
#include <platsupport/plat/timer.h>
#include <platsupport/mach/pwm.h>
#include "utils.h"
#include <common.h>
#include <stdio.h>

//#define DEBUG_TIMER
#ifdef DEBUG_TIMER
#define DTIME(args...) \
    do { \
        printf("TIMER %s(%d): ", __func__, __LINE__); \
        printf(args); \
        printf("\n"); \
    } while(0)
#else
#define DTIME(...) do{}while(0)
#endif

#define THRESH_TIME_INT (100000)
// Move this to shared file

pstimer_t *timer;
pstimer_t *gtimer;
pwm_config_t config;

struct event_node {
    int id; // The id of the component that requested interrupt
    uint64_t time; // The global time this should be triggered at
    uint64_t period; // If non zero, the period at which that event should be rescheduled
    struct event_node *next;
};

struct event_node *event_list = NULL;

struct timer_client_details {
    int id;
    void (*ping)(void);
};

/**
  * Client parameters. There should be an entry for every component that wants to use this service.
  */
#define CLIENT_PARAMS(i, p)     \
    {                           \
        .id = i,                \
        .ping = p               \
    }       
static struct timer_client_details client_params[] = {
    //CLIENT_PARAMS(GCC_APP_ID, &gcc_wakeup_emit),
    CLIENT_PARAMS(PLANNER_APP_ID, &planner_wakeup_emit),
};

static struct timer_client_details*
get_client_details(int id)
{
    int i;
    for(i = 0; i < ARRAY_SIZE(client_params); i++){
        if (client_params[i].id == id){
            return &client_params[i];
        }
    }
    return NULL;
}

uint64_t get_current_time() {
    // Return the global time in us
    return timer->get_time(timer);
}

void add_event_to_list(struct event_node *e) {
    // Put the new event node into the correct order in the list
    DTIME("Scheduling for %d at %llu", e->id, e->time);
    e->next = NULL; // Rest of function assumes this is true
    struct event_node *n = event_list;
    struct event_node *prev = NULL;
    while (n != NULL) {
        if (e->time < n->time) {
            e->next = n;
            if (prev == NULL) {
                // Inserting at head
                event_list = e;
            } else {
                prev->next = e;
            }
            break;
        }
        prev = n;
        n = n->next;
    }
    if (n == NULL) {
        // Inseting at end of list
        if (prev == NULL) {
            DTIME("Adding request by %d to head", e->id);
            event_list = e;
        } else {
            prev->next = e;
        }
    }

    // Printing schedule list
#ifdef DEBUG_TIMER
    n = event_list;
    while (n != NULL) {
        printf("%d(%llu)-> ", n->id, n->time);
        n = n->next;
    }
    printf("\n"); 
#endif
}

void timer_register_wakeup(int client_id, uint64_t time_us, uint64_t period_us) {
    timer_driver_lock();
    DTIME("Id %d, time %llu, period %llu", client_id, time_us, period_us);
    struct event_node *e = malloc(sizeof(struct event_node));
    e->id = client_id;
    e->time = get_current_time() + time_us;
    e->period = period_us;
    e->next = NULL;
    struct event_node *prev_head = event_list;
    add_event_to_list(e);   
    
    // Register any required interrupts
    if (prev_head != event_list) {
        DTIME("Added new head");
        timer->oneshot_relative(timer, time_us);
    }
    timer_driver_unlock();
}

int64_t prev_time = 0;
int64_t gpt = 0;
int i = 0;
int pinger = 0;
void time_expired() {
    char buffer[200];
    int64_t cur_time = get_current_time();
    //DTIME("Oneshot expired");
    // The head event has triggered
    // Wake up all events within the threshold
    DTIME("Current time is %lld, since last hit %lld", cur_time, (cur_time - prev_time)/1000);
    if (i % 50 == 0) {
        sprintf(buffer, "C: %lld D: %lld\n", cur_time, cur_time - prev_time);
    }

    prev_time = cur_time;
    while (event_list != NULL) {
        if (i % 50 == 0) {
            sprintf(buffer, "W: %llu, C: %llu, D: %lld\n", event_list->time, get_current_time(), cur_time - event_list->time);
            sprintf(buffer, "GC: %llu, GD: %llu\n", gtimer->get_time(gtimer), gtimer->get_time(gtimer) - gpt);
        }
        gpt = gtimer->get_time(gtimer);
        DTIME("Wakeup time was %llu, cur time is %llu", event_list->time, get_current_time());
        if ((event_list->time) > (get_current_time() + THRESH_TIME_INT)) {
            // Schedule the next event
            //DTIME("Scheduling event for %d at %llu", event_list->id, event_list->period);
            timer->oneshot_relative(timer, event_list->period);
            //DTIME("Return");
            break;
            DTIME("Da fuck?");
        }
        // Wakeup the owner of this event
        struct timer_client_details *c = get_client_details(event_list->id);
        //DTIME("Pinged %d", event_list->id);
        if (i % 50 == 0) {
            sprintf(buffer, "P %d, S %d\n", event_list->id, pinger);
        }
        c->ping();
        ++pinger;
        struct event_node *expired = event_list;
        event_list = event_list->next;
        // If the expired node is periodic, schedule it again
        if (expired->period != 0) {
            expired->time = get_current_time() + expired->period;
            //DTIME("Period %llu, time %llu", expired->period, get_current_time());
            add_event_to_list(expired);
        } else {
            free(expired);
        }
    }
    ++i;
    DTIME("Waiting for int");
}


static void timer_interupt_handler0(void *arg) {
    timer_driver_lock();
    timer->handle_irq(timer, PWM_TIMER0);
    timer_int0_reg_callback(&timer_interupt_handler0, PWM_TIMER0);
    timer_driver_unlock();
}

static void timer_interupt_handler4(void *arg) {
    timer_driver_lock();
    //DTIME("Got interrupt 4");
    timer->handle_irq(timer, PWM_TIMER4);
    time_expired();
    timer_int4_reg_callback(&timer_interupt_handler4, PWM_TIMER4);
    timer_driver_unlock();
}

void timer__init(void) {
    DTIME("Starting init");
    config.vaddr = pwm_clk;
    timer = pwm_get_timer(&config);
    gtimer = generic_timer_get_timer();
    timer_int4_reg_callback(&timer_interupt_handler4, NULL);
    timer_int0_reg_callback(&timer_interupt_handler0, NULL);
    timer->start(timer);
    DTIME("Finished init");
}

int run() {
    //timer__init();
    //while (1) {
    //    printf("Time is %lld\n", timer->get_time(timer));
    //    udelay(10000);
    //}
    return 0;
}
