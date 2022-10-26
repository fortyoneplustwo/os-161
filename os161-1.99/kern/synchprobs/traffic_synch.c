#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
//static struct semaphore *intersectionSem;
static struct lock* trafficlight_lock;
volatile int trafficlight; // 0 = NULL, 1 = N, 2 = S, 3 = E, 4 = W
static struct cv *q[4];
static struct lock* qlock[4];
volatile int qcount[4];



/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */
  //intersectionSem = sem_create("intersectionSem",1);
  //if (intersectionSem == NULL) {
  //  panic("could not create intersection semaphore");
  //}

  trafficlight = -1;

  q[north] = cv_create("cv north");
  if (q[north] == NULL) {
    panic("could not create cv north");
  }
  q[east] = cv_create("cv east");
  if (q[east] == NULL) {
    panic("could not create cv east");
  }
  q[south] = cv_create("cv south");
  if (q[south] == NULL) {
    panic("could not create cv south");
  }
  q[west] = cv_create("cv west");
  if (q[west] == NULL) {
    panic("could not create cv west");
  }

  trafficlight_lock = lock_create("trafficlight lock");
  if (trafficlight_lock == NULL) {
    panic("could not create trafficlight lock");
  }
  qlock[north] = lock_create("qlock n");
  if (qlock[north] == NULL) {
    panic("could not create qlock n");
  }
  qlock[east] = lock_create("qlock e");
  if (qlock[east] == NULL) {
    panic("could not create qlock e");
  }
  qlock[south] = lock_create("qlock s");
  if (qlock[south] == NULL) {
    panic("could not create qlock s");
  }
  qlock[west] = lock_create("qlock w");
  if (qlock[west] == NULL) {
    panic("could not create qlock w");
  }

  qcount[north] = qcount[east] = qcount[south] = qcount[west] = 0;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  //KASSERT(intersectionSem != NULL);
  //sem_destroy(intersectionSem);

  cv_destroy(q[north]);
  cv_destroy(q[east]);
  cv_destroy(q[south]);
  cv_destroy(q[west]);

  lock_destroy(trafficlight_lock);
  lock_destroy(qlock[north]);
  lock_destroy(qlock[east]);
  lock_destroy(qlock[south]);
  lock_destroy(qlock[west]);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  //(void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  //KASSERT(intersectionSem != NULL);
  lock_acquire(qlock[origin]);
  ++qcount[origin];
  lock_release(qlock[origin]);
  lock_acquire(trafficlight_lock);
  if(trafficlight == -1) { trafficlight = origin; }
  int origin_int = origin;
  if(trafficlight != origin_int) { 
  	cv_wait(q[origin], trafficlight_lock); 
  }
  lock_release(trafficlight_lock);


 // P(intersectionSem);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  //(void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  //KASSERT(intersectionSem != NULL);

  lock_acquire(trafficlight_lock);
  lock_acquire(qlock[north]);
  lock_acquire(qlock[east]);
  lock_acquire(qlock[south]);
  lock_acquire(qlock[west]);
  --qcount[origin];
  int newdirection = 0;
  int maxqcount = qcount[0];
  if(qcount[origin] == 0) {
  	for(int i=1; i < 4; ++i) {
  		if (qcount[i] > maxqcount) { 
  			maxqcount = qcount[i]; 
  			newdirection = i;
  		}
  	}
  	if(maxqcount == 0) { // no cars waiting
  		trafficlight = -1;
  	} else {
  		trafficlight = newdirection;
  		cv_broadcast(q[newdirection], trafficlight_lock);
  	}
  }
  
  lock_release(qlock[west]);
  lock_release(qlock[south]);
  lock_release(qlock[east]);
  lock_release(qlock[north]);
  lock_release(trafficlight_lock);
  	 
  //V(intersectionSem);
}



