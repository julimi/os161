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
// create a Lock in the intersection
static struct lock *intersectionLock;

// create 12 CVs for each possibilities
static struct cv *nw;
static struct cv *ns;
static struct cv *ne;
static struct cv *ws;
static struct cv *we;
static struct cv *wn;
static struct cv *se;
static struct cv *sn;
static struct cv *sw;
static struct cv *en;
static struct cv *ew;
static struct cv *es;

// for 12 possibilities of origin and destination,
// except for the same origin and destination
volatile int wcount[12];

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
  	intersectionLock = lock_create("intersectionLock");
	nw = cv_create("nw");
	ns = cv_create("ns");
	ne = cv_create("ne");	
	ws = cv_create("ws");
	we = cv_create("we");
	wn = cv_create("wn");
	se = cv_create("se");
	sn = cv_create("sn");
	sw = cv_create("sw");
	en = cv_create("en");
	ew = cv_create("ew");
	es = cv_create("es");
	for (int i = 0; i < 12; i++) {
		wcount[i] = 0;
	}	

	if (intersectionLock == NULL) {
    		panic("could not create Lock");
  	}
	if (nw == NULL || ns == NULL || ne == NULL ||
	    ws == NULL || we == NULL || wn == NULL ||
	    se == NULL || sn == NULL || sw == NULL ||
	    en == NULL || ew == NULL || es == NULL) {
		panic("could not creat CVs");
	}
  	return;
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
	KASSERT(intersectionLock != NULL);
	KASSERT(nw != NULL);
	KASSERT(ns != NULL);
	KASSERT(ne != NULL);
	KASSERT(ws != NULL);
	KASSERT(we != NULL);
	KASSERT(wn != NULL);
	KASSERT(se != NULL);
	KASSERT(sn != NULL);
	KASSERT(sw != NULL);
	KASSERT(en != NULL);
	KASSERT(ew != NULL);
	KASSERT(es != NULL);

	lock_destroy(intersectionLock);
	cv_destroy(nw);
	cv_destroy(ns);
	cv_destroy(ne);
	cv_destroy(ws);
	cv_destroy(we);
	cv_destroy(wn);
	cv_destroy(se);
	cv_destroy(sn);
	cv_destroy(sw);
	cv_destroy(en);
	cv_destroy(ew);
	cv_destroy(es);
}

/* For intersection_before_entry and intersection_after_exit,
 * seperate the intersection into 4 blocks NW, SW, SE, NE
 * if the block is taken up by one thread, increment counts of other threads which gonna use these blocks as well
 * after the thread exit, decrement the counts 
 * specail situation: left turn: the other right turn takes its second block can enter the intersection as well
 */

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
	KASSERT(intersectionLock != NULL);
	KASSERT(origin != destination);	

	lock_acquire(intersectionLock);

	// 0 - 2
	if (origin == north) {
		if (destination == west) {
			while (wcount[0] != 0) {
				cv_wait(nw, intersectionLock);
			}
			wcount[8]++;
			wcount[10]++;
			//wcount[11]++;
		} else if (destination == south) {
			while (wcount[1] != 0) {
				cv_wait(ns, intersectionLock);
			}
			wcount[3]++;
			wcount[4]++;
			wcount[5]++;
			wcount[8]++;
			wcount[10]++;
			wcount[11]++;
		} else {
			while (wcount[2] != 0) {
				cv_wait(ne, intersectionLock);
			}
			//wcount[3]++;
			wcount[4]++;
			wcount[5]++;
			wcount[6]++;
			wcount[7]++;
			wcount[8]++;
			wcount[10]++;
			wcount[11]++;
		}
	} 
	// 3 - 5
	else if (origin == west) {
		if (destination == south) {
			while (wcount[3] != 0) {
				cv_wait(ws, intersectionLock);
			}
			wcount[1]++;
			//wcount[2]++;
			wcount[11]++;
		} else if (destination == east) {
			while (wcount[4] != 0) {
				cv_wait(we, intersectionLock);
			}
			wcount[1]++;
			wcount[2]++;
			wcount[6]++;
			wcount[7]++;
			wcount[8]++;
			wcount[11]++;
		} else {
			while (wcount[5] != 0) {
				cv_wait(wn, intersectionLock);
			}
			wcount[1]++;
			wcount[2]++;
			//wcount[6]++;
			wcount[7]++;
			wcount[8]++;
			wcount[9]++;
			wcount[10]++;
			wcount[11]++;
		}
	} 
	// 6 - 8
	else if (origin == south) {
		if (destination == east) {
			while (wcount[6] != 0) {
				cv_wait(se, intersectionLock);
			}
			wcount[2]++;
			wcount[4]++;
			//wcount[5]++;
		} else if (destination == north) {
			while (wcount[7] != 0) {
				cv_wait(sn, intersectionLock);
			}
			wcount[2]++;
			wcount[4]++;
			wcount[5]++;
			wcount[9]++;
			wcount[10]++;
			wcount[11]++;
		} else {
			while (wcount[8] != 0) {
				cv_wait(sw, intersectionLock);
			}
			wcount[0]++;
			wcount[1]++;
			wcount[2]++;
			wcount[4]++;
			wcount[5]++;
			//wcount[9]++;
			wcount[10]++;
			wcount[11]++;
		}
	}
	// 9 - 11
	else {
		if (destination == north) {
			while (wcount[9] != 0) {
				cv_wait(en, intersectionLock);
			}
			wcount[5]++;
			wcount[7]++;
			//wcount[8]++;
		} else if (destination == west) {
			while (wcount[10] != 0) {
				cv_wait(ew, intersectionLock);
			}
			wcount[0]++;
			wcount[1]++;
			wcount[2]++;
			wcount[5]++;
			wcount[7]++;
			wcount[8]++;
		} else {
			while (wcount[11] != 0) {
				cv_wait(es, intersectionLock);
			}
			//wcount[0]++;
			wcount[1]++;
			wcount[2]++;
			wcount[3]++;
			wcount[4]++;
			wcount[5]++;
			wcount[7]++;
			wcount[8]++;
		}
	}

	lock_release(intersectionLock);
}


// Use broadcast will be faster than signal
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
	KASSERT(intersectionLock != NULL);
	KASSERT(origin != destination);

	lock_acquire(intersectionLock);

	if (origin == north) {
		if (destination == west) {
			wcount[8]--;
			wcount[10]--;
			//wcount[11]--;
			if (wcount[8] == 0) {
				cv_broadcast(sw, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_broadcast(ew, intersectionLock);
			}
			//if (wcount[11] == 0) {
			//	cv_broadcast(es, intersectionLock);
			//}
		} else if (destination == south) {
			wcount[3]--;
			wcount[4]--;
			wcount[5]--;
			wcount[8]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[3] == 0) {
				cv_broadcast(ws, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_broadcast(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_broadcast(wn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_broadcast(sw, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_broadcast(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_broadcast(es, intersectionLock);
			}
		} else {
			//wcount[3]--;
			wcount[4]--;
			wcount[5]--;
			wcount[6]--;
			wcount[7]--;
			wcount[8]--;
			wcount[10]--;
			wcount[11]--;
			//if (wcount[3] == 0) {
			//	cv_broadcast(ws, intersectionLock);
			//}
			if (wcount[4] == 0) {
				cv_broadcast(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_broadcast(wn, intersectionLock);
			}
			if (wcount[6] == 0) {
				cv_broadcast(se, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_broadcast(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_broadcast(sw, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_broadcast(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_broadcast(es, intersectionLock);
			}
		}
	} else if (origin == west) {
		if (destination == south) {
			wcount[1]--;
			//wcount[2]--;
			wcount[11]--;
			if (wcount[1] == 0) {
				cv_broadcast(ns, intersectionLock);
			}
			//if (wcount[2] == 0) {
			//	cv_broadcast(ne, intersectionLock);
			//}
			if (wcount[11] == 0) {
				cv_broadcast(es, intersectionLock);
			}
		} else if (destination == east) {
			wcount[1]--;
			wcount[2]--;
			wcount[6]--;
			wcount[7]--;
			wcount[8]--;
			wcount[11]--;			
			if (wcount[1] == 0) {
				cv_broadcast(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_broadcast(ne, intersectionLock);
			}
			if (wcount[6] == 0) {
				cv_broadcast(se, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_broadcast(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_broadcast(sw, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_broadcast(es, intersectionLock);
			}
		} else {
			wcount[1]--;
			wcount[2]--;
			//wcount[6]--;
			wcount[7]--;
			wcount[8]--;
			wcount[9]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[1] == 0) {
				cv_broadcast(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_broadcast(ne, intersectionLock);
			}
			//if (wcount[6] == 0) {
			//	cv_broadcast(se, intersectionLock);
			//}
			if (wcount[7] == 0) {
				cv_broadcast(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_broadcast(sw, intersectionLock);
			}
			if (wcount[9] == 0) {
				cv_broadcast(en, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_broadcast(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_broadcast(es, intersectionLock);
			}
			
		}
	} else if (origin == south) {
		if (destination == east) {
			wcount[2]--;
			wcount[4]--;
			//wcount[5]--;
			if (wcount[2] == 0) {
				cv_broadcast(ne, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_broadcast(we, intersectionLock);
			}
			//if (wcount[5] == 0) {
			//	cv_broadcast(wn, intersectionLock);
			//}
		} else if (destination == north) {
			wcount[2]--;
			wcount[4]--;
			wcount[5]--;
			wcount[9]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[2] == 0) {
				cv_broadcast(ne, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_broadcast(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_broadcast(wn, intersectionLock);
			}
			if (wcount[9] == 0) {
				cv_broadcast(en, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_broadcast(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_broadcast(es, intersectionLock);
			}
		} else {
			wcount[0]--;
			wcount[1]--;
			wcount[2]--;
			wcount[4]--;
			wcount[5]--;
			//wcount[9]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[0] == 0) {
				cv_broadcast(nw, intersectionLock);
			}
			if (wcount[1] == 0) {
				cv_broadcast(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_broadcast(ne, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_broadcast(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_broadcast(wn, intersectionLock);
			}
			//if (wcount[9] == 0) {
			//	cv_broadcast(en, intersectionLock);
			//}
			if (wcount[10] == 0) {
				cv_broadcast(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_broadcast(es, intersectionLock);
			}
		}
	} else {
		if (destination == north) {
			wcount[5]--;
			wcount[7]--;
			//wcount[8]--;
			if (wcount[5] == 0) {
				cv_broadcast(wn, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_broadcast(sn, intersectionLock);
			}
			//if (wcount[8] == 0) {
			//	cv_broadcast(sw, intersectionLock);
			//}
		} else if (destination == west) {
			wcount[0]--;
			wcount[1]--;
			wcount[2]--;
			wcount[5]--;
			wcount[7]--;
			wcount[8]--;
			if (wcount[0] == 0) {
				cv_broadcast(nw, intersectionLock);
			}
			if (wcount[1] == 0) {
				cv_broadcast(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_broadcast(ne, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_broadcast(wn, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_broadcast(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_broadcast(sw, intersectionLock);
			}
			
		} else {
			//wcount[0]--;
			wcount[1]--;
			wcount[2]--;
			wcount[3]--;
			wcount[4]--;
			wcount[5]--;
			wcount[7]--;
			wcount[8]--;
			//if (wcount[0] == 0) {
			//	cv_broadcast(nw, intersectionLock);
			//}
			if (wcount[1] == 0) {
				cv_broadcast(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_broadcast(ne, intersectionLock);
			}
			if (wcount[3] == 0) {
				cv_broadcast(ws, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_broadcast(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_broadcast(wn, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_broadcast(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_broadcast(sw, intersectionLock);
			}
		}
	}

	lock_release(intersectionLock);
}

