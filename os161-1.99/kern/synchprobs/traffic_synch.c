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
static struct lock *intersectionLock;

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
	
	lock_acquire(intersectionLock);

	int count;
	// 0 - 2
	if (origin == north) {
		if (destination == west) {
			count = wcount[0];
			while (count != 0) {
				cv_wait(nw, intersectionLock);
				count = wcount[0];
			}
			wcount[8]++;
			wcount[10]++;
		} else if (destination == south) {
			count = wcount[1];
			while (count != 0) {
				cv_wait(ns, intersectionLock);
				count = wcount[1];
			}
			wcount[3]++;
			wcount[4]++;
			wcount[5]++;
			wcount[8]++;
			wcount[10]++;
			wcount[11]++;
		} else {
			count = wcount[2];
			while (count != 0) {
				cv_wait(ne, intersectionLock);
				count = wcount[2];
			}
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
			count = wcount[3];
			while (count != 0) {
				cv_wait(ws, intersectionLock);
				count = wcount[3];
			}
			wcount[1]++;
			wcount[11]++;
		} else if (destination == east) {
			count = wcount[4];
			while (count != 0) {
				cv_wait(we, intersectionLock);
				count = wcount[4];
			}
			wcount[1]++;
			wcount[2]++;
			wcount[6]++;
			wcount[7]++;
			wcount[8]++;
			wcount[11]++;
		} else {
			count = wcount[5];
			while (count != 0) {
				cv_wait(wn, intersectionLock);
				count = wcount[5];
			}
			wcount[1]++;
			wcount[2]++;
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
			count = wcount[6];
			while (count != 0) {
				cv_wait(se, intersectionLock);
				count = wcount[6];
			}
			wcount[2]++;
			wcount[4]++;
		} else if (destination == north) {
			count = wcount[7];
			while (count != 0) {
				cv_wait(sn, intersectionLock);
				count = wcount[7];
			}
			wcount[2]++;
			wcount[4]++;
			wcount[5]++;
			wcount[9]++;
			wcount[10]++;
			wcount[11]++;
		} else {
			count = wcount[8];
			while (count != 0) {
				cv_wait(sw, intersectionLock);
				count = wcount[8];
			}
			wcount[0]++;
			wcount[1]++;
			wcount[2]++;
			wcount[4]++;
			wcount[5]++;
			wcount[10]++;
			wcount[11]++;
		}
	}
	// 9 - 11
	else {
		if (destination == north) {
			count = wcount[9];
			while (count != 0) {
				cv_wait(en, intersectionLock);
				count = wcount[9];
			}
			wcount[5]++;
			wcount[7]++;
		} else if (destination == west) {
			count = wcount[10];
			while (count != 0) {
				cv_wait(ew, intersectionLock);
				count = wcount[10];
			}
			wcount[0]++;
			wcount[1]++;
			wcount[2]++;
			wcount[5]++;
			wcount[7]++;
			wcount[8]++;
		} else {
			count = wcount[11];
			while (count != 0) {
				cv_wait(es, intersectionLock);
				count = wcount[11];
			}
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
	
	lock_acquire(intersectionLock);

	if (origin == north) {
		if (destination == west) {
			wcount[8]--;
			wcount[10]--;
			if (wcount[8] == 0) {
				cv_signal(sw, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_signal(ew, intersectionLock);
			}
		} else if (destination == south) {
			wcount[3]--;
			wcount[4]--;
			wcount[5]--;
			wcount[8]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[3] == 0) {
				cv_signal(ws, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_signal(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_signal(wn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_signal(sw, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_signal(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_signal(es, intersectionLock);
			}
		} else {
			wcount[4]--;
			wcount[5]--;
			wcount[6]--;
			wcount[7]--;
			wcount[8]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[4] == 0) {
				cv_signal(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_signal(wn, intersectionLock);
			}
			if (wcount[6] == 0) {
				cv_signal(se, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_signal(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_signal(sw, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_signal(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_signal(es, intersectionLock);
			}
		}
	} else if (origin == west) {
		if (destination == south) {
			wcount[1]--;
			wcount[11]--;
			if (wcount[1] == 0) {
				cv_signal(ns, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_signal(es, intersectionLock);
			}
		} else if (destination == east) {
			wcount[1]--;
			wcount[2]--;
			wcount[6]--;
			wcount[7]--;
			wcount[8]--;
			wcount[11]--;			
			if (wcount[1] == 0) {
				cv_signal(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_signal(ne, intersectionLock);
			}
			if (wcount[6] == 0) {
				cv_signal(se, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_signal(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_signal(sw, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_signal(es, intersectionLock);
			}
		} else {
			wcount[1]--;
			wcount[2]--;
			wcount[7]--;
			wcount[8]--;
			wcount[9]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[1] == 0) {
				cv_signal(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_signal(ne, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_signal(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_signal(sw, intersectionLock);
			}
			if (wcount[9] == 0) {
				cv_signal(en, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_signal(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_signal(es, intersectionLock);
			}
			
		}
	} else if (origin == south) {
		if (destination == east) {
			wcount[2]--;
			wcount[4]--;
			if (wcount[2] == 0) {
				cv_signal(ne, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_signal(we, intersectionLock);
			}
		} else if (destination == north) {
			wcount[2]--;
			wcount[4]--;
			wcount[5]--;
			wcount[9]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[2] == 0) {
				cv_signal(ne, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_signal(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_signal(wn, intersectionLock);
			}
			if (wcount[9] == 0) {
				cv_signal(en, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_signal(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_signal(es, intersectionLock);
			}
		} else {
			wcount[0]--;
			wcount[1]--;
			wcount[2]--;
			wcount[4]--;
			wcount[5]--;
			wcount[10]--;
			wcount[11]--;
			if (wcount[0] == 0) {
				cv_signal(nw, intersectionLock);
			}
			if (wcount[1] == 0) {
				cv_signal(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_signal(ne, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_signal(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_signal(wn, intersectionLock);
			}
			if (wcount[10] == 0) {
				cv_signal(ew, intersectionLock);
			}
			if (wcount[11] == 0) {
				cv_signal(es, intersectionLock);
			}
		}
	} else {
		if (destination == north) {
			wcount[5]--;
			wcount[7]--;
			if (wcount[5] == 0) {
				cv_signal(wn, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_signal(sn, intersectionLock);
			}
		} else if (destination == west) {
			wcount[0]--;
			wcount[1]--;
			wcount[2]--;
			wcount[5]--;
			wcount[7]--;
			wcount[8]--;
			if (wcount[0] == 0) {
				cv_signal(nw, intersectionLock);
			}
			if (wcount[1] == 0) {
				cv_signal(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_signal(ne, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_signal(wn, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_signal(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_signal(sw, intersectionLock);
			}
			
		} else {
			wcount[1]--;
			wcount[2]--;
			wcount[3]--;
			wcount[4]--;
			wcount[5]--;
			wcount[7]--;
			wcount[8]--;
			if (wcount[1] == 0) {
				cv_signal(ns, intersectionLock);
			}
			if (wcount[2] == 0) {
				cv_signal(ne, intersectionLock);
			}
			if (wcount[3] == 0) {
				cv_signal(ws, intersectionLock);
			}
			if (wcount[4] == 0) {
				cv_signal(we, intersectionLock);
			}
			if (wcount[5] == 0) {
				cv_signal(wn, intersectionLock);
			}
			if (wcount[7] == 0) {
				cv_signal(sn, intersectionLock);
			}
			if (wcount[8] == 0) {
				cv_signal(sw, intersectionLock);
			}
		}
	}

	lock_release(intersectionLock);
}
