(define (domain navigation)
(:requirements :adl :fluents)
(:types item surface place - object
        movable fixed - item
)
(:predicates (arm-empty)
             (holding ?m - movable)
             (is-stackable ?i - item)
             (located-in ?o - object ?p - place)
             (loaded ?m - movable)
             (leave-free ?i - item ?m - movable)
             (near ?p - place)
             (obstruct ?i - item ?o - object)
             (on ?i - item ?o - object)
)
(:functions (distance ?x - place ?y - place)
            (speed)
            (load)
            (max-load)
            (weight ?m - movable)
            (time)
            (manipulation-time)
)

(:action navigate
  :parameters (?x - place ?y - place)
  :precondition
  (and
    (arm-empty)
    (near ?x)
   )
  :effect
  (and
    (not(near ?x))
    (near ?y)
    (increase(time) (distance ?x ?y))
  )
)

(:action load
  :parameters (?m - movable)
  :precondition
  (and
    (holding ?m)
    (>= (max-load) (+ (load) (weight ?m)))
   )
  :effect
  (and
    (not(holding ?m))
    (arm-empty)
    (loaded ?m)
    (increase(load)(weight ?m))
    (increase(time)(manipulation-time))
  )
)

(:action unload
  :parameters (?m - movable)
  :precondition
  (and
    (arm-empty)
    (loaded ?m)
   )
  :effect
  (and
    (holding ?m)
    (not(arm-empty))
    (not(loaded ?m))
    (decrease(load)(weight ?m))
    (increase(time)(manipulation-time))
  )
)

(:action pickup
  :parameters (?m - movable ?s - surface)
  :precondition
  (and
    (arm-empty)
    (on ?m ?s)
    (exists (?p - place) (and (located-in ?s ?p) (near ?p)))
    (not (exists (?i - item) (obstruct ?i ?m)))
    (not (exists (?i - item) (on ?i ?m)))
  )
  :effect
  (and
    (holding ?m)
    (not (arm-empty))
    (forall (?i - item) (not (obstruct ?m ?i)))
    (not (on ?m ?s))
    (forall (?p - place) (not (located-in ?m ?p)))
    (increase(time)(manipulation-time))
  )
)

(:action unstack
 :parameters (?m - movable ?i - item)
 :precondition
  (and
    (arm-empty)
    (on ?m ?i)
    (exists (?p - place) (and (located-in ?i ?p) (near ?p)))
    (not (exists (?o - item) (obstruct ?o ?m)))
    (not (exists (?o - item) (on ?o ?m)))
  )
 :effect
  (and
    (holding ?m)
    (not (arm-empty))
    (forall (?o - item) (not (obstruct ?m ?o)))
    (forall (?s - item) (not (leave-free ?s ?m)))
    (not (on ?m ?i))
    (forall (?p - place) (not (located-in ?m ?p)))
    (increase(time)(manipulation-time))
  )
)

(:action putdown
 :parameters (?m - movable ?s - surface)
 :precondition
  (and
    (holding ?m)
    (not (= ?m ?s))
    (exists (?p - place) (and (located-in ?s ?p) (near ?p)))
    (not (exists (?i - item) (obstruct ?i ?s)))
  )
 :effect
  (and
    (arm-empty)
    (not (holding ?m))
    (on ?m ?s)
    (forall (?p - place) (when (located-in ?s ?p) (located-in ?m ?p)))
    (increase(time)(manipulation-time))
  )
)

(:action stack
 :parameters (?m - movable ?i - item)
 :precondition
  (and
    (holding ?m)
    (not (= ?m ?i))
    (exists (?p - place) (and (located-in ?i ?p) (near ?p)))
    (not (exists (?mx - movable) (on ?mx ?i)))
    (not (exists (?mx - movable) (leave-free ?i ?mx)))
    (not (exists (?ix - item) (obstruct ?ix ?i)))
    (is-stackable ?i)
  )
 :effect
  (and
    (arm-empty)
    (not (holding ?m))
    (on ?m ?i)
    (forall (?p - place) (when (located-in ?i ?p) (located-in ?m ?p)))
    (increase(time)(manipulation-time))
  )
)
)
