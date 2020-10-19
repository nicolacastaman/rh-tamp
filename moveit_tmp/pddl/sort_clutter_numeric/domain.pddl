(define (domain sort_clutter)
(:requirements :strips :adl :typing :fluents)
(:types item surface place locatable - object
        movable others - item
        clutter target - surface
        dock_clutter dock_target - place
        robot - locatable
)
(:predicates (handempty)
             (holding ?i - item)
             (obstruct ?i - item ?o - object)
             (using ?i - item)
             (on ?i - item ?s - surface)
             (in ?i - item ?r - robot)
             (at ?r - robot ?p - place)
)
(:functions (distance ?x - place ?y -place)
           (speed ?r - robot)
           (load ?r - robot)
           (maxLoad)
           (weight ?i - item)
           (loadTime ?i - item)
           (unloadTime ?i - item)
           (liftTime ?i - item)
           (dropTime ?i - item)
           (time)
)
(:action navigate
  :parameters (?r - robot ?x - place ?y - place)
  :precondition
  (and
    (handempty)
    (at ?r ?x)
   )
  :effect
  (and
    (not(at ?r ?x))
    (at ?r ?y)
    (increase(time) (/ (distance ?x ?y) (speed ?r)))
  )
)
(:action load-robot
  :parameters (?m - movable ?r - robot ?p - place)
  :precondition
  (and
    (holding ?m)
    (not(handempty))
    (not(exists(?i - item)(obstruct ?i ?r)))
    (at ?r ?p)
    (>= (maxLoad) (+ (load ?r) (weight ?m)))
   )
  :effect
  (and
    (not(holding ?m))
    (handempty)
    (in ?m ?r)
    (increase(load ?r)(weight ?m))
    (increase(time)(loadTime ?m))
  )
)
(:action unload-robot
  :parameters (?m - movable ?r - robot ?p - place)
  :precondition
  (and
    (handempty)
    (in ?m ?r)
    (not (exists (?i - item) (obstruct ?i ?m)))
    (at ?r ?p)
   )
  :effect
  (and
    (holding ?m)
    (not(handempty))
    (not(in ?m ?r))
    (forall (?i - item)(not (obstruct ?m ?i)))
    (decrease(load ?r)(weight ?m))
    (increase(time)(unloadTime ?m))
  )
)
(:action drop_to_clutter
  :parameters (?m - movable ?c - clutter ?r - robot ?d - dock_clutter)
  :precondition
  (and
    (holding ?m)
    (not(handempty))
    (not(exists(?i - item)(obstruct ?i ?c)))
    (at ?r ?d)
   )
  :effect
  (and
    (not(holding ?m))
    (handempty)
    (on ?m ?c)
    (increase(time)(dropTime ?m))
  )
)
(:action lift_from_clutter
  :parameters (?m - movable ?c - clutter ?r - robot ?d - dock_clutter)
  :precondition
  (and
    (handempty)
    (on ?m ?c)
    (not(exists(?i - item)(obstruct ?i ?m)))
    (at ?r ?d)
   )
  :effect
  (and
    (holding ?m)
    (not(handempty))
    (not(on ?m ?c))
    (forall(?i - item) (not(obstruct ?m ?i)))
    (increase(time)(liftTime ?m))
  )
)
(:action drop_to_target
  :parameters (?m - movable ?t - target ?r - robot ?d - dock_target)
  :precondition
  (and
    (holding ?m)
    (not(handempty))
    (not(exists(?i - item)(obstruct ?i ?t)))
    (at ?r ?d)
   )
  :effect
  (and
    (not(holding ?m))
    (handempty)
    (on ?m ?t)
    (increase(time)(dropTime ?m))
  )
)
(:action lift_from_target
  :parameters (?m - movable ?t - target ?r - robot ?d - dock_target)
  :precondition
  (and
    (handempty)
    (on ?m ?t)
    (not(exists(?i - item)(obstruct ?i ?m)))
    (at ?r ?d)
   )
  :effect
  (and
    (holding ?m)
    (not(handempty))
    (not(on ?m ?t))
    (forall(?i - item)(not(obstruct ?m ?i)))
    (increase(time)(liftTime ?m))
  )
)
)
