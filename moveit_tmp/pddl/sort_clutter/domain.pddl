(define (domain sort_clutter)
(:requirements :adl :typing :durative-actions :fluents)
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
           (maxLoad ?r - robot)
           (weight ?i - item)
           (loadTime ?i - item)
           (unloadTime ?i - item)
           (liftTime ?i - item)
           (dropTime ?i - item)
)
(:durative-action navigate
  :parameters (?r - robot ?x - place ?y - place)
  :duration (= ?duration (/ (distance ?x ?y) (speed ?r)))
  :condition
  (and
    (at start (at ?r ?x))
   )
  :effect
  (and
    (at start (not(at ?r ?x)))
    (at end (at ?r ?y))
  )
)
(:durative-action load-robot
  :parameters (?m - movable ?r - robot ?p - place)
  :duration (= ?duration (loadTime ?m))
  :condition
  (and
    (over all (using ?m))
    (at start (holding ?m))
    (at start (not(handempty)))
    (at start (not(exists(?i - item)(obstruct ?i ?r))))
    (at start (at ?r ?p))
    (at start (>=(maxLoad)(+(load ?r)(weight ?m))))
   )
  :effect
  (and
    (at start (not(holding ?m)))
    (at start (handempty))
    (at end (in ?m ?r))
    (at end (increase(load ?r)(weight ?m)))
  )
)
(:durative-action unload-robot
  :parameters (?m - movable ?r - robot ?p - place)
  :duration (= ?duration (unloadTime ?m))
  :condition
  (and
    (over all (using ?m))
    (at start (handempty))
    (at start (in ?m ?r))
    (at start (not (exists (?i - item) (obstruct ?i ?m))))
    (at start (at ?r ?p))
   )
  :effect
  (and
    (at start (holding ?m))
    (at start (not(handempty)))
    (at end (not(in ?m ?r)))
    (forall (?i - item)(at end (not (obstruct ?m ?i))))
    (at end (decrease(load ?r)(weight ?m)))
  )
)
(:durative-action drop_to_clutter
  :parameters (?m - movable ?c - clutter ?r - robot ?d - dock_clutter)
  :duration (= ?duration (dropTime ?m))
  :condition
  (and
    (over all (using ?m))
    (at start (holding ?m))
    (at start (not(handempty)))
    (at start (not(exists(?i - item)(obstruct ?i ?c))))
    (at start (at ?r ?d))
   )
  :effect
  (and
    (at start (not(holding ?m)))
    (at start (handempty))
    (at end (on ?m ?c))
  )
)
(:durative-action lift_from_clutter
  :parameters (?m - movable ?c - clutter ?r - robot ?d - dock_clutter)
  :duration (= ?duration (liftTime ?m))
  :condition
  (and
    (over all (using ?m))
    (at start (handempty))
    (at start (on ?m ?c))
    (at start (not(exists(?i - item)(obstruct ?i ?m))))
    (at start (at ?r ?d))
   )
  :effect
  (and
    (at start (holding ?m))
    (at start (not(handempty)))
    (at end (not(on ?m ?c)))
    (forall(?i - item) (at end (not(obstruct ?m ?i))))
  )
)
(:durative-action drop_to_target
  :parameters (?m - movable ?t - target ?r - robot ?d - dock_target)
  :duration (= ?duration (dropTime ?m))
  :condition
  (and
    (over all (using ?m))
    (at start (holding ?m))
    (at start (not(handempty)))
    (at start (not(exists(?i - item)(obstruct ?i ?t))))
    (at start (at ?r ?d))
   )
  :effect
  (and
    (at start (not(holding ?m)))
    (at start (handempty))
    (at end (on ?m ?t))
  )
)
(:durative-action lift_from_target
  :parameters (?m - movable ?t - target ?r - robot ?d - dock_target)
  :duration (= ?duration (liftTime ?m))
  :condition
  (and
    (over all (using ?m))
    (at start (handempty))
    (at start (on ?m ?t))
    (at start (not(exists(?i - item)(obstruct ?i ?m))))
    (at start (at ?r ?d))
   )
  :effect
  (and
    (at start (holding ?m))
    (at start (not(handempty)))
    (at end (not(on ?m ?t)))
    (forall(?i - item)(at end (not(obstruct ?m ?i))))
  )
)
)
