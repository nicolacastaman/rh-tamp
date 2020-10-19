(define (problem three_objs_test)
(:domain sort_clutter)
(:objects movable - item
          red_0 red_1 - movable
          clutter_table - clutter
          target_red_0 target_red_1 - target
          rur53 - robot
          dock_clutter_table - dock_clutter
          dock_sorted_color - dock_target
)
(:init (using red_0) 
       (using red_1)
       (handempty)
       (at rur53 dock_clutter_table)
       (on red_1 clutter_table)
       (on red_0 clutter_table)
       (=(weight red_0) 1)
       (=(weight red_1) 1)   
       (=(loadTime red_0) 1)
       (=(unloadTime red_0) 1)
       (=(liftTime red_0) 1)
       (=(dropTime red_0) 1)     
       (=(loadTime red_1) 1)
       (=(unloadTime red_1) 1)
       (=(liftTime red_1) 1)
       (=(dropTime red_1) 1) 
       (=(load rur53) 0)
       (=(maxLoad) 1)
       (=(distance dock_clutter_table dock_sorted_color) 5)
       (=(distance dock_sorted_color dock_clutter_table) 5)
       (=(speed rur53) 2.5)
)
(:goal (and (on red_0 target_red_0) (in red_1 rur53) (at rur53 dock_sorted_color))
)
)
(:metric minimize(total-time))
)
