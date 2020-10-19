(define (problem three_objs_test)
(:domain sort_clutter)
(:objects movable - item
          red_0 red_1 blue_0 - movable
          clutter_table - clutter
          target_red_0 target_red_1 target_blue_0 - target
          rur53 - robot
          dock_clutter_table - dock_clutter
          dock_sorted_color - dock_target
)
(:init (handempty)
       (at rur53 dock_clutter_table)
       (on red_0 clutter_table)
       (on red_1 clutter_table)
       (on blue_0 clutter_table)
       (obstruct blue_0 red_1)
       (obstruct red_0 red_1)
       (=(weight red_0) 1)
       (=(weight red_1) 1)
       (=(weight blue_0) 1)
       (=(loadTime movable) 1)
       (=(unloadTime movable) 1)
       (=(liftTime movable) 1)
       (=(dropTime movable) 1)
       (=(load rur53) 0)
       (=(maxLoad rur53) 2)
       (=(distance dock_clutter_table dock_sorted_color) 5)
       (=(speed rur53) 2.5)
)
(:goal (and (on red_0 target_red_0)
       	    (on red_1 target_red_1)
            (on blue_0 target_blue_0))
)
(:metric minimize(total-time))
)
