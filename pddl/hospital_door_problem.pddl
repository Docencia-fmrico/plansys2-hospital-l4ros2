(define (problem indoor_nav_problem0)
;; indoor_nav_domain.pddl
(:domain indoor-nav-strips-typed)


(:objects 
  ;;ROBOTS:
  brobot - robot
  
  ;;ROOMS:
  str1 str2 - room ;external rooms.
  
  ;;DOORS:
  door1 - door
  ;;v16 v11 s1 s1i s2 s2i s32 s36 s4 s4i s5 s5i - door ;external room doors.
  ;;x1 x2 c1 c2 a1 a2 - door ;central room doors.
  ;;e1 e2 e3 e4 g1 g2 t1 t2 - door ;hall doors.
  
  ;;CORRIDORS:
  ;;corridor1 corridor2 corridor3 corridor4 corridor5 corridor6 - corridor
  ;;reception hall - corridor

  ;;ZONES:
  ;zoneX zoneY zoneZ - zone
  
  ;;OBJECTS:
  ;;object1 - object
)
(:init
  ;;ROBOT POSITION:
  (robot_at brobot str1)
  (closed door1)
  (connected_to_door str1 door1)
  (connected_to_door str2 door1)

  ;;OBJECT POSITION:
  ;;(objectAt object1 inner1)
)

(:goal  ;; to be achieved
  
  ;; our robot BROBOT is a polite robot that respects intimacy of humans 
  ;; and it will leave ALL the doors closed

    (robot_at brobot str2)

 
  

)
)
