(define (problem indoor_nav_problem0)
;; indoor_nav_domain.pddl
(:domain indoor-nav-strips-typed)


(:objects 
  ;;ROBOTS:
  brobot - robot
  
  ;;ROOMS:
  visit1 str1 str2 str3 str4 str5 - room ;external rooms.
  inner1 inner2 inner4 inner5 - room ;rooms inside strX (stretching rooms).
  xray1 xray2  comp1 comp2 admin1 admin2 - room ;central rooms.
  emerg1 emerg2 emerg3 emerg4 storage1 storage2 stairs1 stairs2 - room ;hall rooms.
  
  ;;DOORS:
  v16 v11 s1 s1i s2 s2i s32 s36 s4 s4i s5 s5i - door ;external room doors.
  x1 x2 c1 c2 a1 a2 - door ;central room doors.
  e1 e2 e3 e4 g1 g2 t1 t2 - door ;hall doors.
  
  ;;CORRIDORS:
  corridor1 corridor2 corridor3 corridor4 corridor5 corridor6 - corridor
  reception hall - corridor

  ;;ZONES:
  ;zoneX zoneY zoneZ - zone
  
  ;;OBJECTS:
  object1 - object
)
(:init
  ;;CORRIDOR-CORRIDOR CONNECTIONS:
  (connected hall corridor1) ;hall
  (connected corridor1 hall)
  (connected hall corridor2)
  (connected corridor2 hall)

  (connected corridor1 reception) ;corridor1
  (connected reception corridor1)
  (connected corridor1 corridor3)
  (connected corridor3 corridor1)
  (connected corridor1 corridor4)
  (connected corridor4 corridor1)
  (connected corridor1 corridor5)
  (connected corridor5 corridor1)
  (connected corridor1 corridor6)
  (connected corridor6 corridor1)

  (connected corridor2 reception) ;corridor2
  (connected reception corridor2)
  (connected corridor2 corridor3)
  (connected corridor3 corridor2)
  (connected corridor2 corridor4)
  (connected corridor4 corridor2)
  (connected corridor2 corridor5)
  (connected corridor5 corridor2)

  ;;CORRIDOR-DOOR CONNECTIONS:
  (connected hall e1) ;hall
  (connected e1 hall)
  (connected hall e2)
  (connected e2 hall)
  (connected hall e3)
  (connected e3 hall)
  (connected hall e4)
  (connected e4 hall)
  (connected hall g1)
  (connected g1 hall)
  (connected hall g2)
  (connected g2 hall)
  (connected hall t1)
  (connected t1 hall)
  (connected hall t2)
  (connected t2 hall)

  (connected reception a1) ;reception
  (connected a1 reception)
  (connected reception a2)
  (connected a2 reception)

  (connected corridor1 v11) ;corridor1
  (connected v11 corridor1)
  (connected corridor1 s1)
  (connected s1 corridor1)
  (connected corridor1 s2)
  (connected s2 corridor1)

  (connected corridor2 s32) ;corridor2
  (connected s32 corridor2)
  (connected corridor2 s4)
  (connected s4 corridor2)
  (connected corridor2 s5)
  (connected s5 corridor2)

  (connected corridor3 c1) ;corridor3
  (connected c1 corridor3)
  (connected corridor3 c2)
  (connected c2 corridor3)

  (connected corridor4 x1) ;corridor4
  (connected x1 corridor4)
  (connected corridor4 x2)
  (connected x2 corridor4)

  (connected corridor6 v16) ;corridor6
  (connected v16 corridor6)
  (connected corridor6 s36)
  (connected s36 corridor6)

  ;;ROOM-DOOR CONNECTIONS:
  (connected visit1 v16) ;visit1
  (connected v16 visit1)
  (connected visit1 v11)
  (connected v11 visit1)

  (connected str1 s1) ;str1
  (connected s1 str1)
  (connected str1 s1i)
  (connected s1i str1)
  (connected inner1 s1i) ;inner1
  (connected s1i inner1)

  (connected str2 s2) ;str2
  (connected s2 str2)
  (connected str2 s2i)
  (connected s2i str2)
  (connected inner2 s2i) ;inner2
  (connected s2i inner2)

  (connected str3 s32) ;str3
  (connected s32 str3)
  (connected str3 s36)
  (connected s36 str3)

  (connected str4 s4) ;str4
  (connected s4 str4)
  (connected str4 s4i)
  (connected s4i str4)
  (connected inner4 s4i) ;inner4
  (connected s4i inner4)

  (connected str5 s5) ;str5
  (connected s5 str5)
  (connected str5 s5i)
  (connected s5i str5)
  (connected inner5 s5i) ;inner5
  (connected s5i inner5)

  (connected xray1 x1) ;xray1
  (connected x1 xray1)
  (connected xray2 x2) ;xray2
  (connected x2 xray2)

  (connected comp1 c1) ;comp1
  (connected c1 comp1)
  (connected comp2 c2) ;comp2
  (connected c2 comp2)

  (connected admin1 a1) ;admin1
  (connected a1 admin1)
  (connected admin2 a2) ;admin2
  (connected a2 admin2)

  (connected emerg1 e1) ;emerg1
  (connected e1 emerg1)
  (connected emerg2 e2) ;emerg2
  (connected e2 emerg2)
  (connected emerg3 e3) ;emerg3
  (connected e3 emerg3)
  (connected emerg4 e4) ;emerg4
  (connected e4 emerg4)

  (connected storage1 g1) ;storage1
  (connected g1 storage1)
  (connected storage2 g2) ;storage2
  (connected g2 storage2)

  (connected stairs1 t1) ;stairs1
  (connected t1 stairs1)
  (connected stairs1 t2) ;stairs2
  (connected t2 stairs1)

  ;;ALL DOOR CLOSED:
  (closed v16)
  (closed v11)
  (closed s1)
  (closed s1i)
  (closed s2)
  (closed s2i)
  (closed s32)
  (closed s36)
  (closed s4)
  (closed s4i)
  (closed s5)
  (closed s5i)
  (closed x1)
  (closed x2)
  (closed c1)
  (closed c2)
  (closed a1)
  (closed a2)
  (closed e1)
  (closed e2)
  (closed e3)
  (closed e4)
  (closed g1)
  (closed g2)
  (closed t1)
  (closed t2)

  ;;ROBOT POSITION:
  (robotAt brobot hall)

  ;;OBJECT POSITION:
  (objectAt object1 inner1)
)

(:goal  ;; to be achieved
  
  ;; our robot BROBOT is a polite robot that respects intimacy of humans 
  ;; and it will leave ALL the doors closed
  (and

    (objectAt object1 stairs2)

    (robotAt brobot stairs2)

    (closed v16)
    (closed v11)
    (closed s1)
    (closed s1i)
    (closed s2)
    (closed s2i)
    (closed s32)
    (closed s36)
    (closed s4)
    (closed s4i)
    (closed s5)
    (closed s5i)
    (closed x1)
    (closed x2)
    (closed c1)
    (closed c2)
    (closed a1)
    (closed a2)
    (closed e1)
    (closed e2)
    (closed e3)
    (closed e4)
    (closed g1)
    (closed g2)
    (closed t1)
    (closed t2)

  )
  

)
)
