(define (domain indoor-nav-strips-typed)
(:requirements :strips :typing :adl :fluents :durative-actions)

;;TYPES

(:types
room corridor zone door - location
object
robot
)

;;PREDICATES

(:predicates 
  (robot_at ?r - robot ?l - location)
  (object_at ?o - object ?l - location)
  (connected ?c1 ?c2 - location)
  (closed ?d - door)
  (opened ?d - door)
  (object_in ?o - object ?r - robot)
  ;(elevatorIn ?e - elevator ?c - corridor)
)

;;ACTIONS

(:durative-action move_to_location
    :parameters (?r - robot ?from ?to - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?from ?to))
        (at start(robot_at ?r ?from))
        )
    :effect (and
        (at start(not(robot_at ?r ?from)))
        (at end(robot_at ?r ?to))
    )
)

;(:durative-action move_through_door
;  :parameters (?r - robot ?d - door ?from ?to - location)
;  :duration ( = ?duration 5)
;  :condition 
;    (and 
;      (at start(opened ?d))
;      (at start(robot_at ?r ?from))
;      (at start(connected ?from ?d))
;      (at start(connected ?to ?d))
;    )
;  :effect 
;    (and 
;      (at start(not (robot_at ?r ?from)))
;      ((at end(robot_at ?r ?to)))
;    )
;)

;(:action move_through_elevator
;  :parameters (?r - robot ?e - elevator ?from ?to - corridor)
;  :precondition 
;    (and 
;      (elevatorIn ?e ?from)
;      (robot_at ?r ?from)
;      (connected ?from ?e)
;      (connected ?to ?e)
;    )
;  :effect 
;    (and 
;      (not (elevatorIn ?e ?from))
;      (elevatorIn ?e ?to)
;      (not (robot_at ?r ?from))
;      (robot_at ?r ?to)
;    )
;)

;(:action call_elevator
;  :parameters (?r - robot ?e - elevator ?from ?to - corridor)
;  :precondition 
;    (and 
;      (elevatorIn ?e ?from)
;      (robot_at ?r ?to)
;      (connected ?from ?e)
;      (connected ?to ?e)
;    )
;  :effect 
;    (and 
;      (not (elevatorIn ?e ?from))
;      (elevatorIn ?e ?to)
;    )
;)

;(:durative-action pick
;  :parameters (?r - robot ?o - object ?l - location)
;  :duration ( = ?duration 3)
;  :condition 
;    (and 
;      (at start(robot_at ?r ?l))
;      (at start(object_at ?o ?l))
;    )
;  :effect 
;    (and 
;      (at start(not (object_at ?o ?l)))
;      (at end(object_in ?o ?r))
;    )
;)

;(:durative-action drop
;  :parameters (?r - robot ?o - object ?l - location)
;  :duration ( = ?duration 3)
;  :condition 
;    (and 
;      (at start(robot_at ?r ?l))
;      (at start(object_in ?o ?r))
;    )
;  :effect 
;    (and 
;      (at start(not (object_in ?o ?r)))
;      (at end(object_at ?o ?l))
;    )
;)

;(:durative-action open 
;  :parameters (?r - robot ?d - door ?from - location)
;  :duration ( = ?duration 3)
;  :precondition 
;    (and 
;      (at start(robot_at ?r ?from))
;      (at start(connected ?from ?d))
;      (at start(closed ?d))
;    )
;  :effect 
;    (and 
;      (at start(not (closed ?d))) ;;closen't
;      (at end(opened ?d))
;    )
;)

;(:durative-action closed
;  :parameters (?r - robot ?d - door ?from - location)
;  :duration ( = ?duration 3)
;  :precondition 
;    (and 
;      (at start(robot_at ?r ?from))
;      (at start(connected ?from ?d))
;      (at start(opened ?d))
;    )
;  :effect 
;    (and 
;      (at start(not (opened ?d))) ;;open't
;      (at end(closed ?d))
;    )
;)


)

