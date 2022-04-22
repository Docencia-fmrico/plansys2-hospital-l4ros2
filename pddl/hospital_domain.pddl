(define (domain indoor-nav-strips-typed)
(:requirements :strips :typing :adl :fluents :durative-actions)

;;TYPES

(:types
room corridor zone - location
location door - connectable
object
robot
)

;;PREDICATES

(:predicates 
  (robotAt ?r - robot ?l - location)
  (objectAt ?o - object ?l - location)
  (connected ?c1 ?c2 - connectable)
  (closed ?d - door)
  (opened ?d - door)
  (objectIn ?o - object ?r - robot)
  ;(elevatorIn ?e - elevator ?c - corridor)
)

;;ACTIONS

(:durative-action moveToLocation
    :parameters (?r - robot ?from ?to - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?from ?to))
        (at start(robotAt ?r ?from))
        )
    :effect (and
        (at start(not(robotAt ?r ?from)))
        (at end(robotAt ?r ?to))
    )
)

;(:durative-action moveThroughDoor
;  :parameters (?r - robot ?d - door ?from ?to - location)
;  :duration ( = ?duration 5)
;  :condition 
;    (and 
;      (at start(opened ?d))
;      (at start(robotAt ?r ?from))
;      (at start(connected ?from ?d))
;      (at start(connected ?to ?d))
;    )
;  :effect 
;    (and 
;      (at start(not (robotAt ?r ?from)))
;      ((at end(robotAt ?r ?to)))
;    )
;)

;(:action moveThroughElevator
;  :parameters (?r - robot ?e - elevator ?from ?to - corridor)
;  :precondition 
;    (and 
;      (elevatorIn ?e ?from)
;      (robotAt ?r ?from)
;      (connected ?from ?e)
;      (connected ?to ?e)
;    )
;  :effect 
;    (and 
;      (not (elevatorIn ?e ?from))
;      (elevatorIn ?e ?to)
;      (not (robotAt ?r ?from))
;      (robotAt ?r ?to)
;    )
;)

;(:action callElevator
;  :parameters (?r - robot ?e - elevator ?from ?to - corridor)
;  :precondition 
;    (and 
;      (elevatorIn ?e ?from)
;      (robotAt ?r ?to)
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
;      (at start(robotAt ?r ?l))
;      (at start(objectAt ?o ?l))
;    )
;  :effect 
;    (and 
;      (at start(not (objectAt ?o ?l)))
;      (at end(objectIn ?o ?r))
;    )
;)

;(:durative-action drop
;  :parameters (?r - robot ?o - object ?l - location)
;  :duration ( = ?duration 3)
;  :condition 
;    (and 
;      (at start(robotAt ?r ?l))
;      (at start(objectIn ?o ?r))
;    )
;  :effect 
;    (and 
;      (at start(not (objectIn ?o ?r)))
;      (at end(objectAt ?o ?l))
;    )
;)

;(:durative-action open 
;  :parameters (?r - robot ?d - door ?from - location)
;  :duration ( = ?duration 3)
;  :precondition 
;    (and 
;      (at start(robotAt ?r ?from))
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
;      (at start(robotAt ?r ?from))
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

