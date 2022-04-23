(define (domain indoor-nav-strips-typed)
(:requirements :strips :typing)

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

(:action moveToLocation
  :parameters (?r - robot ?from ?to - location)
  :precondition 
    (and 
      (robotAt ?r ?from)
      (connected ?from ?to)
    )
  :effect 
    (and 
      (not (robotAt ?r ?from))
      (robotAt ?r ?to)
    )
)

(:action moveThroughDoor
  :parameters (?r - robot ?d - door ?from ?to - location)
  :precondition 
    (and 
      (opened ?d)
      (robotAt ?r ?from)
      (connected ?from ?d)
      (connected ?to ?d)
    )
  :effect 
    (and 
      (not (robotAt ?r ?from))
      (robotAt ?r ?to)
    )
)

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

(:action pick
  :parameters (?r - robot ?o - object ?l - location)
  :precondition 
    (and 
      (robotAt ?r ?l)
      (objectAt ?o ?l)
    )
  :effect 
    (and 
      (not (objectAt ?o ?l))
      (objectIn ?o ?r)
    )
)

(:action drop
  :parameters (?r - robot ?o - object ?l - location)
  :precondition 
    (and 
      (robotAt ?r ?l)
      (objectIn ?o ?r)
    )
  :effect 
    (and 
      (not (objectIn ?o ?r))
      (objectAt ?o ?l)
    )
)

(:action open
  :parameters (?r - robot ?d - door ?from - location)
  :precondition 
    (and 
      (robotAt ?r ?from)
      (connected ?from ?d)
      (closed ?d)
    )
  :effect 
    (and 
      (not (closed ?d)) ;;closen't
      (opened ?d)
    )
)

(:action closed
  :parameters (?r - robot ?d - door ?from - location)
  :precondition 
    (and 
      (robotAt ?r ?from)
      (connected ?from ?d)
      (opened ?d)
    )
  :effect 
    (and 
      (not (opened ?d)) ;;open't
      (closed ?d)
    )
)


)

