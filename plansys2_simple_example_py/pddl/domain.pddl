(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions :conditional-effects)


(:types
robot
location
person
state
consciousness
stairs
)

(:constants stand sit lay unknown - state
            conscious unconscious confused - consciousness)

(:predicates
(robot_at ?r - robot ?l - location)
(connected ?lo1 ?lo2 - location)
(person_detected ?p - person ?l - location)
(searched ?r - robot ?l - location)
(person_evaluated ?p - person)
(battery_unchecked ?r - robot)
(battery_checked ?r - robot)
(is_free ?r - robot)
(person_state ?p - person ?s - state)
(person_reported ?p - person)
(next_move ?r - robot ?l - location)
(dialog_finished ?p - person)
(person_dialog ?p - person ?c - consciousness)
(emergency ?r - robot)
(not_emergency ?r - robot)
(gas_unchecked ?l - location)
(gas_checked ?l - location)
(crack_checked ?l -location)
(is_exit ?l - location)
(stairs_connected ?lo1 - location ?s - stairs)
(stairs_checked ?s - stairs ?l1 ?l2 -location)
(environment_checked ?l - location)
(link_checked ?r1 ?r2 - location)
(room_open ?l - location)
(door_closed ?l - location)
(door_blocked ?l -location)
(door_notchecked ?l - location)
)


(:functions

)


(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(battery_checked ?r))
        (at start(connected ?r1 ?r2))
        (at start(link_checked ?r1 ?r2))
        (at start(robot_at ?r ?r1))
    )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
        (at end(battery_unchecked ?r))
        (at end(not(battery_checked ?r)))

    )
)

(:durative-action search
    :parameters (?r - robot ?l - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (over all(robot_at ?r ?l))
        (over all(not_emergency ?r))
        (over all(room_open ?l))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(searched ?r ?l))
        (at end(battery_unchecked ?r))
        (at end(not(battery_checked ?r)))
    )
)


(:durative-action evaluate
    :parameters (?l - location ?p - person ?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(person_detected ?p ?l))
        (over all(robot_at ?r ?l))
        (over all(not_emergency ?r))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(person_evaluated ?p))
    )
)

(:durative-action dialog
    :parameters (?l - location ?p - person ?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(person_evaluated ?p))
        (over all(robot_at ?r ?l))
        (over all(not_emergency ?r))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(dialog_finished ?p))
    )
)

(:durative-action report
    :parameters (?l - location ?p - person ?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(dialog_finished ?p))
        (over all(robot_at ?r ?l))
        (over all(not_emergency ?r))
    )
    :effect(and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(person_reported ?p))
    )
)

(:durative-action check
    :parameters (?r - robot ?from - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(battery_unchecked ?r))
        (at start(is_free ?r))
        (at start(robot_at ?r ?from))

    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(not(battery_unchecked ?r)))
        (at end(battery_checked ?r))
    )
)

(:durative-action checkgas
    :parameters (?r -robot ?l -location)
    :duration( = ?duration 5)
    :condition (and 
        (over all(robot_at ?r ?l))
        (at start(is_free ?r))
        (at start(not_emergency ?r))
        (over all (room_open ?l))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(gas_checked ?l))
    )

)

(:durative-action checkcrack
    :parameters (?r -robot ?l -location)
    :duration( = ?duration 5)
    :condition (and 
        (over all(robot_at ?r ?l))
        (at start(is_free ?r))
        (at start(not_emergency ?r))
        (over all(gas_checked ?l))
        (over all (room_open ?l))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(crack_checked ?l))
    )

)

(:durative-action checktemp
    :parameters (?r -robot ?l -location)
    :duration( = ?duration 5)
    :condition (and 
        (over all(robot_at ?r ?l))
        (at start(is_free ?r))
        (at start(not_emergency ?r))
        (over all(crack_checked ?l))
        (over all (room_open ?l))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(environment_checked ?l))
    )

)


(:durative-action reportemergency
    :parameters (?r -robot ?l -location)
    :duration( = ?duration 5)
    :condition (and 
        (at start(robot_at ?r ?l))
        (at start(is_free ?r))
        (over all(is_exit ?l))
        (at start(emergency ?r))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(not_emergency ?r))
        (at end(not(emergency ?r)))
        
    )
)

(:durative-action checkstairs
    :parameters (?r -robot ?s - stairs ?from ?to - location)
    :duration( = ?duration 5)
    :condition (and 
        (over all(robot_at ?r ?from))
        (over all(stairs_connected ?from ?s))
        (over all(stairs_connected ?to ?s))
        (at start(is_free ?r))
        (over all(not_emergency ?r))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(stairs_checked ?s ?from ?to))
        (at end (stairs_checked ?s ?to ?from))
    )
)

(:durative-action checklink
    :parameters (?r -robot ?from ?to - location)
    :duration( = ?duration 5)
    :condition (and 
        (over all(robot_at ?r ?from))
        (over all (connected ?from ?to))
        (at start(is_free ?r))
        
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(link_checked ?from ?to))
        (at end(link_checked ?to ?from))
    )
)


(:durative-action climbstairs
    :parameters (?r - robot ?r1 ?r2 - location ?s - stairs)
    :duration ( = ?duration 5)
    :condition (and
        (at start(stairs_connected ?r1 ?s))
        (at start(stairs_connected ?r2 ?s))
        (at start(stairs_checked ?s ?r1 ?r2))
        (at start(robot_at ?r ?r1))
    )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
        (at end(battery_unchecked ?r))
        (at end(not(battery_checked ?r)))
    )
)

(:durative-action checkroomopen
    :parameters (?r - robot ?l - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(robot_at ?r ?l))
        (at start (door_notchecked ?l))
        (over all(not_emergency ?r))


    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(room_open ?l))
        (at end(not(door_notchecked ?l)))
    )
)

(:durative-action opendoor
    :parameters (?r - robot ?l - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(robot_at ?r ?l))
        (at start(door_closed ?l))
        (over all(not_emergency ?r))

    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(not(door_closed ?l)))
        (at end(room_open ?l))
    )
)

(:durative-action searchoutside
    :parameters (?r - robot ?l - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (over all(robot_at ?r ?l))
        (over all(not_emergency ?r))
        (at start(door_blocked ?l))
        (at start(not_emergency ?r))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(searched ?r ?l))
        (at end(battery_unchecked ?r))
        (at end(not(battery_checked ?r)))
    )
)








)
