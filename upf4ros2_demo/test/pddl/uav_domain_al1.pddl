(define (domain uav)
    (:types uav -object waypoint -object)
    (:predicates  
        (at  ?x - waypoint)
        (taken_off ?x - uav)
        (landed ?x - uav)
        (visited ?x - waypoint)
    )

    (:action fly
        :parameters (?x - uav ?y - waypoint ?z - waypoint)
        :precondition (and (at ?y) (taken_off ?x))
        :effect (and (not (at ?y)) (at ?z) (visited ?z))
    )

    (:action take_off
        :parameters (?x - uav)
        :precondition (landed ?x)
        :effect (and (taken_off ?x) (not (landed ?x)))
    )

    (:action land
        :parameters (?x - uav)
        :precondition (taken_off ?x)
        :effect (and (landed ?x) (not (taken_off ?x)))
    )
)