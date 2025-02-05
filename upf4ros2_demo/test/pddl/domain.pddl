(define (domain test)

    (:types
        location ; service areas, points of interest, navigation goals
    )

    (:predicates
        ; the robot is at location ?l
        (robot_at ?l - location)
    )

    ; navigation action
    (:action move
        :parameters (?l_from ?l_to - location)
        :precondition (and
            (robot_at ?l_from)
        )
        :effect (and
            (not (robot_at ?l_from))
            (robot_at ?l_to)
        )
    )

)