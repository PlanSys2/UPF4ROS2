(define (domain doorbell_attend)

    (:types
        wp
        sound
        door
    )

    (:predicates
        ; the robot is at location ?l
        (robot_at ?l - wp)
        (door_at ?d - door ?l - wp)
        (door_checked ?d - door)
        (sound_listened ?s - sound)
    )

    ; navigation action
    (:durative-action move
        :parameters (?l_from ?l_to - location)
        :duration (= ?duration 10)
        :condition (and
            (robot_at ?l_from)
        )
        :effect (and
            (not (robot_at ?l_from))
            (robot_at ?l_to)
        )
    )

    ; check door action
    (:durative-action check_door
        :parameters (?d - door ?entrance - wp)
        :duration (= ?duration 10)
        :condition (and
            (door_at ?d ?entrance)
            (robot_at ?entrance)
        )
        :effect (and
            (door_checked ?d)
        )
    )

)