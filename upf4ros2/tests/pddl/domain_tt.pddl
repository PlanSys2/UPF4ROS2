(define (domain simple)

(:types robot room)

(:predicates
  (robot_at ?r - robot ?ro - room)
  (connected ?r1 ?r2 - room)

)

(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?r1))
        (at start(connected ?r1 ?r2)))
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)
)