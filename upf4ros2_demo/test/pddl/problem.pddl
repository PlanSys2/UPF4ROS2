(define (problem test_problem)
    (:domain test)
    (:objects
        ; locations
        l1 l2 - location
    )
    (:init
        ; the robot at start is at l1
        (robot_at l1)
    )
    (:goal
        (robot_at l2)
    )
)