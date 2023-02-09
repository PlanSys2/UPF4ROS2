(define (problem test_problem)
    (:domain test)
    (:objects
        ; locations
        livingroom entrance - location
    )
    (:init
        ; the robot at start is at livingroom
        (robot_at livingroom)
    )
    (:goal
        (robot_at entrance)
    )
)