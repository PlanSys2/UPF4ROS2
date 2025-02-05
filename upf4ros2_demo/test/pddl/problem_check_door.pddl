(define (problem doorbell_attend_problem)
    (:domain doorbell_attend)
    (:objects
        livingroom entrance - wp
        door - door
        doorbell - sound
    )
    (:init
        ; the robot at start is at livingroom
        (robot_at livingroom)
        (door_at door entrance)
    )
    (:goal
        (door_checked door)
    )
)