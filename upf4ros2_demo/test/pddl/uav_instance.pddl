
(define (problem uav_simple_problem) (:domain uav)
    (:objects
        myUAV - uav
        home waypoint1 waypoint2 waypoint3 - waypoint
    )

    (:init
        (at myUAV home)
        (landed myUAV)
    )

    (:goal (and
        (at myUAV home)
        (landed myUAV)
        (visited myUAV waypoint1)
        (visited myUAV waypoint2)
        (visited myUAV waypoint3)
    ))
)
