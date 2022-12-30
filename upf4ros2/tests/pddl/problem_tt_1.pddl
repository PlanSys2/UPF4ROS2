(define (problem simple_1)
  (:domain simple)
  (:objects
    leia - robot
    kitchen bedroom - room
  )
  (:init
    (robot_at leia kitchen)
    (connected kitchen bedroom)
  )

  ;; The goal is to have both packages delivered to their destinations:
  (:goal (and
    (robot_at leia bedroom) 
    )
  )
  )
