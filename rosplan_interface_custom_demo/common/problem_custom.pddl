(define (problem task)
(:domain turtlebot)
(:objects
    wp0 wp1 - waypoint
    kenny - robot
)
(:init
    (= (charge kenny) 0)
    (robot_at kenny wp0)
    (docked kenny)
    (dock_at wp0)
)
(:goal (and
    (visited wp0)
    (visited wp1)
    (docked kenny)
    (>  (charge kenny) 0)

))
)

