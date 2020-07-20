(define (problem task)
(:domain turtlebot_demo)
(:objects
    wp0 wp1 - waypoint
)
(:init

    (connected wp0 wp1)
    (connected wp1 wp0)






    (= (distance wp0 wp1) 2)
    (= (distance wp1 wp0) 2)

)
(:goal (and
))
)
