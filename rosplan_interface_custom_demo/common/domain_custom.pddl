(define (domain turtlebot_demo)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	waypoint 
	robot
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(connected ?from ?to - waypoint)
	(visited ?wp - waypoint)
	(undocked ?v - robot)
	(docked ?v - robot)
	(localised ?v - robot)
	(dock_at ?wp - waypoint)
)

(:functions
    (charge ?r - robot)
	(distance ?wp1 ?wp2 - waypoint) 
)

;; Move between any two waypoints, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at ?v ?from)))
	:effect (and
		(at end (visited ?to))
		(at start (not (robot_at ?v ?from)))
		(at end (robot_at ?v ?to)))
)

;; Localise
(:durative-action localise
	:parameters (?v - robot)
	:duration ( = ?duration 60)
	:condition (over all (undocked ?v))
	:effect (at end (localised ?v))
)

;; Dock to charge
(:durative-action dock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 30)
	:condition (and
		(over all (dock_at ?wp))
		(at start (robot_at ?v ?wp))
		(at start (undocked ?v)))
	:effect (and
		(at start (not (undocked ?v)))
		(at end (docked ?v)))
)

(:durative-action undock
	:parameters (?v - robot ?wp - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(over all (dock_at ?wp))
		(at start (docked ?v)))
	:effect (and
		(at start (not (docked ?v)))
		(at end (undocked ?v)))
)

)

