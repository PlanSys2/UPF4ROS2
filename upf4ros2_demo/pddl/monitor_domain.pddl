(define 
(domain monitor)
(:types 
	drone waypoint - object
)
(:predicates
	(at ?d - drone ?w - waypoint)
	(inspected ?w - waypoint )
	(taken_off ?d - drone)
    (landed ?d - drone)
)
(:action inspect
	:parameters (?d - drone ?w - waypoint)
	:precondition (and (at ?d ?w) (taken_off ?d) (not (inspected ?w)) )
	:effect (and (inspected ?w))
)

(:action fly
	:parameters (?d - drone ?w1 - waypoint ?w2 - waypoint) 
	:precondition (and (at ?d ?w1) (taken_off ?d))
	:effect (and (not (at ?d ?w1)) (at ?d ?w2))
)

(:action take_off
	:parameters (?d - drone)
	:precondition (landed ?d)
	:effect (and (taken_off ?d) (not (landed ?d)))
)

(:action land
	:parameters (?d - drone)
	:precondition (taken_off ?d)
	:effect (and (landed ?d) (not (taken_off ?d)))
)

)