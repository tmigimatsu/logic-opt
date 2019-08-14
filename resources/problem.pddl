(define (problem put-box-on-shelf)
	(:domain lgp)
	(:objects
		shelf far_shelf - physobj
		hook - movable
		box - throwable
	)
	(:init
		(inworkspace table)
		(inworkspace shelf)
		(inworkspace hook)
		(on hook table)
		(on box table)
	)
	(:goal (and
		(not (inhand hook))
		(not (inhand box))
		(on box shelf)
	))
)
