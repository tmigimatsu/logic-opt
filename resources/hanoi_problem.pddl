(define (problem tower-of-hanoi)
	(:domain hanoi)
	(:objects
		box_3 - movable
		box_2 - movable
		box_1 - movable
	)
	(:init
		(smaller box_3 platform_left)
		(smaller box_3 platform_middle)
		(smaller box_3 platform_right)
		(smaller box_2 platform_left)
		(smaller box_2 platform_middle)
		(smaller box_2 platform_right)
		(smaller box_2 box_3)
		(smaller box_1 platform_left)
		(smaller box_1 platform_middle)
		(smaller box_1 platform_right)
		(smaller box_1 box_3)
		(smaller box_1 box_2)
		(on box_1 box_2)
		(on box_2 box_3)
		(on box_3 platform_right)
	)
	(:goal (or
		(and
			(on box_1 box_2)
			(on box_2 box_3)
			(on box_3 platform_middle)
		)
		(and
			(on box_1 box_2)
			(on box_2 box_3)
			(on box_3 platform_left)
		)
	))
	)
)
