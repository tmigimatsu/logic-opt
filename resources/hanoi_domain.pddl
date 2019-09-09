(define (domain hanoi)
	(:requirements :strips :typing :equality :negative-preconditions :conditional-effects)
	(:types
		physobj - object
		movable - physobj
	)
	(:constants
		platform_left - physobj
		platform_middle - physobj
		platform_right - physobj
	)
	(:predicates
		(inhand ?a - movable)
		(on ?a - movable ?b - physobj)
		(smaller ?a - physobj ?b - physobj)
	)
	(:action pick
		:parameters (?a - movable)
		:precondition (forall
			(?b - movable)
			(and
				(not (inhand ?b))
				(not (on ?b ?a))
			)
		)
		:effect (and
			(inhand ?a)
			(forall (?b - physobj) (not (on ?a ?b)))
		)
	)
	(:action place
		:parameters (?a - movable ?b - physobj)
		:precondition (and
			(inhand ?a)
			(smaller ?a ?b)
			(forall (?c - movable) (not (on ?c ?b)))
		)
		:effect (and
			(not (inhand ?a))
			(on ?a ?b)
		)
	)
)
