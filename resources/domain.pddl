(define (domain lgp)
	(:requirements :strips :typing :equality :negative-preconditions :conditional-effects)
	(:types
		physobj - object
		movable - physobj
		throwable - movable
	)
	(:constants table - physobj)
	(:predicates
		(inhand ?a - movable)
		(on ?a - movable ?b - physobj)
		(inworkspace ?a - physobj)
		(throwable ?a - movable)
	)
	(:action pick
		:parameters (?a - movable)
		:precondition (and
			(forall (?b - movable) (not (inhand ?b)))
			(inworkspace ?a)
		)
		:effect (and
			(inhand ?a)
			(forall (?b - physobj) (not (on ?a ?b)))
		)
	)
	(:action place
		:parameters (?a - movable ?b - physobj)
		:precondition (and
			(not (= ?a ?b))
			(inhand ?a)
			(inworkspace ?b)
		)
		:effect (and
			(not (inhand ?a))
			(on ?a ?b)
		)
	)
	;(:action throw
	;	:parameters (?a - throwable ?b - physobj)
	;	:precondition (and
	;		(not (= ?a ?b))
	;		(inhand ?a)
	;		(forall (?c - movable)
	;			(or
	;				(= ?a ?c)
	;				(not (on ?c ?a))
	;				(throwable ?c)
	;			)
	;		)
	;	)
	;	:effect (and
	;		(not (inhand ?a))
	;		(on ?a ?b)
	;		(when
	;			(not (inworkspace ?b))
	;			(not (inworkspace ?a))
	;		)
	;	)
	;)
	;(:action slide
	;	:parameters (?a - movable ?b - physobj)
	;	:precondition (and
	;		(not (= ?a ?b))
	;		(inhand ?a)
	;		(inworkspace ?b)
	;	)
	;	:effect (on ?a ?b)
	;)
	(:action push
		:parameters (?a ?b - movable ?c - physobj)
		:precondition (and
			(not (= ?a ?b))
			(not (= ?a ?c))
			(not (= ?b ?c))
			(inhand ?a)
			(on ?b ?c)
			(inworkspace ?c)
		)
		:effect (inworkspace ?b)
	)
)
