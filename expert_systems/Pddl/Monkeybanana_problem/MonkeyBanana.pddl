(define (domain MonkeyBanana)

(:predicates (MONKEY ?x) (BANANAS ?x) (BOX ?x)
             (LOCATION ?x) (LABORATORY ?x) (CEILING ?x) (FLOOR ?x)
             (IN ?x ?y) (AT ?x ?y) (ABOVE ?x ?y) (ON-FLOOR ?x ?y) (ON-BOX ?x ?y)
             (LIVES ?x ?y) (HANGS ?x ?y) (HAS ?x ?y) 
             )

(:action MOVE :parameters (?M ?Lab ?A ?B ?Floor ?Box)
:precondition (and (LIVES ?M ?Lab) (IN ?A ?Lab) (IN ?B ?Lab) (AT ?M ?A) (ON-FLOOR ?M ?Floor)
                    (not (ON-BOX ?M ?Box))
                )

:effect (and (AT ?M ?B)
             (not (AT ?M ?A)))
             )
             
(:action PUSH :parameters (?M ?Lab ?A ?B ?Floor ?Box)
:precondition (and (LIVES ?M ?Lab) (IN ?A ?Lab) (IN ?B ?Lab) (AT ?M ?A) (AT ?Box ?A) (ON-FLOOR ?M ?Floor) (ON-FLOOR ?Box ?Floor)
                    (not (ON-BOX ?M ?Box))
                )

:effect (and (AT ?Box ?B)
             (not (AT ?Box ?A)))
             )

(:action CLIMBUp :parameters (?M ?Lab ?A ?B ?Floor ?Box)
:precondition (and (LIVES ?M ?Lab) (IN ?A ?Lab) (IN ?B ?Lab) (AT ?M ?A) (AT ?Box ?A) (ON-FLOOR ?M ?Floor) (ON-FLOOR ?Box ?Floor)
                    (not (ON-BOX ?M ?Box))
                )

:effect (and (ON-BOX ?M ?Box)
             (not (ON-FLOOR ?M ?Floor)))
             )
             

(:action CLIMBDown :parameters (?M ?Lab ?A ?Floor ?Box)
:precondition (and (LIVES ?M ?Lab) (IN ?A ?Lab) (AT ?M ?A) (AT ?Box ?A) (ON-FLOOR ?Box ?Floor) (ON-BOX ?M ?Box)
                    (not (ON-FLOOR ?M ?Floor))
                )

:effect (and (ON-FLOOR ?M ?Floor)
             (not (ON-BOX ?M ?Box)))
             )
             
(:action GRAB :parameters (?M ?Lab ?A ?Floor ?Box ?Ban ?CG)
:precondition (and (LIVES ?M ?Lab) (IN ?A ?Lab) (AT ?M ?A) (AT ?Box ?A) (ON-FLOOR ?Box ?Floor) (ON-BOX ?M ?Box)
                    (HANGS ?Ban ?CG) (ABOVE ?Ban ?A)
                    (not (ON-FLOOR ?M ?Floor))
                    (not (HAS ?M ?Ban))
                )

:effect (and (HAS ?M ?Ban)
             (not (HANGS ?Ban ?CG)))
             )


)