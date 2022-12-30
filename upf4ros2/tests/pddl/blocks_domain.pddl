(define (domain blocks)
(:requirements :strips :equality)

(:predicates 
  (on ?x ?y)
  (clear ?x)
  (block ?b)
)

(:constants Table)

(:action move
  :parameters (?b ?from ?to)
  :precondition 
    (and 
      (block ?b) 
      (block ?to) 
      (clear ?b)
      (clear ?to)
      (on ?b ?from)
      (not (= ?b ?from))
      (not (= ?b ?to))
      (not (= ?from ?to))
    )
  :effect 
    (and 
      (on ?b ?to)
      (clear ?from)
      (not (on ?b ?from))
      (not (clear ?to))
    )
)

(:action move_to_table
  :parameters (?b ?x)
  :precondition 
    (and 
      (block ?b)
      (block ?x)
      (on ?b ?x)
      (clear ?b)
      (not (= ?b ?x))
    )
  :effect 
    (and 
      (on ?b Table)
      (clear ?x)
      (not (on ?b ?x))
    )
)
)

