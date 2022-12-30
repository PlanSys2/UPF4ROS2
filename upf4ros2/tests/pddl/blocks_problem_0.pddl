(define (problem blocks1)
(:domain blocks)
(:objects 
  a b c d
)
(:init 
  (block a)
  (block b)
  (block c)
  (block d)
  (clear d)
  (clear b)
  (clear c)
  (on c Table)
  (on d Table)
  (on a Table)
  (on b a)
)

(:goal (and
  (on a b)(on b d)(on d c)  (on c Table)
 )
)

)
