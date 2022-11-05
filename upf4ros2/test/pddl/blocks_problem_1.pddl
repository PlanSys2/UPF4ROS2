(define (problem blocks1)
(:domain blocks)
(:objects 
  a b c
)
(:init 
  (block a)
  (block b)
  (block c)
  (clear b)
  (clear c)
  (on c Table)
  (on a Table)
  (on b a)
)

(:goal (and
  (on b c))
)

)
