(define (problem MonkeyBanana-problem)
(:domain MonkeyBanana)

(:objects M Ban Box
          Lab A B C
          CG Floor)

(:init (LOCATION A) (LOCATION B) (LOCATION C) (LABORATORY Lab)
       (BANANAS Ban) (MONKEY M) (BOX Box) 
       (CEILING CG) (FLOOR Floor)
       (IN A Lab) (IN B Lab) (IN C Lab)
       (AT M C) (AT Box A) (AT Ban B)
       (HANGS Ban CG) (ABOVE Ban B) (LIVES M Lab)
       (ON-FLOOR M Floor) (ON-FLOOR Box Floor))

(:goal (and (HAS M Ban))))