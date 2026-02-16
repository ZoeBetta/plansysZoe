(define (problem spot_problem_1)
  (:domain spot_domain)

(:objects
  spot - robot
  stand lay sit unknown - state
  conscious unconscious confused - consciousness
  exit - location
  r00f0 - location
  r01f0 - location
  r10f0 - location
  r11f0 - location
  p1 - person
)

(:init
  (is_exit exit)
  (is_free spot)
  (robot_at spot r01f0)
  (not_emergency spot)
  (connected r10f0 r00f0)
  (connected r00f0 r10f0)
  (connected r01f0 r00f0)
  (connected r00f0 r01f0)
  (connected r11f0 r01f0)
  (connected r01f0 r11f0)
  (connected r11f0 r10f0)
  (connected r10f0 r11f0)
  (connected exit r00f0)
  (connected r00f0 exit)
  (door_notchecked r00f0)
  (door_notchecked r01f0)
  (person_detected p1 r01f0)
  (door_notchecked r10f0)
  (door_notchecked r11f0)
)

(:goal
(and
  (searched spot r00f0) (environment_checked r00f0) 
  (person_evaluated p1) (person_reported p1) (dialog_finished p1) 
  (searched spot r01f0) (environment_checked r01f0) 
  (searched spot r10f0) (environment_checked r10f0) 
  (searched spot r11f0) (environment_checked r11f0) 
)
))