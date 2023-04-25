(define (problem uav_generated_problem) (:domain uav) 
(:objects 
myUAV - uav 
urbanareas0 openareas0 openareas1 openareas2 openareas3 openareas4 openareas5 openareas6 openareas7 openareas8 openareas9 openareas10 openareas11 openareas12 openareas13 openareas14 openareas15 openareas16 openareas17 openareas18 openareas19 openareas20 openareas21 openareas22 openareas23 openareas24 openareas25 waters0 waters1 woods0 woods1 woods2 woods3 woods4 woods5 woods6 woods7 woods8 woods9 woods10 woods11 woods12 woods13 woods14 woods15 home -waypoint
)
(:init
(landed myUAV)
(at myUAV home)
)

(:goal (and

(landed myUAV)
))
)