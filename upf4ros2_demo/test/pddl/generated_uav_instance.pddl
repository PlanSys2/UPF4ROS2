(define (problem uav_generated_problem) (:domain uav) 
(:objects 
myUAV - uav 
urbanAreas0 openAreas0 openAreas1 openAreas2 openAreas3 openAreas4 openAreas5 openAreas6 openAreas7 openAreas8 openAreas9 openAreas10 openAreas11 openAreas12 openAreas13 openAreas14 openAreas15 openAreas16 openAreas17 openAreas18 openAreas19 openAreas20 openAreas21 openAreas22 openAreas23 openAreas24 openAreas25 waters0 waters1 woods0 woods1 woods2 woods3 woods4 woods5 woods6 woods7 woods8 woods9 woods10 woods11 woods12 woods13 woods14 woods15 home -waypoint
)
(:init
(landed myUAV)
(at myUAV home)
)

(:goal (and

(landed myUAV)
(visited myUAV urbanAreas0)
(visited myUAV waters0)
(visited myUAV woods14)
))
)