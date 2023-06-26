from gtsg.monitorGame import MonitorSG
from upf4ros2_demo.pddl_from_sg import ProblemMSG
import numpy as np






tmp_zeros =np.zeros((2 ,4))
mobis =MonitorSG(2 ,4,
                [[0, 0, 0, 0], [1, 1, 1, 1]],
                [[[1-0.855, 0.855], [1-0.64, 0.64]],
                 [[1-0.64, 0.64],  [1-0.35, 0.35]],
                 [[1-0.56, 0.56], [1-0.36, 0.36 ]],
                 [[1-0.42, 0.42], [1-0.42, 0.42]]],
                tmp_zeros
                )

print([s.s_game.utilities for s in mobis.sg.states.values()])

probpddl=ProblemMSG(mobis,5)
probpddl.problem_gen((0,1,0,0),0)
