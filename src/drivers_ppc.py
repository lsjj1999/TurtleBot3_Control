# Implementing the driver using Pure Pursuit algorithm assignmnet

import numpy as np
import math

class PurePursuitDriver:

    def pure_pursuit_control(self, pose_x, pose_y, pose_theta, ref):
        Lf = 0.5
        pose = [pose_x, pose_y]
        
        for i in range(len(ref)-1):
            lp = ref[0] 
            lookahead_idx = 0
            if i < len(ref)-1:
                # if math.dist(pose, ref[i]) < Lf and math.dist(pose, ref[i+1]) > Lf:
                if np.linalg.norm(pose - ref[i]) < Lf and np.linalg.norm(pose - ref[i+1]) > Lf:
                    lp = ref[i+1]
                    lookahead_idx = i+1
                    break
            else:
                lp = ref[0]
                lookahead_idx = 0
                # break
                
        theta = np.arctan2(lp[1] - pose_y,lp[0] - pose_x) - pose_theta
        R = 2*np.sin(theta)/Lf
        delta = math.atan(R)
        steering_angle = delta


        R_max = 0.075
        speed_min = 9.5
        speed_max = 25.0
        R = abs(R)
        if R <= 0.0:
             y = speed_max
        elif R > R_max:
            y = speed_min
        else:
            y = -(speed_max - speed_min)/(R_max*R_max*R_max)*(R-R_max)*(R-R_max)*(R-R_max) + speed_min

        speed = 0.5
        # print(y)
        return speed, steering_angle, lookahead_idx 