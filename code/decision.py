import numpy as np
def get_bearing(x_r, y_r, x_h, y_h, yaw):
    dy = y_r - y_h
    dx = x_r - x_h
    if dx == 0:
        if y_r > y_h:
            theta = 90
        else:
            theta = 270
    else:
        theta = np.arctan2(dy, dx) * 180/np.pi

    bearing = 180 + theta - yaw

    # Corrections that lead to smoother path planning
    if bearing > 180:
        bearing = bearing - 360
    elif bearing < -180:
        bearing = 360 + bearing
    elif np.absolute(bearing) == 360:
        bearing = 0
        
    return bearing

def calc_dist(pos1, pos2):
    return np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1] - pos2[1])**2)


# Auxiliary function that implements careful forward motion 
def forward_aux(Rover):

    # If a rock is seen, then enter get_rock mode
    if Rover.mode != 'stuck' and len(Rover.rock_angles) >= Rover.rock_collect_threshold:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'get_rock'
        return Rover

    # Get pixels in a vertical band ahead of the rover, 40 pixels wide
    mid_x = []
    mid_y = []

    for i in range(0, len(Rover.nav_x)):
        if Rover.nav_y[i] <= 20 and Rover.nav_y[i] >= -20:
            mid_x.append(Rover.nav_x[i])
            mid_y.append(Rover.nav_y[i])
    mid_x = np.array(mid_x)
    mid_y = np.array(mid_y)

        
    # Extremely low room to the side, then turn hard
    if len(mid_y[mid_y >= 0]) < Rover.stop_side:
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = -15
        return Rover
    elif len(mid_y[mid_y < 0]) < Rover.stop_side:
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = 15
        return Rover
    # Much more room to right, turn more gradually
    elif len(mid_y[mid_y < 0]) >= Rover.left_right_ratio * len(mid_y[mid_y >= 0]):
        Rover.steer = np.clip(np.mean(Rover.nav_angles[Rover.nav_angles < 0]) * 180/np.pi, -15, 15)
    # Much more room to left, turn more gradually
    elif len(mid_y[mid_y >= 0]) >= Rover.left_right_ratio * len(mid_y[mid_y < 0]):
        Rover.steer = np.clip(np.mean(Rover.nav_angles[Rover.nav_angles >= 0]) * 180/np.pi, -15, 15)
    # If on the way home, follow the bearing to home as closely as possible
    elif Rover.samples_collected == 6 and (Rover.pos[0] < 84 or Rover.pos[0] > 97 or Rover.pos[1] < 81 or Rover.pos[1] > 88):
        lo = np.percentile(Rover.nav_angles, 25)
        hi = np.percentile(Rover.nav_angles, 75)
        deg_angles = Rover.nav_angles[Rover.nav_angles < hi]
        deg_angles = deg_angles[deg_angles > lo] * 180/np.pi
        arr = np.absolute(deg_angles - get_bearing(Rover.pos[0], Rover.pos[1], Rover.init_pos[0], Rover.init_pos[1], Rover.yaw))
        idx = np.argmin(arr)
        Rover.steer = np.clip(deg_angles[idx], -15, 15)
    # Otherwise just follow the steer bias
    else:
        Rover.steer = np.clip(np.mean(Rover.nav_angles) * 180/np.pi + Rover.steer_bias, -15, 15)

    # Speed up if we're too slow
    if Rover.vel < Rover.max_vel:  
        Rover.throttle = Rover.throttle_set
    # Otherwise just coast
    else: 
        Rover.throttle = 0
    Rover.brake = 0
    
    return Rover













# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

