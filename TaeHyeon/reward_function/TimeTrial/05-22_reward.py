def reward_function(params):
    ###############
    ### Imports ###
    ###############

    import math

    ########################
    ###    Constants     ###
    ########################
   
    LOOKAHEAD_DISTANCE = 1.25

    ########################
    ### Input parameters ###
    ########################

    x = params['x']
    y = params['y']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    speed = params['speed']

    ########################
    ### Calculate Reward ###
    ########################
   
    # init reward
    reward = 1e-3
    pure_pursuit_reward = 0
    speed_reward = 0
   
    # speed reward
    next_point_idx = closest_waypoints[1]
    waypoints_size = len(waypoints)
   
    x1_y1 = waypoints[next_point_idx % waypoints_size]
    x2_y2 = waypoints[(next_point_idx+1) % waypoints_size]
    x3_y3 = waypoints[(next_point_idx+2) % waypoints_size]
   
    a = math.hypot(x1_y1[0] - x2_y2[0], x1_y1[1] - x2_y2[1])
    b = math.hypot(x2_y2[0] - x3_y3[0], x2_y2[1] - x3_y3[1])
    c = math.hypot(x1_y1[0] - x3_y3[0], x1_y1[1] - x3_y3[1])
   
    cos_theta = (a**2 + b**2 - c**2) / (2*a*b)
   
    if -1 <= cos_theta <= 0:
        z = speed * (1-cos_theta)
        speed_reward = 2 * ((1 / (1 + math.e ** (-z))) - 0.5)
   
   
    # set lookahead distance
    if speed < 1:
        LOOKAHEAD_DISTANCE = speed * 1
    else:
        LOOKAHEAD_DISTANCE = 0.333 * speed + 0.666
       
       
    # pure pursuit
   
    # next_point_idx = closest_waypoints[1]
    # waypoints_size = len(waypoints)

    while True:
        next_point = waypoints[next_point_idx % waypoints_size]
        distance = math.hypot(x - next_point[0], y - next_point[1])

        if distance > LOOKAHEAD_DISTANCE:
            break
        next_point_idx += 1

    radius = math.hypot(x - next_point[0], y - next_point[1])
    pointing = [x + (radius * math.cos(heading)), y + (radius * math.sin(heading))]
    vector_delta = math.hypot(pointing[0] - next_point[0], pointing[1] - next_point[1])

    # Max distance for pointing away will be the radius * 2
    # Min distance means we are pointing directly at the next waypoint
    # We can setup a reward that is a ratio to this max.  
    if radius != 0:
        pure_pursuit_reward = (1 - (vector_delta / (radius * 2)))
           
   
    # reward add
    reward += (pure_pursuit_reward * 0.9)
    reward += (speed_reward * 0.1)

    return reward
