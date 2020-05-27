# 기존 코드 reward 값과 장애물 reward 값에 가중치를 둬서 더하는데 그렇게 하면
# 장애물만 앞에 없으면 어떤 행동을 하더라도 reward 값이 너무 높아서
# 기존 코드 reward에 장애물 상황에 따라 reward 값을 변경시켜줘야 하지 않을까 생각중

def reward_function(params):
    ###############
    ### Imports ###
    ###############

    import math

    ########################
    ###    Constants     ###
    ########################
    
    LOOKAHEAD_DISTANCE = 1.5

    ########################
    ### Input parameters ###
    ########################

    x = params['x']
    y = params['y']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    speed = params['speed']

    # for objects avoidance
    objects_distance = params['objects_distance']
    _, next_object_index = params['closest_objects']
    objects_left_of_center = params['objects_left_of_center']
    is_left_of_center = params['is_left_of_center']

    ########################
    ### Calculate Reward ###
    ########################
    
    # init reward
    reward = 1e-3
    pure_pursuit_reward = 0
    speed_reward = 0
    
    # pure pursuit
    
    next_point_idx = closest_waypoints[1]
    waypoints_size = len(waypoints)

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

    if vector_delta == 0:
        pure_pursuit_reward = 1
    else:
        pure_pursuit_reward = (1 - (vector_delta / (radius * 2)))
        
    ########################
    
    # avoidance reward
    # Penalize if the agent is too close to the next object
    avoidance_reward = 1.0
    
    # Distance to the next object
    distance_closest_object = objects_distance[next_object_index]
    # Decide if the agent and the next object is on the same lane
    is_same_lane = objects_left_of_center[next_object_index] == is_left_of_center

    if is_same_lane:
        if 0.5 <= distance_closest_object < 0.8: 
            avoidance_reward *= 0.5
        elif 0.3 <= distance_closest_object < 0.5:
            avoidance_reward *= 0.2
        elif distance_closest_object < 0.3:
            avoidance_reward = 0 # Likely crashed
    
    # reward add
    reward += (pure_pursuit_reward * 0.5)
    reward += (avoidance_reward * 0.5)

    return reward
