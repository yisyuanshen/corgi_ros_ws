import numpy as np
import pandas as pd
import LegModel
import swing


#%% Define Functions
def find_closest_beta(target_beta, ref_beta):
    if round(target_beta, 6) < round(ref_beta, 6):
        while abs(round(target_beta, 6)-round(ref_beta, 6)) >= np.pi:
            target_beta += 2 * np.pi
            
    elif round(target_beta, 6) > round(ref_beta, 6):
        while abs(round(target_beta, 6)-round(ref_beta, 6)) >= np.pi:
            target_beta -= 2 * np.pi
            
    return target_beta


def find_smaller_closest_beta(target_beta, ref_beta):
    target_beta = find_closest_beta(target_beta, ref_beta)
    
    if target_beta > ref_beta: target_beta -= 2 * np.pi
    
    return target_beta


def find_hybrid_step(RH_beta, LH_beta, body_angle):
    step_num = 1
    max_step_num = 6
    
    min_step_length = 0.1
    max_step_length = 0.2
    
    RH_target_beta = find_smaller_closest_beta(np.deg2rad(90)-body_angle, RH_beta)
    LH_target_beta = find_smaller_closest_beta(np.deg2rad(90)-body_angle, LH_beta)
    
    for i in range(max_step_num):
        # calculate step length and check if it is valid
        if step_num % 2 == 1:
            step_length = abs(leg.radius * (RH_target_beta-RH_beta)) / step_num
            if step_length < min_step_length: RH_target_beta -= 2 * np.pi
            elif min_step_length < step_length < max_step_length: return step_num, step_length
            
        else:
            step_length = abs(leg.radius * (LH_target_beta-LH_beta)) / step_num
            if step_length < min_step_length: LH_target_beta -= 2 * np.pi
            elif min_step_length < step_length < max_step_length: return step_num, step_length

        step_num += 1

    return 0, 0

print('= = =')


#%% User Parameters
init_beta = np.deg2rad(np.random.randint(0, 360, size=4))  # rad
init_theta = np.deg2rad([17, 17, 17, 17])  # rad

last_transform_step_x = 0.17  # x-direction position of the last transfromed leg

body_vel = 0.2  # m/s
stance_height = 0.2  # m (currently, only 0.2 is available)

#%% Initialization
leg = LegModel.LegModel(sim=True)
body_length = 0.444  # m
dt = 0.001  # s

wheel_delta_beta = -body_vel/leg.radius*dt  # rad/ms

# transform to initial pose in 5 sec (optional)
traj_theta_transform = np.linspace(np.deg2rad([17, 17, 17, 17]), init_theta, 5000)  # [LF, RF, RH, LH]
traj_beta_transform = np.linspace(np.deg2rad([0, 0, 0, 0]), init_beta, 5000)

# stay for 2 sec (optional)
traj_theta_stay= np.linspace(init_theta, init_theta, 2000)
traj_beta_stay = np.linspace(init_beta, init_beta, 2000)

# intialize current theta and beta
curr_theta = init_theta.copy()
curr_beta = init_beta.copy()

# initialize trajectories lists
traj_theta = [curr_theta.copy()]
traj_beta = [curr_beta.copy()]

print(f'Initial Beta = {np.rad2deg(init_beta)}')


#%% Rotate in Wheel Mode until RF_beta = 45 deg
RF_target_beta = np.deg2rad(45)
RF_target_beta = find_smaller_closest_beta(RF_target_beta, curr_beta[1])

body_move_dist = abs(leg.radius * (RF_target_beta-curr_beta[1]))
delta_time_step = int(round(body_move_dist/body_vel, 3)/dt)

# append trajectories
for t in range(delta_time_step):
    curr_beta += wheel_delta_beta
    traj_theta.append(curr_theta.copy())
    traj_beta.append(curr_beta.copy())

print(f'Front Start Beta = {np.rad2deg(curr_beta).round(4)}')
print('= = =')


#%% Front Transform
body_angle = np.arcsin(((stance_height-leg.radius)/body_length))
RF_target_beta = -body_angle
RF_target_beta = find_smaller_closest_beta(RF_target_beta, curr_beta[1])
RF_target_theta = leg.inverse([0, -stance_height+leg.r])[0]

print(f'Body Angle = {np.rad2deg(body_angle).round(4)}')
print(f'RF Target Theta = {np.rad2deg(RF_target_theta).round(4)}')
print(f'RF Target Beta = {np.rad2deg(RF_target_beta).round(4)}')

body_move_dist = abs(leg.radius * (RF_target_beta-curr_beta[1]))
delta_time_step = int(round(body_move_dist/body_vel, 3)/dt)

RF_delta_beta = (RF_target_beta-curr_beta[1])/delta_time_step
RF_delta_theta = (RF_target_theta-curr_theta[1])/delta_time_step

# determine the step length from hybrid mode
step_num, step_length = find_hybrid_step(curr_beta[2]+wheel_delta_beta*delta_time_step,
                                         curr_beta[3]+wheel_delta_beta*delta_time_step, body_angle)

[LF_target_theta, LF_target_beta] = leg.inverse([step_length, -stance_height+leg.r])
LF_target_beta = find_smaller_closest_beta(LF_target_beta-body_angle, curr_beta[0])

LF_delta_beta = (LF_target_beta-curr_beta[0])/delta_time_step
LF_delta_theta = (LF_target_theta-curr_theta[0])/delta_time_step

for t in range(delta_time_step):
    if t < delta_time_step/3: curr_beta[0] += wheel_delta_beta
    elif t < delta_time_step*2/3: curr_beta[0] += LF_delta_beta*3 - wheel_delta_beta
    else: curr_theta[0] += LF_delta_theta * 3
    
    curr_theta[1] += RF_delta_theta
    curr_beta[1] += RF_delta_beta
    
    curr_beta[2] += wheel_delta_beta
    curr_beta[3] += wheel_delta_beta
    
    traj_theta.append(curr_theta.copy())
    traj_beta.append(curr_beta.copy())

print(f'Front End Theta = {np.rad2deg(curr_theta).round(4)}')
print(f'Front End Beta = {np.rad2deg(curr_beta).round(4)}')
leg.forward(curr_theta, curr_beta+body_angle)
print(f'Front End Height = {(leg.G[:,1]-leg.r).round(4)}')
print('= = =')


#%% Hybrid Mode
delta_time_step_each = int(round(step_length/body_vel, 3)/dt)
delta_time_step = delta_time_step_each * step_num

print(f'Step Number = {step_num} => {("LH" if step_num%2==0 else "RH")} Transform')
print(f'Step Length = {step_length.round(4)}')

# swing phase trajectory
sp = swing.SwingProfile(step_length, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -stance_height+leg.r, 0.0)
d = np.linspace(0, 1, delta_time_step_each)
curve_points = [sp.getFootendPoint(_) for _ in d]

swing_target_theta_traj = []
swing_target_beta_traj = []

hip_pos = np.array([0, 0])
for point in curve_points:
    [swing_theta, swing_beta] = leg.inverse(point-hip_pos)
    swing_target_theta_traj.append(swing_theta)
    swing_target_beta_traj.append(swing_beta-body_angle)
    hip_pos[0] += step_length / delta_time_step_each

# stance phase trajectory
stance_target_theta_traj = []
stance_target_beta_traj = []

move_vector = np.array([step_length, 0])
[stance_theta, stance_beta] = [swing_target_theta_traj[-1], swing_target_beta_traj[-1]+body_angle]
for t in range(delta_time_step_each):
    [stance_theta, stance_beta] = leg.move(stance_theta, stance_beta, move_vector/delta_time_step_each)
    stance_target_theta_traj.append(stance_theta)
    stance_target_beta_traj.append(stance_beta-body_angle)

leg.forward(stance_target_theta_traj[0], stance_target_beta_traj[0]+body_angle)
print(f'Stance Start Point = [{leg.G.round(4)[0]}, {leg.G.round(4)[1]-leg.r}]')
leg.forward(stance_target_theta_traj[-1], stance_target_beta_traj[-1]+body_angle)
print(f'Stance End Point = [{leg.G.round(4)[0]}, {leg.G.round(4)[1]-leg.r}]')
leg.forward(swing_target_theta_traj[0], swing_target_beta_traj[0]+body_angle)
print(f'Swing Start Point = [{leg.G.round(4)[0]}, {leg.G.round(4)[1]-leg.r}]')
leg.forward(swing_target_theta_traj[-1], swing_target_beta_traj[-1]+body_angle)
print(f'Swing End Point = [{leg.G.round(4)[0]}, {leg.G.round(4)[1]-leg.r}]')

for t in range(delta_time_step):
    traj_idx = t % delta_time_step_each
    if (t // delta_time_step_each) % 2 == 0:
        curr_theta[1] = swing_target_theta_traj[traj_idx]
        curr_beta[1] = find_closest_beta(swing_target_beta_traj[traj_idx], curr_beta[1])
        
        curr_theta[0] = stance_target_theta_traj[traj_idx]
        curr_beta[0] = find_closest_beta(stance_target_beta_traj[traj_idx], curr_beta[0])
        
    else:
        curr_theta[0] = swing_target_theta_traj[traj_idx]
        curr_beta[0] = find_closest_beta(swing_target_beta_traj[traj_idx], curr_beta[0])
        
        curr_theta[1] = stance_target_theta_traj[traj_idx]
        curr_beta[1] = find_closest_beta(stance_target_beta_traj[traj_idx], curr_beta[1])
    
    curr_beta[2] += wheel_delta_beta
    curr_beta[3] += wheel_delta_beta
    
    traj_theta.append(curr_theta.copy())
    traj_beta.append(curr_beta.copy())

print(f'Hybrid End Theta = {np.rad2deg(curr_theta).round(4)}')
print(f'Hybrid End Beta = {np.rad2deg(curr_beta).round(4)}')
leg.forward(curr_theta, curr_beta+body_angle)
print(f'Hybrid End Height = {(leg.G[:,1]-leg.r).round(4)}')
print('= = =')


#%% Hind Transform
# ensure the robot stability
body_move_dist = leg.radius * np.deg2rad(45)
delta_time_step = int(round(body_move_dist/body_vel, 3)/dt)

LF_target_theta_traj = []
RF_target_theta_traj = []
LF_target_beta_traj = []
RF_target_beta_traj = []

move_vector = np.array([body_move_dist, 0])
[LF_theta, LF_beta] = [curr_theta[0], curr_beta[0]+body_angle]
[RF_theta, RF_beta] = [curr_theta[1], curr_beta[1]+body_angle]
for t in range(delta_time_step):
    [LF_theta, LF_beta] = leg.move(LF_theta, LF_beta, move_vector/delta_time_step)
    [RF_theta, RF_beta] = leg.move(RF_theta, RF_beta, move_vector/delta_time_step)
    LF_target_theta_traj.append(LF_theta)
    RF_target_theta_traj.append(RF_theta)
    LF_target_beta_traj.append(LF_beta-body_angle)
    RF_target_beta_traj.append(RF_beta-body_angle)

for t in range(delta_time_step):
    curr_theta[0] = LF_target_theta_traj[t]
    curr_theta[1] = RF_target_theta_traj[t]
    curr_beta[0] = find_closest_beta(LF_target_beta_traj[t], curr_beta[0])
    curr_beta[1] = find_closest_beta(RF_target_beta_traj[t], curr_beta[1])

    curr_beta[2] += wheel_delta_beta
    curr_beta[3] += wheel_delta_beta

    traj_theta.append(curr_theta.copy())
    traj_beta.append(curr_beta.copy())
    
print(f'Hind Start Theta = {np.rad2deg(curr_theta).round(4)}')
print(f'Hind Start Beta = {np.rad2deg(curr_beta).round(4)}')
leg.forward(curr_theta, curr_beta+body_angle)
print(f'Hind Start Height = {(leg.G[:,1]-leg.r).round(4)}')
print('= = =')

# final transform
body_move_dist = leg.radius * (np.deg2rad(45) - body_angle)
delta_time_step = int(round(body_move_dist/body_vel, 3)/dt)

transform_start_beta = curr_beta[3] if step_num % 2 == 0 else curr_beta[2]
transform_start_theta = curr_theta[3] if step_num % 2 == 0 else curr_theta[2]
transform_target_beta = find_smaller_closest_beta(0, transform_start_beta)
transform_target_theta = leg.inverse([0, -stance_height+leg.r])[0]

transform_delta_beta = (transform_target_beta-transform_start_beta)/delta_time_step
transform_delta_theta = (transform_target_theta-transform_start_theta)/delta_time_step


last_start_beta = curr_beta[2] if step_num % 2 == 0 else curr_beta[3]
last_start_theta = curr_theta[2] if step_num % 2 == 0 else curr_theta[3]

[last_target_theta, last_target_beta] = leg.inverse([last_transform_step_x, -stance_height+leg.r])
last_target_beta = find_smaller_closest_beta(last_target_beta, last_start_beta)

last_delta_beta = (last_target_beta-last_start_beta)/delta_time_step
last_delta_theta = (last_target_theta-last_start_theta)/delta_time_step


leg_traj = [[],[],[],[]]
for t in range(delta_time_step):
    leg.contact_map(transform_start_theta+transform_delta_theta*t,
                    transform_start_beta+transform_delta_beta*t+body_angle)
    
    if leg.rim == 2:
        hind_body_height = abs(np.imag(leg.L_l) - leg.radius)
    elif leg.rim == 3:
        hind_body_height = abs(np.imag(leg.G) - leg.r)
    
    body_angle = np.arcsin((stance_height-hind_body_height)/body_length)
    
    leg.forward(curr_theta, curr_beta+body_angle)
    LF_target_pos = np.array([leg.G[0][0]-body_move_dist/delta_time_step, -stance_height+leg.r])
    [LF_target_theta, LF_target_beta] = leg.inverse(LF_target_pos)
    LF_target_beta = find_smaller_closest_beta(LF_target_beta-body_angle, curr_beta[0])
    
    leg.forward(curr_theta, curr_beta+body_angle)
    RF_target_pos = np.array([leg.G[1][0]-body_move_dist/delta_time_step, -stance_height+leg.r])
    [RF_target_theta, RF_target_beta] = leg.inverse(RF_target_pos)
    RF_target_beta = find_smaller_closest_beta(RF_target_beta-body_angle, curr_beta[1])
    
    curr_theta[0] = LF_target_theta
    curr_beta[0] = LF_target_beta
    
    curr_theta[1] = RF_target_theta
    curr_beta[1] = RF_target_beta
    
    if step_num % 2 == 0:
        curr_theta[3] += transform_delta_theta
        curr_beta[3] += transform_delta_beta
        
        if t < delta_time_step/3: curr_beta[2] += wheel_delta_beta
        elif t < delta_time_step*2/3: curr_beta[2] += last_delta_beta*3 - wheel_delta_beta
        else: curr_theta[2] += last_delta_theta * 3
        
    else:
        curr_theta[2] += transform_delta_theta
        curr_beta[2] += transform_delta_beta
        
        if t < delta_time_step/3: curr_beta[3] += wheel_delta_beta
        elif t < delta_time_step*2/3: curr_beta[3] += last_delta_beta*3 - wheel_delta_beta
        else: curr_theta[3] += last_delta_theta * 3

    traj_theta.append(curr_theta.copy())
    traj_beta.append(curr_beta.copy())

# check final pose
leg.forward(curr_theta, curr_beta)
print('Final Pose =')
print(f'A [{leg.G[0,0].round(4)}, {(leg.G[0,1]-leg.r).round(4)}]', end=';  ')
print(f'B [{leg.G[1,0].round(4)}, {(leg.G[1,1]-leg.r).round(4)}]')
print(f'C [{leg.G[2,0].round(4)}, {(leg.G[2,1]-leg.r).round(4)}]', end=';  ')
print(f'D [{leg.G[3,0].round(4)}, {(leg.G[3,1]-leg.r).round(4)}]')
print(f'= = =')


#%% Conbine All Trajectories
traj_theta_final = np.row_stack((traj_theta_transform, traj_theta_stay, np.array(traj_theta)))
traj_beta_final = np.row_stack((traj_beta_transform, traj_beta_stay, np.array(traj_beta)))

traj_final = np.column_stack([traj_theta_final[:, 0], -traj_beta_final[:, 0],
                              traj_theta_final[:, 1],  traj_beta_final[:, 1],
                              traj_theta_final[:, 2],  traj_beta_final[:, 2],
                              traj_theta_final[:, 3], -traj_beta_final[:, 3]])

pd.DataFrame(traj_final).to_csv('traj.csv', header=False, index=False)
