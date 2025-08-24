from controller import Robot
import csv
import os

robot = Robot()
timestep = int(robot.getBasicTimeStep())

force_plates = [robot.getDevice(f'force_plate_{i}') for i in range(1, 5)]
for plate in force_plates:
    plate.enable(timestep)

columns = [f'{axis}_{i}' for i in range(1, 5) for axis in ['Fx', 'Fy', 'Fz']]

for label in ['O1', 'O2', 'O3', 'O4', 'Trigger']:
    columns.extend([f'{label}_x', f'{label}_y', f'{label}_z'])
    
columns.extend(['vicon_pos_x', 'vicon_pos_z', 'vicon_vel_x', 'vicon_vel_z', 'vicon_roll', 'vicon_pitch'])

with open(os.path.join(os.getenv('HOME'), 'corgi_ws/corgi_ros_ws/output_data/sim_force_plate.csv'), 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(columns)
    
    while robot.step(timestep) != -1:
        row = [value for plate in force_plates for value in plate.getValues()[:3]]
        
        for i in range(len(force_plates)):
            row[i*3+2] -= 9.81
            row[i*3+2] *= -1
        
        writer.writerow(row)
