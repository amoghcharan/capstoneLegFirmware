from fiveBarLinkageKinematics import fiveBarLinkageKinematics
import numpy as np
import time

test = fiveBarLinkageKinematics()
test_states = ['Inverse Kinematics', 
               'Forward Kinematics', 
               'Trajectory',
               'Joint Space'
               ]

# Parametric equations for a circle centered at (h, k)
x_left = 5
x_right = 2
height = -1.5
num_points = 15
h = (x_left + x_right) / 2
k = 3
theta = np.linspace(0, np.pi, num_points)

# Generate x and y values for the semicircle
x_values = h + (x_right - x_left) / 2 * np.cos(theta)
y_values = k + height * np.sin(theta)
x_values = np.append(x_values, np.linspace(x_left, x_right, num_points), axis=0)
y_values = np.append(y_values, np.linspace(k, k, num_points), axis=0)

trajectory_path = np.array([
                    x_values, 
                    y_values
                    ])

def test_cases(test_state):
    if test_state == 'Inverse Kinematics':
        new_theta_B3, new_theta_B1 = test.inverse_kinematics(0.9, 3.0)
        print(f'Theta B1: {2*np.pi - new_theta_B1}')
        print(f'Theta B3: {np.pi - new_theta_B3}')
        
        test.plot_linkage()
        
    elif test_state == 'Forward Kinematics':
        test.forward_kinematics(1.5, 1.0)
        test.plot_linkage()
        
    elif test_state == 'Trajectory':
        thetaB1, thetaB3 = test.position_trajectory(trajectory_path, storeAngles=True)
        new_theta_B1 = [(2*np.pi - x) for x in thetaB1]
        new_theta_B3 = [(np.pi - x) for x in thetaB3]
        
        with np.printoptions(precision=4, suppress=True):
            print(f'ThetaB1: {np.round(new_theta_B1, decimals=4)}')
            print()
            print(f'ThetaB3: {np.round(new_theta_B3, decimals=4)}')
        
    elif test_state == 'Joint Space':
        test.joint_space_animation()


# for i in range(len(test_states)):
#     test_state = test_states[i]
#     test_cases(test_state)
    
#     time.sleep(0.5)

test_state = test_states[2]

test_cases(test_state)

