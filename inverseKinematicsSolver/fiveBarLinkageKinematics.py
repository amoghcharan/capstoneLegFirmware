import numpy as np
import pylab as pl
from matplotlib import *
from matplotlib import collections as mc
import matplotlib.pyplot as plt
from PIL import Image
from io import BytesIO

class fiveBarLinkageKinematics:
    def __init__(self):
        
        self.base_link = 2.0
        self.l1_length = 1.0
        self.l2_length = 3.0
        self.l3_length = 3.0
        self.l4_length = 5.0
        self.mid_length = 1.5
        
        self.x = None
        self.y = None
        self.check = []
        self.skip_point = True
        
        self.rgb = np.array([
            [255, 0, 0],    # Red
            [0, 255, 0],    # Green
            [0, 0, 255],    # Blue
            [255, 165, 0],  # Orange
            [128, 0, 128]   # Purple
        ])
        
    def forward_kinematics(self, theta_B3, theta_B1, theta_12=None):

        self.base_start = (-self.base_link/2, 0)
        self.base_end = (self.base_link/2, 0)

        self.l3_start = (-self.base_link/2, 0)
        self.l3_end = (-self.base_link/2 + self.l3_length*np.cos(theta_B3), self.l3_length*np.sin(theta_B3))

        self.l1_start = (self.base_link/2, 0)
        self.l1_end = (self.base_link/2 + self.l1_length*np.cos(theta_B1), self.l1_length*np.sin(theta_B1))
        
        if theta_12 is None:
            theta_13 = np.arctan2((self.l3_end[1]-self.l1_end[1]), (self.l3_end[0]-self.l1_end[0]))
            dist_13 = np.sqrt((self.l3_end[1]-self.l1_end[1])**2 + (self.l3_end[0]-self.l1_end[0])**2)
            
            phi_arccos = (self.l2_length**2 + dist_13**2 - self.mid_length**2)/(2*self.l2_length*dist_13)
            if abs(phi_arccos) > 1.0:
                self.skip_point = True
                # exit()
            else:
                self.skip_point = False
                phi = np.arccos(phi_arccos)
                theta_12 = theta_13 - phi

                self.l2_start = (self.l1_end[0], self.l1_end[1])
                self.l2_end = (self.l1_end[0] + self.l2_length*np.cos(theta_12), self.l1_end[1] + self.l2_length*np.sin(theta_12))
                    
                theta_34 = np.arctan2((self.l2_end[1]-self.l3_end[1]), (self.l2_end[0]-self.l3_end[0]))
                dist_32 = np.sqrt((self.l2_end[1]-self.l3_end[1])**2 + (self.l2_end[0]-self.l3_end[0])**2)

                self.l4_start = (self.l3_end[0], self.l3_end[1])
                self.l4_end = ((self.l3_end[0] + self.l4_length*np.cos(theta_34)), (self.l3_end[1] + self.l4_length*np.sin(theta_34)))
        
    def inverse_kinematics(self, x, y):

        # Make input endpoints accessible to class
        self.x = x
        self.y = y
        
        # Use Law of Cosines to find theta between base and Link 3
        side_34 = np.sqrt((-self.base_link/2 - x)**2 + y**2)        
        theta_34 = np.arctan2(y, (x + self.base_link/2))
        phi_34 = np.arccos((self.l3_length**2 + side_34**2 - self.l4_length**2)/(2*self.l3_length*side_34))
        theta_B3 = theta_34 + phi_34
        
        # Transform endpoint along Link 4 to find intersection of 2 and 4
        l3_end = [(-self.base_link/2 + self.l3_length*np.cos(theta_B3)), self.l3_length*np.sin(theta_B3)]        
        theta_32 = np.arctan2((y-l3_end[1]), (x-l3_end[0]))
        x_transform = l3_end[0] + self.mid_length*np.cos(theta_32)
        y_transform = l3_end[1] + self.mid_length*np.sin(theta_32)
        
        # Use Law of Cosines to find theta between base and Link 1
        side_12 = np.sqrt((self.base_link/2 - x_transform)**2 + y_transform**2)
        theta_12 = np.arctan2(y_transform, (self.base_link/2 - x_transform))
        phi_12 = np.arccos((self.l1_length**2 + side_12**2 - self.l2_length**2)/(2*self.l1_length*side_12))
        theta_B1 = np.pi - (theta_12 - phi_12)
        
        # Find theta between Link 1 and Link 2 
        l1_end = [(self.base_link/2 + self.l1_length*np.cos(theta_B1)), self.l1_length*np.sin(theta_B1)]        
        theta_41 = np.arctan2((y_transform-l1_end[1]), (x_transform-l1_end[0]))
        
        # Check potential IK solution for validity in FK 
        self.forward_kinematics(theta_B3, theta_B1)
        self.singularity_check()
        assert all(self.check), 'Error: Point provided is out of bounds :('

        return theta_B3, theta_B1
    
    def singularity_check(self):
        self.check = np.isclose([self.x, self.y], [self.l4_end[0], self.l4_end[1]])

    def plot_linkage(self):

        if not self.skip_point:
            base_link = [self.base_start, self.base_end]
            link_1 = [self.l1_start, self.l1_end]
            link_2 = [self.l2_start, self.l2_end]
            link_3 = [self.l3_start, self.l3_end]
            link_4 = [self.l4_start, self.l4_end]
            
            lines = [base_link, link_1, link_2, link_3, link_4]
            
            lc = mc.LineCollection(lines, colors=self.rgb/255, linewidths=2)
            fig, ax = pl.subplots()
            if len(self.check)>0:
                plt.plot(self.x, self.y, 'bo')
            ax.add_collection(lc)
            ax.set_xlim(-10, 10)
            ax.set_ylim(-0.5, 8)
            ax.margins(0.1)
            plt.show()
        else:
            print('Error: Angles provided are out of bounds :(')
            
    def position_trajectory(self, trajectory_points: np.ndarray, storeAngles=False):
        
        fig, ax = plt.subplots()
        images = []
        plt.show(block=False)
        thetasB1 = []
        thetasB3 = []
        
        m, n = trajectory_points.shape
        for i in range(n):
            x = trajectory_points[0, i]
            y = trajectory_points[1, i]
            theta_B3, theta_B1 = self.inverse_kinematics(x, y) 
            thetasB1.append(theta_B1)
            thetasB3.append(theta_B3)
            self.forward_kinematics(theta_B3, theta_B1)
            
            base_link = [self.base_start, self.base_end]
            link_1 = [self.l1_start, self.l1_end]
            link_2 = [self.l2_start, self.l2_end]
            link_3 = [self.l3_start, self.l3_end]
            link_4 = [self.l4_start, self.l4_end]
            lines = [base_link, link_1, link_2, link_3, link_4]
            lc = mc.LineCollection(lines, colors=self.rgb/255, linewidths=2)
    
            # Save the current figure to a BytesIO object
            img_stream = BytesIO()
            plt.savefig(img_stream, format='png', bbox_inches='tight')
            img_stream.seek(0)

            # Open the image using Pillow and append it to the list
            img = Image.open(img_stream)
            images.append(img)
                            
            # Clear the previous lines and scatter
            ax.cla()
            ax.set_xlim(-10, 10)
            ax.set_ylim(-8, 8)
            ax.margins(0.1)
            
            # Plot the new data
            ax.add_collection(lc)
            scatter = ax.scatter(trajectory_points[0, :], trajectory_points[1, :], color='red')
            ax.set_title('Path Trajectory for 5 Bar Leg')

            # Pause to create an animation effect
            plt.pause(0.01)
        
        images[0].save('trajectoryAnimation.gif', save_all=True, append_images=images[1:], duration=500, loop=0)
        plt.savefig('pathTrajectory.png')
        plt.show()
        
        if storeAngles:
            return thetasB1, thetasB3

    def joint_space_animation(self):
        
        angle_increment = 100
        angle_space = np.linspace(0.01, 2*np.pi, angle_increment)
        jointSpaceX = []
        jointSpaceY = [] 
             
        fig, ax = plt.subplots()
        images = []
        plt.show(block=False)
        
        # Update the plot at each timestep
        for i in range(angle_increment):
            for j in range(angle_increment):
               
                self.forward_kinematics(angle_space[i], angle_space[j])
                
                if not self.skip_point:                    
                    jointSpaceX.append(self.l4_end[0])
                    jointSpaceY.append(self.l4_end[1])
            
                    # Show linkage periodically
                    if j%4 == 0:
                        base_link = [self.base_start, self.base_end]
                        link_1 = [self.l1_start, self.l1_end]
                        link_2 = [self.l2_start, self.l2_end]
                        link_3 = [self.l3_start, self.l3_end]
                        link_4 = [self.l4_start, self.l4_end]
                        lines = [base_link, link_1, link_2, link_3, link_4]
                        lc = mc.LineCollection(lines, colors=self.rgb/255, linewidths=2)
                        
                        # Save the current figure to a BytesIO object
                        img_stream = BytesIO()
                        plt.savefig(img_stream, format='png', bbox_inches='tight')
                        img_stream.seek(0)

                        # Open the image using Pillow and append it to the list
                        img = Image.open(img_stream)
                        images.append(img)
                                        
                        # Clear the previous lines and scatter
                        ax.cla()
                        ax.set_xlim(-10, 10)
                        ax.set_ylim(-8, 8)
                        ax.margins(0.1)
                        
                        # Plot the new data
                        ax.add_collection(lc)
                        scatter = ax.scatter(jointSpaceX[::], jointSpaceY[::], color='red')
                        ax.set_title('Workspace for 5 Bar Leg')

                        # Pause to create an animation effect
                        plt.pause(0.1)

        # Keep the plot window open at the end
        images[0].save('animation_refitted.gif', save_all=True, append_images=images[1:], duration=500, loop=0)
        plt.savefig('jointSpace_new.png')
        plt.show()
                


