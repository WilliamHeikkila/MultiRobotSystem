from .nebosim_core.plotter import draw2DUnicyle
from .nebosim_core.logger import dataLogger
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np

# Library for voronoi-related plotting
import matplotlib.patches as patches
from mpl_toolkits.axes_grid1.inset_locator import inset_axes


class PlotVisualizer():
    def __init__(self, param_dict, scenario_dict):

        # PARAMETER ON VISUALIZATION
        self.SHOW_LIDAR_DETECTION = True
        self.SHOW_COMMUNICATION = True
        self.SHOW_ROBOT_GOAL = True
        self.SHOW_ROBOT_ID = True
        self.SHOW_ROBOT_SI_VELOCITY = True

        # TIME SERIES DATA
        self.SHOW_ROBOT_POS = True
        self.SHOW_LAHEAD_POS = True

        # Only allow one (describe priority over the other)
        if self.SHOW_LAHEAD_POS: self.SHOW_ROBOT_POS = False

        self.list_ID = scenario_dict.list_robot_ID
        self.Tmax = param_dict.Tmax
        self.Ts = param_dict.Ts

        # self.time_series_window = param_dict.Tmax # in second
        self.time_series_window = 5. # in seconds
        self.t_range = (-0.1, self.time_series_window)

        self.tseries_data_num = round(self.time_series_window/param_dict.Ts)
        self.t_idx = 0
        self.array_time = [None] * self.tseries_data_num

        # For now plot 2D with 2x2 grid space, to allow additional plot later on
        rowNum, colNum = 2, 3
        self.fig = plt.figure(figsize=(4 * colNum, 3 * rowNum), dpi=100)
        gs = GridSpec(rowNum, colNum, figure=self.fig)

        trail_nums = 500 # trail of trajectory plot
        # Initiate the canvas for animating the robot movement
        self.ax_2D = self.fig.add_subplot(gs[0:2, 0:2])  # Always on
        self.canvas = draw2DUnicyle( self.ax_2D, 
            field_x = param_dict.field_x, field_y = param_dict.field_y,
            pos_trail_nums = trail_nums)

        # Save list of robots within each formation
        self.form_ids, self.form_A, self.form_tol = {}, {}, {}
        for f_id in scenario_dict.form_param:
            self.form_ids[f_id] = scenario_dict.form_param[f_id]['ids']
            self.form_A[f_id] = scenario_dict.form_param[f_id]['adj_mat']

        self.pl_sens = {} # for self.SHOW_LIDAR_DETECTION
        self.pl_goal = {} # for self.SHOW_ROBOT_GOAL
        self.pl_comm = {} # for self.SHOW_COMMUNICATION
        self._txt_robotID = {} # for self.SHOW_ROBOT_ID
        self.pl_vel = {} # self.SHOW_ROBOT_SI_VELOCITY

        self.pl_ropx, self.array_pos_x = {}, {} # for self.SHOW_ROBOT_POS
        self.pl_ropy, self.array_pos_y = {}, {} # for self.SHOW_ROBOT_POS
        self.pl_lapx, self.array_lahead_x = {}, {} # for self.SHOW_LAHEAD_POS
        self.pl_lapy, self.array_lahead_y = {}, {} # for self.SHOW_LAHEAD_POS

        self.patch_voronoi = {}
        self.pl_dens = None

        self.tseries_colorList = plt.rcParams['axes.prop_cycle'].by_key()['color']

        if self.SHOW_ROBOT_POS:
            # Plot the center point for all robots
            self.ax_ropx = self.fig.add_subplot(gs[0, 2])
            self.ax_ropx.set(xlabel="t [s]", ylabel="robot pos-X [m]")
            self.ax_ropy = self.fig.add_subplot(gs[1, 2])
            self.ax_ropy.set(xlabel="t [s]", ylabel="robot pos-Y [m]")

        if self.SHOW_LAHEAD_POS:
            # Plot the Look-ahead points for all robots
            self.ax_lapx = self.fig.add_subplot(gs[0, 2])
            self.ax_lapx.set(xlabel="t [s]", ylabel="Look-ahead pos-X [m]")
            self.ax_lapy = self.fig.add_subplot(gs[1, 2])
            self.ax_lapy.set(xlabel="t [s]", ylabel="Look-ahead pos-Y [m]")


    def update(self, time, robot_est_dict, cent_evaluator):

        for i in robot_est_dict:            
            if robot_est_dict[i].pos is not None:
                self.canvas.plot_robot_pos(i, robot_est_dict[i].pos, robot_est_dict[i].theta)
    
                if self.SHOW_ROBOT_ID:
                    shift = np.array([0.2, 0.2, 0.]) # ad shift to make it visible
                    pos = robot_est_dict[i].pos + shift
                    try: # update existing plot
                        self._txt_robotID[i].set_position( (pos[0], pos[1]) )
                    except: # Initiate the first time
                        self._txt_robotID[i] = self.ax_2D.text( pos[0], pos[1], 
                            str(i), weight='bold', color=self.canvas._colorList[i],
                            horizontalalignment='center', verticalalignment='center')
            
                if self.SHOW_ROBOT_SI_VELOCITY and (robot_est_dict[i].lahead_pos is not None):
                    vector_upscale = 10.
                    pos_fr = robot_est_dict[i].lahead_pos
                    vec = vector_upscale * robot_est_dict[i].vel_command
                    try: # update existing plot
                        self.pl_vel[i].set_offsets( [pos_fr[0], pos_fr[1]] )
                        self.pl_vel[i].set_UVC( vec[0], vec[1] )
                    except: # Initiate the first time - Draw a new arrow
                        head_scale = 2
                        self.pl_vel[i] = self.ax_2D.quiver( pos_fr[0], pos_fr[1], vec[0], vec[1], 
                            scale_units='xy', scale=1.0, color='k', width=0.01, alpha=0.3,
                            headwidth = head_scale, headlength = head_scale, 
                            headaxislength=head_scale)


            sensed_pos = robot_est_dict[i].obs_pos
            if self.SHOW_LIDAR_DETECTION and sensed_pos is not None:
                try: # update existing plot
                    self.pl_sens[i].set_data(sensed_pos[:, 0], sensed_pos[:, 1])
                except: # Initiate the first time
                    self.pl_sens[i], = self.ax_2D.plot(sensed_pos[:, 0], sensed_pos[:, 1], '.', color=self.canvas._colorList[i])

            if self.SHOW_ROBOT_GOAL:
                goal_pos = robot_est_dict[i].goal
                try: # update existing plot
                    self.pl_goal[i].set_data([goal_pos[0]], [goal_pos[1]])
                except: # Initiate the first time
                    self.pl_goal[i], = self.ax_2D.plot([goal_pos[0]], [goal_pos[1]], '.', color=self.canvas._colorList[i])

        if self.SHOW_COMMUNICATION:
            self.plot_bidirectional_communication(robot_est_dict)


        # Setup for time series data with moving window horizon
        self.set_up_time(time)

        if self.SHOW_ROBOT_POS:
            self.plot_robot_pos(robot_est_dict)

        if self.SHOW_LAHEAD_POS:
            self.plot_lahead_pos(robot_est_dict)

        self.draw(time)


    def set_up_time(self, time):
        # Setup for time series data with moving window horizon
        if (self.t_idx >= 0): 
            self.t_idx += 1 # data less than full window horizon
            if (self.t_idx >= self.tseries_data_num): 
                self.t_idx = -1 # data longer than horizon, need to roll
            else:
                self.array_time[self.t_idx] = time

        if self.t_idx == -1: 
            self.array_time.pop(0) # remove first element
            self.array_time.append(time) # add new data at the end
            self.t_range = (time - (self.time_series_window), time + 0.1)


    def draw(self, time):
        self.canvas.plot_time(time)
        plt.pause(0.000001) # The pause is needed to show the plot
        # plt.pause(0.5) # Slower pause to observe the movement


    def plot_bidirectional_communication(self, robot_est_dict):
        # Plot for communication graph
        for f_id in self.form_ids:
            ids = self.form_ids[f_id]
            for i in range(len(ids)):
                for j in range(len(ids)):
                    if (i < j) and (self.form_A[f_id][i,j] > 0.):
                        pos_i = robot_est_dict[ids[i]].lahead_pos
                        pos_j = robot_est_dict[ids[j]].lahead_pos
                        try: # update existing plot
                            self.pl_comm[f'{ids[i]}_{ids[j]}'].set_data(
                                [pos_i[0], pos_j[0]], [pos_i[1], pos_j[1]])
                        except: # initiate the first time
                            if (pos_i is not None) and (pos_j is not None):
                                self.pl_comm[f'{ids[i]}_{ids[j]}'], = \
                                    self.ax_2D.plot([pos_i[0], pos_j[0]], [pos_i[1], pos_j[1]], 
                                                    color='k', linewidth=0.5)


    def plot_robot_pos(self, robot_est_dict):
        # Plot time series of robot pos in X and Y
        for id in self.list_ID:
            if robot_est_dict[id].pos is not None:
                pos_x = robot_est_dict[id].pos[0]
                pos_y = robot_est_dict[id].pos[1]

                try: # update existing array and plot
                    if self.t_idx == -1: 
                        self.array_pos_x[id].pop(0) # remove first element
                        self.array_pos_y[id].pop(0) # remove first element
                        self.array_pos_x[id].append(pos_x) # add new data at the end
                        self.array_pos_y[id].append(pos_y) # add new data at the end
                    else:
                        self.array_pos_x[id][self.t_idx] = pos_x
                        self.array_pos_y[id][self.t_idx] = pos_y

                    self.pl_ropx[id].set_data(self.array_time, self.array_pos_x[id])
                    self.pl_ropy[id].set_data(self.array_time, self.array_pos_y[id])

                except: # initiate the first time
                    self.array_pos_x[id] = [None] * self.tseries_data_num
                    self.array_pos_y[id] = [None] * self.tseries_data_num
                    self.array_pos_x[id][self.t_idx] = pos_x
                    self.array_pos_y[id][self.t_idx] = pos_y
                    
                    self.pl_ropx[id], = self.ax_ropx.plot(
                        self.array_time, self.array_pos_x[id],
                        color=self.canvas._colorList[id], label=f'{id}' )
                    self.pl_ropy[id], = self.ax_ropy.plot(
                        self.array_time, self.array_pos_y[id],
                        color=self.canvas._colorList[id], label=f'{id}' )
                    # update legend
                    self.ax_ropx.legend(loc=(0.05, 0.68), prop={'size': 6})
                    self.ax_ropy.legend(loc=(0.05, 0.68), prop={'size': 6})
                    self.ax_ropx.grid(True)
                    self.ax_ropy.grid(True)

        # Move the time-series window
        self.ax_ropx.set(ylim=(-1.1, 7.1), xlim=self.t_range)
        self.ax_ropy.set(ylim=(-1.1, 7.1), xlim=self.t_range)


    def plot_lahead_pos(self, robot_est_dict):
        # Plot time series of robot pos in X and Y
        for id in self.list_ID:
            if robot_est_dict[id].lahead_pos is not None:
                lahead_pos_x = robot_est_dict[id].lahead_pos[0]
                lahead_pos_y = robot_est_dict[id].lahead_pos[1]

                try: # update existing array and plot
                    if self.t_idx == -1: 
                        self.array_lahead_x[id].pop(0) # remove first element
                        self.array_lahead_y[id].pop(0) # remove first element
                        self.array_lahead_x[id].append(lahead_pos_x) # add new data at the end
                        self.array_lahead_y[id].append(lahead_pos_y) # add new data at the end
                    else:
                        self.array_lahead_x[id][self.t_idx] = lahead_pos_x
                        self.array_lahead_y[id][self.t_idx] = lahead_pos_y

                    self.pl_lapx[id].set_data(self.array_time, self.array_lahead_x[id])
                    self.pl_lapy[id].set_data(self.array_time, self.array_lahead_y[id])

                except: # initiate the first time
                    self.array_lahead_x[id] = [None] * self.tseries_data_num
                    self.array_lahead_y[id] = [None] * self.tseries_data_num
                    self.array_lahead_x[id][self.t_idx] = lahead_pos_x
                    self.array_lahead_y[id][self.t_idx] = lahead_pos_y
                    
                    self.pl_lapx[id], = self.ax_lapx.plot(
                        self.array_time, self.array_lahead_x[id],
                        color=self.canvas._colorList[id], label=f'{id}' )
                    self.pl_lapy[id], = self.ax_lapy.plot(
                        self.array_time, self.array_lahead_y[id],
                        color=self.canvas._colorList[id], label=f'{id}' )
                    # update legend
                    self.ax_lapx.legend(loc=(0.05, 0.68), prop={'size': 6})
                    self.ax_lapy.legend(loc=(0.05, 0.68), prop={'size': 6})
                    self.ax_lapx.grid(True)
                    self.ax_lapy.grid(True)

        # Move the time-series window
        self.ax_lapx.set(ylim=(-1.1, 7.1), xlim=self.t_range)
        self.ax_lapy.set(ylim=(-1.1, 7.1), xlim=self.t_range)


    # ADDITIONAL PLOTTER FOR VORONOI CELL
    # -----------------------------------------------------------------------------------------
    def plot_voronoi_cell(self, id, vertices, face_fill = False):
        try: # update existing array and plot
            self.patch_voronoi[id].set_xy(vertices)

        except: # initiate the first time
            fc_col = self.canvas._colorList[id]
            if face_fill: ec_col = self.canvas._colorList[id]
            else: ec_col = None

            self.patch_voronoi[id] = patches.Polygon(vertices, 
                alpha=0.5, fc=fc_col, ec=ec_col, fill=face_fill)
            self.ax_2D.add_patch(self.patch_voronoi[id])

    def plot_density_function(self, grid_points, density_value):
        try: # update existing array and plot
            # HERE ASSUMING THE GRID_POINTS ARE NOT CHANGING
            self.pl_dens.set_array(density_val)            

        except: # initiate the first time
            self.pl_dens = self.ax_2D.tripcolor(
                grid_points[:,0], grid_points[:,1], density_value, 
                vmin = 0, vmax = 1, shading='flat')

            axins1 = inset_axes(self.ax_2D, width="25%", height="2%", loc='lower right')
            plt.colorbar(self.pl_dens, cax=axins1, orientation='horizontal', ticks=[0, 0.5, 1])
            axins1.xaxis.set_ticks_position("top")

