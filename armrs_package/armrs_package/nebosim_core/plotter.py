import numpy as np
import matplotlib.pyplot as plt


class draw2DPointSI():
    """
    A class for plotting for multi-robots with single integrator (kinematic) model
    in 2D field (x,y plane).
    """

    def __init__(self, ax, *, field_x=None, field_y=None, pos_trail_nums=0):
        # The state should be n rows and 3 dimensional column (pos.X, pos.Y, and theta)
        # pos_trail_nums determine the number of past data to plot as trajectory trails
        self._ax = ax
        self._ax.set(xlabel="x [m]", ylabel="y [m]")
        self._ax.set_aspect('equal', adjustable='box', anchor='C')
        # Set field
        if field_x is not None:
            self._ax.set(xlim=(field_x[0] - 0.1, field_x[1] + 0.1))
        if field_y is not None:
            self._ax.set(ylim=(field_y[0] - 0.1, field_y[1] + 0.1))

        # plot placeholder for the position
        self._pl_pos = {}

        self._trail_num = pos_trail_nums
        if pos_trail_nums > 0:
            # Prepare buffer for the trail
            self._pl_trail = {}
            self._trail_data = {}

        # Plotting variables
        self._colorList = plt.rcParams['axes.prop_cycle'].by_key()['color']
        self._drawn_time = None

    def plot_robot_pos(self, key_id, pos):
        # Update data on existing plot
        if key_id in self._pl_pos:
            self._pl_pos[key_id].set_data([pos[0]], [pos[1]])

        # Initiate plot the first time
        else:
            # Adjust color
            color_id = key_id % (len(self._colorList))
            # Draw first position
            self._pl_pos[key_id], = self._ax.plot(pos[0], pos[1],
                                                  color=self._colorList[color_id],
                                                  marker='X', markersize=10)

        # Update the trail data
        self.update_trail(key_id, pos)


    def update_trail(self, key_id, pos):
        # Update data on existing plot
        if self._trail_num > 0:  # update trail data
            if key_id in self._trail_data:
                # roll the data, fill the new one from the top and then update plot
                self._trail_data[key_id] = np.roll(self._trail_data[key_id], self._trail_data[key_id].shape[1])
                self._trail_data[key_id][0, :] = pos
                self._pl_trail[key_id].set_data(self._trail_data[key_id][:, 0], self._trail_data[key_id][:, 1])

            else:
                # Adjust color
                color_id = key_id % (len(self._colorList))
                # use initial position to populate all matrix (pos_trail_nums-row x dim-col)
                # theta is not used for plotting the trail
                self._trail_data[key_id] = np.tile(pos, (self._trail_num, 1)).astype(float)

                # Plot the first trail data
                self._pl_trail[key_id], = self._ax.plot(
                    self._trail_data[key_id][:, 0], self._trail_data[key_id][:, 1],
                    '--', color=self._colorList[color_id])


    def plot_time(self, time):
        if self._drawn_time is None:
            # Display simulation time
            self._drawn_time = self._ax.text(0.78, 0.99,
                                             't = ' + f"{time:.2f}" + ' s', color='k', fontsize='large',
                                             horizontalalignment='left', verticalalignment='top',
                                             transform=self._ax.transAxes)
        else:
            self._drawn_time.set_text('t = ' + f"{time:.2f}" + ' s')



class draw2DUnicyle(draw2DPointSI):

    def __init__(self, ax, *, field_x=None, field_y=None, pos_trail_nums=0):
        super().__init__(ax, 
                         field_x=field_x, field_y=field_y, pos_trail_nums=pos_trail_nums)

    def plot_robot_pos(self, key_id, pos, theta):
        # Adjust color
        color_id = key_id % (len(self._colorList))
        self.__draw_icon( key_id, pos, theta, arrow_col=self._colorList[color_id])
        # Update the trail data
        self.update_trail(key_id, pos)

    def __draw_icon(self, key_id, pos, theta, arrow_col = 'b'): # draw mobile robot as an icon
        # Extract data for plotting
        px, py, th = pos[0], pos[1], theta
        # Basic size parameter
        scale = 1
        body_rad = 0.08 * scale # m
        wheel_size = [0.1*scale, 0.02*scale] 
        arrow_size = body_rad
        # left and right wheels anchor position (bottom-left of rectangle)
        thWh = [th+0., th+np.pi] # unicycle
        # thWh = [ (th + i*(2*np.pi/3) - np.pi/2) for i in range(3)] # for omnidirectional icon
        thWh_deg = [np.rad2deg(i) for i in thWh]
        wh_x = [ px - body_rad*np.sin(i) - (wheel_size[0]/2)*np.cos(i) + (wheel_size[1]/2)*np.sin(i) for i in thWh ]
        wh_y = [ py + body_rad*np.cos(i) - (wheel_size[0]/2)*np.sin(i) - (wheel_size[1]/2)*np.cos(i) for i in thWh ]
        # Arrow orientation anchor position
        ar_st= [px, py] #[ px - (arrow_size/2)*np.cos(th), py - (arrow_size/2)*np.sin(th) ]
        ar_d = (arrow_size*np.cos(th), arrow_size*np.sin(th))
        # initialized unicycle icon at the center with theta = 0
        if key_id not in self._pl_pos: # first time drawing
            self._pl_pos[key_id] = [None]*(2+len(thWh))
            self._pl_pos[key_id][0] = self._ax.add_patch( plt.Circle( (px, py), body_rad, color='#AAAAAAAA') )
            self._pl_pos[key_id][1] = self._ax.quiver( ar_st[0], ar_st[1], ar_d[0], ar_d[1], 
                scale_units='xy', scale=1, color=arrow_col, width=0.1*arrow_size)
            for i in range( len(thWh) ):
                self._pl_pos[key_id][2+i] = self._ax.add_patch( plt.Rectangle( (wh_x[i], wh_y[i]), 
                    wheel_size[0], wheel_size[1], angle=thWh_deg[i], color='k') )
        else: # update existing patch
            self._pl_pos[key_id][0].set( center=(px, py) )
            self._pl_pos[key_id][1].set_offsets( ar_st )
            self._pl_pos[key_id][1].set_UVC( ar_d[0], ar_d[1] )
            for i in range( len(thWh) ):
                self._pl_pos[key_id][2+i].set( xy=(wh_x[i], wh_y[i]) )
                self._pl_pos[key_id][2+i].angle = thWh_deg[i]