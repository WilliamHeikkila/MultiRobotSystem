import numpy as np

def calc_robot_circ_bounds(pos, theta, robot_rad, sides=8):
    # Approximate robot bound as polygon with a given number of equal sides
    # Putting a higher number of sides making it closer to circle but add burden for obstacle detection
    robot_angle_bound = np.append(np.linspace(0., 2 * np.pi, num=sides, endpoint=False), 0) + np.pi / 8
    # Update robot shape to be used for range detection
    v_angles = robot_angle_bound + theta
    robot_shape = np.array([np.cos(v_angles), np.sin(v_angles), v_angles * 0]) * robot_rad
    robot_bounds = np.transpose(robot_shape + pos.reshape(3, 1))
    return robot_bounds

def calc_detected_pos(range_data, pos, theta, beam_angles):
    all_detected_pos = np.zeros((len(beam_angles), 3))
    sensing_angle_rad = theta + beam_angles
    all_detected_pos[:, 0] = pos[0] + range_data * np.cos(sensing_angle_rad)
    all_detected_pos[:, 1] = pos[1] + range_data * np.sin(sensing_angle_rad)
    return all_detected_pos


class DetectObstacle2D():

    def __init__(self):
        # Store the obstacle as line segments (x1, y1, x2, y2)
        self.__y1_min_y2, self.__x1_min_x2 = {}, {}
        self.__line_segment_2D = {}

    def register_obstacle_bounded(self, id, vertices):
        # store list of vertices that construct the obstacle into self.__line_segment
        # expect the vertices to be numpy array N x 3 
        # TODO: assert that the last vertex should be the same as the first
        new_line_segment = np.zeros((vertices.shape[0]-1, 4))
        new_line_segment[:,:2] = vertices[:-1,:2]
        new_line_segment[:,2:] = vertices[1:,:2]
        # store the data
        self.__line_segment_2D[id] = new_line_segment
        # self.__line_segment_2D = np.vstack((self.__line_segment_2D, new_line_segment))
        self.__update_basic_comp(id)

    def remove_obstacle_bounded(self, id):
        del self.__line_segment_2D[id]
        del self.__y1_min_y2[id]
        del self.__x1_min_x2[id]

    def __update_basic_comp(self, id):
        self.__y1_min_y2[id] = self.__line_segment_2D[id][:,1] - self.__line_segment_2D[id][:,3]
        self.__x1_min_x2[id] = self.__line_segment_2D[id][:,0] - self.__line_segment_2D[id][:,2]

    def get_sensing_data(self, posx, posy, theta_rad, exclude=[],
                         beam_angles=np.linspace(0., 2*np.pi, num=360, endpoint=False),
                         max_distance=10, default_empty_val = None):
        # The computation of detected obstacle will rely on the intersection 
        # between sensing's line-segment and obstacle's line-segment
        # The basic computation is following https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

        # Given the obstacle's line-segment denoted by (x1, y1) and (x2, y2)
        # and the sensing's line-segment denoted by (x3, y3) and (x4, y4), with
        # u = ( (x1-x3)(y1-y2) - (y1-y3)(x1-x2) ) / ( (x1-x2)(y3-y4) - (y1-y2)(x3-x4) ),
        # the obstacle is detected at (x3 + u(x4-x3), y3 + u(y4-y3))
        # if 0 <= u <= 1
        # So with (x3, y3) as the sensor's position then 
        # u is the ratio from the maximum sensing distance
        
        # number of sensing beam is m and number of obstacle line segment is n
        m = len(beam_angles)
        sensing_angle_rad = theta_rad + beam_angles
        m_x4_min_x3 = max_distance * np.cos( sensing_angle_rad )
        m_y4_min_y3 = max_distance * np.sin( sensing_angle_rad )

        # FIltering and collecting all line segments
        line_segment_2D = np.zeros((0,4)) 
        n_y1_min_y2 = np.zeros((0,1)) 
        n_x1_min_x2 = np.zeros(0) 
        for key in self.__line_segment_2D:
            if key not in exclude:
                # TODO: some selection for the nearest obstacle
                line_segment_2D = np.vstack((line_segment_2D, self.__line_segment_2D[key]))
                n_y1_min_y2 = np.append(n_y1_min_y2, self.__y1_min_y2[key])
                n_x1_min_x2 = np.append(n_x1_min_x2, self.__x1_min_x2[key])

        # Computing the intersections
        n = line_segment_2D.shape[0]
        n_0 = np.repeat(0., n)
        n_1 = np.repeat(1., n)

        n_x1_min_x3 = line_segment_2D[:,0] - np.repeat(posx, n)
        n_y1_min_y3 = line_segment_2D[:,1] - np.repeat(posy, n)

        # Loop over each sensing direction
        u_all = np.repeat(1., m)
        for i in range(m):
            # create repmat x3 and y3 for n_obs_lseg
            n_x3_min_x4 = - np.repeat( m_x4_min_x3[i], n )
            n_y3_min_y4 = - np.repeat( m_y4_min_y3[i], n )

            t_upper = (n_x1_min_x3 * n_y3_min_y4) - (n_y1_min_y3 * n_x3_min_x4)
            u_upper = (n_x1_min_x3 * n_y1_min_y2) - (n_y1_min_y3 * n_x1_min_x2)
            lower = (n_x1_min_x2 * n_y3_min_y4) - (n_y1_min_y2 * n_x3_min_x4)
            with np.errstate(divide='ignore'):
                t = t_upper / lower
                u = u_upper / lower

            t_idx = np.logical_and( t >= n_0, t <= n_1 )
            u_idx = np.logical_and( u >= n_0, u <= n_1 )
            idx = np.logical_and( t_idx, u_idx )
            if np.any(idx): u_all[i] = min( u[idx] )

        sensing_data = max_distance * u_all
        # Assign the default value for measurement with no object
        # e.g, the common LiDAR in turtlebot assign 0 value for no-detection
        if default_empty_val is not None:
            sensing_data[sensing_data > 0.99*max_distance] = default_empty_val

        return sensing_data

        # sensing_pos = np.array([[posx, posy],]*m)
        # sensing_pos[:,0] += u_all * m_x4_min_x3
        # sensing_pos[:,1] += u_all * m_y4_min_y3

        # return sensing_data, sensing_pos
