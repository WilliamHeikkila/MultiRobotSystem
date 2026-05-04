import cv2
import numpy as np

class video():
    """
    A class for generating video from simulation.
    Utilizing the OpenCV2 library cv2.VideoWriter
    For now it is defaulting to MJPG format.
    """

    def __init__(self, fig, fname, frame_rate):
        # fname = fpath + 'VideoWriter' + '.avi'

        # create OpenCV video writer
        self.fig = fig
        fourcc = cv2.VideoWriter_fourcc('M','J','P','G') 

        self.writer = cv2.VideoWriter(fname, fourcc, frame_rate, self.fig.canvas.get_width_height())


    def save_image(self):
        self.fig.canvas.draw()
        # put pixel buffer in numpy array
        mat = np.array(self.fig.canvas.renderer._renderer)
        mat = cv2.resize(mat, self.fig.canvas.get_width_height())
        mat = cv2.cvtColor(mat, cv2.COLOR_RGBA2BGR)
        # write frame to video
        self.writer.write(mat)


    def close_editor(self):
        # close video writer
        self.writer.release()
        cv2.destroyAllWindows()