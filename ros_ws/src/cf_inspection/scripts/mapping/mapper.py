#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization. 

Subscribed topics:
/scan

Published topics:
/map 
/map_metadata

Author: Nathan Sprague
Version: 2/13/14
"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from rospy_tutorials.msg import Floats

import numpy as np
from PIL import Image, ImageDraw


class Map(object):
    """ 
    The Map class stores an occupancy grid as a two dimensional
    numpy array. 
    
    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    
    Note that x increases with increasing column number and y increases
    with increasing row number. 
    """

    def __init__(self, origin_x, origin_y, resolution, width, height):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
                                
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width 
        self.height = height
        self.logodds_grid = np.zeros((int(height/resolution), int(width/resolution)))
        self.grid = np.zeros((int(height/resolution), int(width/resolution)))

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width / self.resolution
        grid_msg.info.height = self.height / self.resolution
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method!  Feel free to throw out
        point that land outside of the grid. 
        """
        pass

    def to_logodds_matrix(self, pose, scans_global, prob=0.01):
        """Converts list of XY points to 2D array map in which each pixel denotes
        probability of pixel being occupied.

        Parameters
        ----------
        pose : ndarray
            XY coordinates of the robot in the map reference frame
        scans_global : ndarray
            List of XY points measured by sensor in the map reference frame
        prob : float
            Probability

        Returns
        -------
        map : ndarray
            2D array representing map with dtype numpy.float32
        """
        map_size = (int(self.width / self.resolution), int(self.height / self.resolution) )
        zero = (pose//self.resolution).astype(np.int32)
        pixels = (scans_global//self.resolution).astype(np.int32)
        mask = (pixels[:, 0] >= 0) & (pixels[:, 0] < map_size[0]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < map_size[1])
        pixels = pixels[mask]
        img = Image.new('L', (map_size[1], map_size[0]))
        draw = ImageDraw.Draw(img)
        zero = (zero[1], zero[0])
        for p in set([(q[1], q[0]) for q in pixels]):
            draw.line([zero, p], fill=1)
        data = -np.fromstring(img.tobytes(), np.int8).reshape(map_size)
        data[pixels[:, 0], pixels[:, 1]] = 1

        return 0.5 + prob*data.astype(np.float32)


class Mapper(object):
    """ 
    The Mapper class creates a map from laser scan data.
    """
    def __init__(self):
        """ Start the mapper. """
        rospy.init_node('mapper')
        self._map = Map(origin_x=2.0, origin_y=2.0, resolution=0.04, 
                 width=4.0, height=4.0)

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        rospy.Subscriber('pose_scans',
                         Floats, self.scan_callback, queue_size=1)

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it. 
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=1)
        self._map_data_pub = rospy.Publisher('map_metadata', 
                                             MapMetaData, latch=True, queue_size=1)

        rospy.spin()

    def scan_callback(self, msg):
        """ Update the map on every scan callback. """
        # Fill some cells in the map just so we can see that something is 
        # being published.
        LOWEST_X = self._map.origin_x - self._map.width
        LOWEST_Y = self._map.origin_y - self._map.height
        pose = np.array(msg.data[:2]) - np.array([LOWEST_X, LOWEST_Y])
        scans = np.array(msg.data[2:]).reshape(len(msg.data[2:])//2, 2) - np.array([LOWEST_X, LOWEST_Y])
        grid = self._map.to_logodds_matrix(pose[:2], scans, prob=0.02 )
        # Converting of occupancy grid to log-odds representation
        l = np.log(grid/(1-grid))
        self._map.logodds_grid += l

        # # Converting from the log-odds representation to the probabilities grid
        self._map.grid = 1 / (1 + np.exp(-self._map.logodds_grid))

        # Now that the map is updated, publish it!
        # rospy.loginfo("Scan is processed, publishing updated map.")
        self.publish_map()

    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass
