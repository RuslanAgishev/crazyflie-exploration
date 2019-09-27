# coding: utf-8

import numpy as np
import pandas as pd
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2, PointField
from rospy_tutorials.msg import Floats
from tf.transformations import quaternion_from_euler

def file_read(filename):
    """
    Reading LIDAR laser beams (x,y and z global coordinates of the scans)
    """
    points = np.array( pd.read_csv(filename) )

    poses = points[:,:3]
    measurements = points[:,3:]

    return poses, measurements

def preprocessing(poses, scans):
    # remove points without movement
    # poses = poses[500:3200, :]
    # scans = scans[500:3200, :]

    # remove nan-values from scans array
    scans_global = []
    for i in range(poses.shape[0]):
        scan_x = np.array( [scans[i,0],
                            scans[i,3],
                            scans[i,6],
                            scans[i,9]
                            ] )
        scan_y = np.array( [scans[i,1],
                            scans[i,4],
                            scans[i,7],
                            scans[i,10]
                            ] )
        # remove nans:
        scan_x = [x for x in scan_x if str(x) != 'nan']
        scan_y = [y for y in scan_y if str(y) != 'nan']
        
        a = np.array(scan_x)[:,np.newaxis]
        b = np.array(scan_y)[:,np.newaxis]
        
        scans_global.append(np.hstack([a,b]))
    return poses, scans_global


def msg_def_PoseStamped(pose, orient):
        worldFrame = "base_link"
        msg = PoseStamped()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = worldFrame
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        orient = np.array(orient) / 180*np.pi
        quaternion = quaternion_from_euler(orient[0], orient[1], orient[2]) #1.57
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        msg.header.seq += 1
        return msg
def publish_pose(pose, orient=[0,0,0,1], topic_name='pose'):
    msg = msg_def_PoseStamped(pose, orient)
    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
    pub.publish(msg)

def xyz_array_to_pointcloud2(points, limit):
    '''
    Create a sensor_msgs.PointCloud2 from an array of points.
    '''
    if limit>0: points = points[-limit:]
    msg = PointCloud2()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()
    return msg 
def publish_pointcloud(points, topic_name='pointcloud', limit=-1):
    msg = xyz_array_to_pointcloud2(points, limit)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=4)
    pub.publish(msg)

def publish_scan(scan, topic_name='scans'):
    scan = np.array(scan)
    scan_to_publish = scan.flatten()
    pub = rospy.Publisher(topic_name, numpy_msg(Floats),queue_size=10)
    scan_to_publish = np.array(scan_to_publish, dtype=np.float32)
    pub.publish(scan_to_publish)

def publish_pose_and_scan(pose, scan, topic_name='pose_scans'):
    scan_flattened = np.array(scan).flatten()
    data_to_publish = np.hstack([pose, scan_flattened]).flatten()
    pub = rospy.Publisher(topic_name, numpy_msg(Floats),queue_size=10)
    data_to_publish = np.array(data_to_publish, dtype=np.float32)
    pub.publish(data_to_publish)


def main():
    if len(sys.argv)>1:
        filename = sys.argv[1]
    else:
        filename = 'csvs/coordsXYZ1567005444.69.csv'

    poses_raw, scans_raw = file_read(filename)
    poses, scans_global = preprocessing(poses_raw, scans_raw)

    rate = rospy.Rate(50)
    i = 0
    while not rospy.is_shutdown() and i<len(poses):
        pose = poses[i][:2]
        scan = scans_global[i]

        # publish_pose([pose[0], pose[1], 0], topic_name='robot_pose')
        # publish_pointcloud(scan, topic_name='scans')
        # publish_scan(scan, topic_name='scans')
        publish_pose_and_scan(pose, scan, 'pose_scans')
        i += 1
        rate.sleep()
        


if __name__=='__main__':
    rospy.init_node('perception_data')
    main()