#!/usr/bin/env python3

# Standard library imports
import os
from pathlib import Path
import rospy
import numpy as np
from typing import List

# ROS message imports
from geometry_msgs.msg import PoseWithCovarianceStamped
from model_distance_from_height.msg import ProjectedDetectionsArray
from multi_target_tracking.msg import Target, TargetArray

# Local application imports
from multi_target_tracking.trackers import JPDATracker
from multi_target_tracking.measurement import PosMeasurement
from multi_target_tracking.track import TrackStage, MNLogic
from multi_target_tracking.gaters import MahalanobisGater
from multi_target_tracking.filters import KFTrack
from multi_target_tracking.likelihoods import MahalanobisLikelihood

class JPDANode:
    """ROS node that uses a JPDA tracker to track targets with a single drone."""
    
    def __init__(self):
        """Initialize the ROS node for JPDA tracking."""
        rospy.init_node('JPDA_node')

        self.initialize_subscribers()
        self.initialize_publishers()
        
        rospy.loginfo(f"Initializing multi target tracker with robot ID: {self.robot_id}")
        
        # Initialize the tracker
        gater = MahalanobisGater()
        likelihood = MahalanobisLikelihood()
        filter = KFTrack()
        mn_logic = MNLogic()

        # drone ID is set to None since an offset calculation that is only based on its own measurements does not make sense
        self.tracker = JPDATracker(
            filter_obj=filter,
            gater=gater,
            likelihood=likelihood,
            track_logic=mn_logic,
            drone_id=None, 
        )
        rospy.loginfo("JPDA node initialized successfully")

    def initialize_subscribers(self):
        """Initialize ROS subscribers for pose and measurement topics."""
        
        # Get robot and system configuration
        self.robot_id = rospy.get_param('~robotID', 0)
        self.num_robots = rospy.get_param('~numRobots', 0)
        
        # Measurement subscribers
        measurement_suffix = rospy.get_param('~projected_objects_array_topic', '/nonono')
        
        # Other robots' measurement subscribers
        self.subs = []
        topic = f"/machine_{self.robot_id}/{measurement_suffix}"
        sub = rospy.Subscriber(
            topic,
            ProjectedDetectionsArray,  # Adjust message type as needed
            self.detection_callback,
            callback_args=self.robot_id,
            queue_size=10
        )
        self.subs.append(sub)
        rospy.loginfo(f"Registered detection subscriber for topic: {topic} for robot {self.robot_id}")

    def initialize_publishers(self):
        """Initialize ROS publishers for tracking results."""
        # Create publisher for tracked targets
        self.target_array_pub = rospy.Publisher(
            f"/machine_{self.robot_id}/target_tracker/MTT/target_array",
            TargetArray,
            queue_size=10
        )
        self.target_pub = rospy.Publisher(
            f"/machine_{self.robot_id}/target_tracker/MTT/target",
            Target,
            queue_size=10
        )
        rospy.loginfo(f"Publisher for tracked targets initialized for robot {self.robot_id}")
        
        # Publisher for offset (if needed)
        self.offset_pub = rospy.Publisher(
            f"/machine_{self.robot_id}/MTT/offset",
            PoseWithCovarianceStamped,
            queue_size=10
        )

    def detection_callback(self, msg, drone_id: int):
        """Process incoming detection messages from a drone.
        
        Args:
            msg: The projected detections message
            drone_id: ID of the drone that sent the message
        """
        rospy.logdebug(f"Received {len(msg.detections)} detections from drone {drone_id}")
        
        # Convert message to PosMeasurement objects
        timestamp, measurements = self.convert_to_measurements(msg)
        self.tracker.evaluate((timestamp, measurements, drone_id))

        # publish the results
        self.publish_tracks()
        # self.publish_offset() # publishing the offset only makes sense for distributed MTTs

    def convert_to_measurements(self, msg: ProjectedDetectionsArray) -> List[PosMeasurement]:
        """Convert ROS message to PosMeasurement objects.
        
        Args:
            msg: The detection message
            
        Returns:
            List of PosMeasurement objects
        """
        measurements = []
        timestamp = msg.header.stamp.to_sec()
        
        for detection in msg.detections:
            try:
                # Extract position
                pos = detection.pose.pose.position
                position = np.array([pos.x, pos.y, pos.z]).reshape(-1, 1)
                
                # Extract covariance matrix (position part only)
                covariance_flat = detection.pose.covariance
                covariance = np.zeros((3, 3))
                for i in range(3):
                    for j in range(3):
                        covariance[i, j] = covariance_flat[i*6 + j]
                
                # Create measurement object
                measurement = PosMeasurement(
                    position=position,
                    covariance=covariance
                )
                measurements.append(measurement)
            except Exception as e:
                rospy.logerr(f"Error processing detection: {e}")
        measurements = np.array(measurements)
        
        return (timestamp, measurements)

    def publish_tracks(self):
        """Publish confirmed tracks as ROS messages.
        
        Args:
            tracks: List of Track objects from the tracker
        """
        tracks = self.tracker.tracks
        # Create message
        msg = TargetArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        
        # Filter for confirmed tracks only
        publish_tracks = [t for t in tracks if t.stage == TrackStage.CONFIRMED]
        # if not publish_tracks: # fall back to tentative tracks if no confirmed tracks
        #     publish_tracks = [t for t in tracks if t.stage == TrackStage.TENTATIVE]

        for track in publish_tracks:
            target = Target()
            target.header.stamp = rospy.Time.from_sec(track.timestamp)
            target.header.frame_id = "world"
            target.track_id = track.id
            target.timestamp = track.timestamp
            
            # Position
            target.position.x = float(track.state[0])
            target.position.y = float(track.state[1])
            target.position.z = float(track.state[2])
            
            # Velocity
            target.velocity.x = float(track.state[3])
            target.velocity.y = float(track.state[4])
            target.velocity.z = float(track.state[5])

            # Covariance
            target.covariance = track.covariance.flatten().tolist()

            # publish single target messages and add to array
            self.target_pub.publish(target)       
            msg.targets.append(target)
        
        # Publish
        self.target_array_pub.publish(msg)
        if publish_tracks:
            rospy.logdebug(f"Published {len(publish_tracks)} tracked targets")

    def publish_offset(self):
        offset = self.tracker.offset_filter.state
        covariance = self.tracker.offset_filter.cov
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.pose.position.x = float(offset[0])
        msg.pose.pose.position.y = float(offset[1])
        msg.pose.pose.position.z = float(offset[2])
        covariance = np.bmat([
            [covariance[0:3, 0:3], np.zeros((3, 3))],
            [np.zeros((3, 3)), np.zeros((3, 3))]
        ])
        msg.pose.covariance = covariance.flatten().tolist()
        self.offset_pub.publish(msg)


if __name__ == '__main__':
    try:
        node = JPDANode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("JPDA node terminated")
