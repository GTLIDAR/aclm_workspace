#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_py as tf2
import copy
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray, Marker

class PoseCapture:
    def __init__(self):
        rospy.init_node('pose_capture_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Get prefix pairs from param server. It should be a list of dicts with 'source' and 'target' keys.
        # e.g. [{'source': 'b1z1_2/', 'target': 'captured_b1z1_2/'}, {'source': 'robot2/', 'target': 'captured_robot2/'}]
        
        # --- DEFINE YOUR PREFIX PAIRS SEQUENCE HERE ---
        # This is a list of lists. Each inner list is a group of pairs
        # that will be processed in a single service call.
        self.prefix_pairs_sequence = [
            [  # First service call
              {'source': 'b1z1_1/', 'target': 'captured_b1z1_1_0/'},
              {'source': 'b1z1_2/', 'target': 'captured_b1z1_2_0/'},
              {'source': 'cargo/',  'target': 'captured_cargo_0/'}
            ],
            [  # Second service call
                {'source': 'b1z1_1/', 'target': 'captured_b1z1_1_1/'},
                {'source': 'b1z1_2/', 'target': 'captured_b1z1_2_1/'},
                {'source': 'cargo/',  'target': 'captured_cargo_1/'}

            ],
            [  # Third service call
                {'source': 'b1z1_1/', 'target': 'captured_b1z1_1_2/'},
                {'source': 'b1z1_2/', 'target': 'captured_b1z1_2_2/'},
                {'source': 'cargo/',  'target': 'captured_cargo_2/'}
            ],
            [  # Third service call
                {'source': 'b1z1_1/', 'target': 'captured_b1z1_1_3/'},
                {'source': 'b1z1_2/', 'target': 'captured_b1z1_2_3/'},
                {'source': 'cargo/',  'target': 'captured_cargo_3/'}
            ]
            # Add more groups of pairs here as needed
        ]
        self.capture_index = 0
        # ------------------------------------

        # --- Foothold polygon capture ---
        self.latest_footholds_msg = None
        self.captured_footholds_publisher = rospy.Publisher(
            '/cen_opt/captured_footholds', MarkerArray, queue_size=10, latch=True)
        self.footholds_subscriber = rospy.Subscriber(
            '/cen_opt/predicted_footholds', MarkerArray, self.footholds_callback)
        self.all_captured_foothold_markers = []  # accumulate across captures
        # --------------------------------

        # Give the listener some time to fill the buffer
        rospy.sleep(2.0)

        # The service that triggers the capture
        self.capture_service = rospy.Service('pose_capture', Trigger, self.handle_capture_request)

        rospy.loginfo("Pose capture service is ready.")

    def footholds_callback(self, msg):
        """Store the latest predicted footholds MarkerArray."""
        self.latest_footholds_msg = msg

    def handle_capture_request(self, req):
        rospy.loginfo("Pose capture request received.")
        
        if not self.prefix_pairs_sequence:
            rospy.logerr("Prefix pairs sequence is empty.")
            return TriggerResponse(success=False, message="Prefix pairs sequence is empty.")

        try:
            # Get the current group of pairs for this capture sequence
            current_pairs_group = self.prefix_pairs_sequence[self.capture_index]
            
            # Move to the next group for the next call, and loop around if at the end
            self.capture_index = (self.capture_index + 1) % len(self.prefix_pairs_sequence)

            rospy.loginfo("Processing capture group #{}".format(self.capture_index))

            all_frames_yaml = self.tf_buffer.all_frames_as_yaml()
            if not all_frames_yaml:
                return TriggerResponse(success=False, message="TF buffer is empty.")

            import yaml
            all_frames = yaml.safe_load(all_frames_yaml)

            captured_transforms = []

            for pair in current_pairs_group:
                # Handle prefix-based capture
                if 'source' in pair and 'target' in pair:
                    source_prefix = pair.get('source')
                    target_prefix = pair.get('target')

                    if not source_prefix or not target_prefix:
                        rospy.logwarn("Invalid prefix pair found, skipping: {}".format(pair))
                        continue

                    rospy.loginfo("  Processing source prefix '{}' to target prefix '{}'".format(source_prefix, target_prefix))

                    for frame_id in all_frames.keys():
                        if frame_id.startswith(source_prefix):
                            parent_id = all_frames[frame_id]['parent']
                            
                            try:
                                transform = self.tf_buffer.lookup_transform(parent_id, frame_id, rospy.Time(0))
                                
                                new_transform = TransformStamped()
                                new_transform.header.stamp = rospy.Time.now()
                                
                                # Also check if the parent needs renaming
                                new_parent_id = parent_id
                                for p in current_pairs_group:
                                    if 'source' in p and parent_id.startswith(p.get('source')):
                                        new_parent_id = parent_id.replace(p.get('source'), p.get('target'), 1)
                                        break
                                new_transform.header.frame_id = new_parent_id

                                new_transform.child_frame_id = frame_id.replace(source_prefix, target_prefix, 1)
                                
                                new_transform.transform = transform.transform
                                
                                captured_transforms.append(new_transform)

                            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
                                rospy.logwarn("Could not get transform for frame {}: {}".format(frame_id, e))
                
                # Handle specific frame-based capture
                elif 'source_parent' in pair and 'source_child' in pair and 'target_prefix' in pair:
                    source_parent = pair.get('source_parent')
                    source_child = pair.get('source_child')
                    target_prefix = pair.get('target_prefix')

                    if not source_parent or not source_child or not target_prefix:
                        rospy.logwarn("Invalid frame pair found, skipping: {}".format(pair))
                        continue
                    
                    rospy.loginfo("  Processing source parent '{}' and child '{}' to target prefix '{}'".format(source_parent, source_child, target_prefix))

                    try:
                        transform = self.tf_buffer.lookup_transform(source_parent, source_child, rospy.Time(0))
                        
                        new_transform = TransformStamped()
                        new_transform.header.stamp = rospy.Time.now()
                        new_transform.header.frame_id = source_parent # Parent is preserved
                        new_transform.child_frame_id = target_prefix + source_child
                        new_transform.transform = transform.transform
                        
                        captured_transforms.append(new_transform)

                    except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
                        rospy.logwarn("Could not get transform for frame {} to {}: {}".format(source_parent, source_child, e))


            if not captured_transforms:
                return TriggerResponse(success=False, message="No frames found for the current capture group.")

            self.static_broadcaster.sendTransform(captured_transforms)

            # --- Capture foothold polygon visualization ---
            num_foothold_markers = 0
            if self.latest_footholds_msg is not None:
                capture_ns_prefix = "captured_{}".format(self.capture_index - 1)
                for marker in self.latest_footholds_msg.markers:
                    # Skip the DELETEALL marker and non-polygon markers
                    if marker.action == Marker.DELETEALL:
                        continue
                    if not marker.ns.startswith("constraint_polygon"):
                        continue
                    captured_marker = copy.deepcopy(marker)
                    # Make permanent (no auto-delete)
                    captured_marker.lifetime = rospy.Duration(0)
                    # Prefix namespace so multiple captures don't collide
                    captured_marker.ns = capture_ns_prefix + "/" + captured_marker.ns
                    self.all_captured_foothold_markers.append(captured_marker)
                    num_foothold_markers += 1

                # Republish all accumulated captured foothold markers
                out_msg = MarkerArray()
                # Re-assign sequential IDs across all accumulated markers
                for idx, m in enumerate(self.all_captured_foothold_markers):
                    m.id = idx
                out_msg.markers = list(self.all_captured_foothold_markers)
                self.captured_footholds_publisher.publish(out_msg)
                rospy.loginfo("Captured {} foothold polygon markers (total accumulated: {}).".format(
                    num_foothold_markers, len(self.all_captured_foothold_markers)))
            else:
                rospy.logwarn("No predicted_footholds message received yet, skipping foothold capture.")
            # -----------------------------------------------

            message = "Successfully captured and republished {} transforms and {} foothold markers for capture group #{}.".format(
                len(captured_transforms), num_foothold_markers, self.capture_index)
            rospy.loginfo(message)
            return TriggerResponse(success=True, message=message)

        except Exception as e:
            rospy.logerr("An error occurred during pose capture: {}".format(e))
            return TriggerResponse(success=False, message=str(e))

if __name__ == '__main__':
    try:
        PoseCapture()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass