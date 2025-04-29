import cv2
import numpy as np
import requests
import time
from threading import Thread
from collections import deque

# URLs for camera and ultrasonic sensor
VIDEO_URL = 'http://100.81.43.28:5000/video_feed'
ULTRASONIC_URL = 'http://100.81.43.28:5000/ultrasonic_data'

# Global variable for distance sensor reading
distance_cm = None

# Example camera matrix and distortion coefficients
# Replace these with your calibrated values for accurate results
camera_matrix = np.array([
    [800, 0, 320],
    [0, 800, 240],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1), dtype=np.float32)

# Feature detection parameters
feature_params = dict(
    maxCorners=100,
    qualityLevel=0.3,
    minDistance=7,
    blockSize=7
)

# Lucas-Kanade optical flow parameters
lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
)

# ORB for feature detection (more efficient than SIFT)
orb = cv2.ORB_create(nfeatures=1000)

# Variables for camera pose tracking
prev_frame = None
prev_points = None
prev_kps = None
prev_desc = None
reference_points_3d = None
reference_image = None
reference_kps = None
reference_desc = None
is_reference_set = False
trajectory = deque(maxlen=100)  # Store recent camera positions

# Feature matching parameters
# Use BFMatcher with Hamming distance for ORB descriptors
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

# Essential matrix threshold
ESSENTIAL_MATRIX_THRESHOLD = 3.0

def get_distance():
    """Continuously fetch distance data from ultrasonic sensor"""
    global distance_cm
    while True:
        try:
            response = requests.get(ULTRASONIC_URL, timeout=1)
            if response.status_code == 200:
                data = response.json()
                distance_cm = data.get('distance_cm')
                print(f"Distance: {distance_cm} cm")
        except Exception as e:
            print(f"Error fetching distance data: {e}")
        time.sleep(0.1)  # Update every 100ms

def set_reference_frame(frame):
    """Set reference frame and extract features for initial pose estimation"""
    global reference_image, reference_kps, reference_desc, reference_points_3d, is_reference_set
    
    # Store reference frame
    reference_image = frame.copy()
    
    # Extract ORB features
    gray = cv2.cvtColor(reference_image, cv2.COLOR_BGR2GRAY)
    reference_kps, reference_desc = orb.detectAndCompute(gray, None)
    
    if reference_kps and len(reference_kps) > 0:
        # Initialize 3D points (assume all points are on a plane at Z=0)
        # We'll refine these later with actual depth information
        reference_points_2d = np.array([kp.pt for kp in reference_kps], dtype=np.float32)
        reference_points_3d = np.zeros((len(reference_kps), 3), dtype=np.float32)
        reference_points_3d[:, :2] = reference_points_2d
        
        is_reference_set = True
        print(f"Set reference frame with {len(reference_kps)} keypoints")
        return reference_kps, reference_desc
    else:
        print("Warning: No keypoints detected in reference frame")
        return None, None

def estimate_pose_with_features(frame, ref_kps, ref_desc, ref_points_3d):
    """Estimate camera pose using feature matching between current frame and reference"""
    if ref_kps is None or ref_desc is None or len(ref_kps) < 8:
        return None, None, frame, False
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect features and compute descriptors
    curr_kps, curr_desc = orb.detectAndCompute(gray, None)
    
    if curr_kps is None or curr_desc is None or len(curr_kps) < 8:
        return None, None, frame, False
    
    # Match descriptors using BFMatcher with knnMatch
    try:
        matches = bf.knnMatch(ref_desc, curr_desc, k=2)
    except cv2.error:
        # Fallback if knnMatch fails
        return None, None, frame, False
    
    # Apply ratio test to find good matches
    good_matches = []
    for pair in matches:
        if len(pair) == 2:  # Ensure we have 2 matches for ratio test
            m, n = pair
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)
    
    # Need at least 8 point correspondences for pose estimation
    if len(good_matches) < 8:
        print(f"Not enough good matches: {len(good_matches)}")
        # Draw the matches we do have
        match_img = cv2.drawMatches(reference_image, ref_kps, frame, curr_kps, good_matches, None)
        return None, None, match_img, False
    
    # Extract matched keypoints
    ref_matched_pts = np.float32([ref_points_3d[m.queryIdx] for m in good_matches])
    curr_matched_pts = np.float32([curr_kps[m.trainIdx].pt for m in good_matches])
    
    # Draw matches
    match_img = cv2.drawMatches(reference_image, ref_kps, frame, curr_kps, good_matches[:20], None)
    
    # If we have distance from ultrasonic sensor, use it to scale 3D points
    if distance_cm is not None:
        # Adjust depth based on ultrasonic measurement
        scale_factor = distance_cm / 100.0  # Convert to meaningful scale
        
        # Create 3D points with varying depth based on image position (simplified)
        scene_points_3d = np.zeros((len(good_matches), 3), dtype=np.float32)
        for i, (pt_3d, pt_2d) in enumerate(zip(ref_matched_pts, curr_matched_pts)):
            # Copy X and Y from 2D point, set Z from distance sensor
            scene_points_3d[i] = [pt_3d[0], pt_3d[1], scale_factor]
    else:
        # Without distance, assume points are at Z=0 (planar scene)
        scene_points_3d = ref_matched_pts.copy()
    
    try:
        # Use PnP to estimate pose
        success, rvec, tvec = cv2.solvePnP(
            scene_points_3d, curr_matched_pts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if success:
            return rvec, tvec, match_img, True
        else:
            return None, None, match_img, False
    except cv2.error as e:
        print(f"Error in solvePnP: {e}")
        return None, None, match_img, False

def estimate_pose_with_optical_flow(prev_gray, current_frame, prev_points):
    """Estimate camera pose using optical flow tracking between consecutive frames"""
    current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    
    if prev_points is None or len(prev_points) < 8:
        # Detect initial points if none exist
        prev_points = cv2.goodFeaturesToTrack(prev_gray, mask=None, **feature_params)
        if prev_points is None:
            return None, None, current_frame, False, None
    
    # Calculate optical flow
    current_points, status, err = cv2.calcOpticalFlowPyrLK(
        prev_gray, current_gray, prev_points, None, **lk_params
    )
    
    # Select good points
    if current_points is None:
        return None, None, current_frame, False, None
    
    good_current = current_points[status == 1]
    good_prev = prev_points[status == 1]
    
    # Need enough points for pose estimation
    if len(good_current) < 8:
        return None, None, current_frame, False, good_current.reshape(-1, 1, 2)
    
    # Draw optical flow tracks
    flow_img = current_frame.copy()
    for i, (new, old) in enumerate(zip(good_current, good_prev)):
        a, b = new.ravel()
        c, d = old.ravel()
        flow_img = cv2.line(flow_img, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
        flow_img = cv2.circle(flow_img, (int(a), int(b)), 5, (0, 0, 255), -1)
    
    try:
        # Find essential matrix
        E, mask = cv2.findEssentialMat(
            good_current, good_prev, camera_matrix, method=cv2.RANSAC, prob=0.999, threshold=ESSENTIAL_MATRIX_THRESHOLD
        )
        
        if E is None:
            return None, None, flow_img, False, good_current.reshape(-1, 1, 2)
        
        # Recover pose from essential matrix
        _, R, t, mask = cv2.recoverPose(E, good_current, good_prev, camera_matrix)
        
        # Convert rotation matrix to rotation vector
        rvec, _ = cv2.Rodrigues(R)
        
        # Scale translation vector if distance data is available
        if distance_cm is not None:
            # Use distance to scale the translation vector
            norm_t = np.linalg.norm(t)
            if norm_t > 0:
                t = t * (distance_cm / 100.0) / norm_t  # Scale to have correct magnitude
        
        return rvec, t, flow_img, True, good_current.reshape(-1, 1, 2)
    except cv2.error as e:
        print(f"Error in pose recovery: {e}")
        return None, None, flow_img, False, good_current.reshape(-1, 1, 2)

def draw_pose_info(frame, rvec, tvec):
    """Draw coordinate axes and pose information on frame"""
    try:
        # Draw coordinate system
        axis_length = 0.1  # meters
        axis_points = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]])
        imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
        
        origin = tuple(map(int, imgpts[0].ravel()))
        x_axis = tuple(map(int, imgpts[1].ravel()))
        y_axis = tuple(map(int, imgpts[2].ravel()))
        z_axis = tuple(map(int, imgpts[3].ravel()))
        
        frame = cv2.line(frame, origin, x_axis, (0, 0, 255), 3)  # X-axis: Red
        frame = cv2.line(frame, origin, y_axis, (0, 255, 0), 3)  # Y-axis: Green
        frame = cv2.line(frame, origin, z_axis, (255, 0, 0), 3)  # Z-axis: Blue
        
        # Extract Euler angles
        rot_matrix, _ = cv2.Rodrigues(rvec)
        sy = np.sqrt(rot_matrix[0, 0] * rot_matrix[0, 0] + rot_matrix[1, 0] * rot_matrix[1, 0])
        singular = sy < 1e-6
        
        if not singular:
            roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
            pitch = np.arctan2(-rot_matrix[2, 0], sy)
            yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
        else:
            roll = np.arctan2(-rot_matrix[1, 2], rot_matrix[1, 1])
            pitch = np.arctan2(-rot_matrix[2, 0], sy)
            yaw = 0
        
        # Convert to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        
        # Display pose information
        info_text = [
            f"Position (X,Y,Z): ({tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f})",
            f"Orientation (Roll,Pitch,Yaw): ({roll_deg:.2f}°, {pitch_deg:.2f}°, {yaw_deg:.2f}°)",
        ]
        
        if distance_cm is not None:
            info_text.append(f"Ultrasonic Distance: {distance_cm:.2f} cm")
        
        # Draw information on frame
        for i, text in enumerate(info_text):
            cv2.putText(frame, text, (10, 30 + i * 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    except Exception as e:
        print(f"Error drawing pose info: {e}")
    
    return frame

def draw_trajectory(frame, trajectory):
    """Draw camera trajectory on the frame"""
    h, w = frame.shape[:2]
    traj_img = np.zeros((h//3, w//3, 3), dtype=np.uint8)
    
    # Draw trajectory points
    scale = 5.0  # Scale factor for visualization
    center_x, center_y = traj_img.shape[1] // 2, traj_img.shape[0] // 2
    
    prev_point = None
    for pos in trajectory:
        x, y = int(center_x + pos[0] * scale), int(center_y + pos[1] * scale)
        if 0 <= x < traj_img.shape[1] and 0 <= y < traj_img.shape[0]:
            # Draw point
            cv2.circle(traj_img, (x, y), 1, (0, 255, 255), -1)
            
            # Draw line connecting to previous point
            if prev_point is not None:
                cv2.line(traj_img, prev_point, (x, y), (0, 255, 0), 1)
            
            prev_point = (x, y)
    
    # Add border and title
    cv2.putText(traj_img, "Camera Trajectory (Top View)", (10, 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Place trajectory image in corner of main frame
    frame[10:10+traj_img.shape[0], 10:10+traj_img.shape[1]] = traj_img
    
    return frame

def main():
    global prev_frame, prev_points, reference_kps, reference_desc, reference_points_3d, is_reference_set, trajectory
    
    print("Starting camera pose estimation...")
    
    # Start distance sensing in a separate thread
    distance_thread = Thread(target=get_distance, daemon=True)
    distance_thread.start()
    
    # OpenCV video capture from URL stream
    print(f"Connecting to video stream at {VIDEO_URL}")
    cap = cv2.VideoCapture(VIDEO_URL)
    
    if not cap.isOpened():
        print("Error: Could not open video stream.")
        return
    
    print("Successfully connected to video stream")
    
    # Wait for first frame to initialize
    ret, frame = cap.read()
    if not ret:
        print("Failed to receive initial frame. Exiting...")
        return
    
    print("Initializing reference frame...")
    # Set first frame as reference
    reference_kps, reference_desc = set_reference_frame(frame)
    
    # Initialize previous frame
    prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Tracking method state
    use_feature_matching = True  # Toggle between feature matching and optical flow
    frame_count = 0
    
    print("Starting main processing loop...")
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to receive frame. Exiting...")
            break
        
        frame_count += 1
        
        # Switch methods periodically or based on tracking quality
        if frame_count % 30 == 0:  # Toggle methods every 30 frames
            use_feature_matching = not use_feature_matching
            print(f"Switched to {'feature matching' if use_feature_matching else 'optical flow'}")
        
        # Process frame for pose estimation
        if use_feature_matching and is_reference_set:
            # Method 1: Feature matching with reference frame
            rvec, tvec, vis_frame, success = estimate_pose_with_features(
                frame, reference_kps, reference_desc, reference_points_3d
            )
            method_text = "Method: Feature Matching"
        else:
            # Method 2: Optical flow tracking between consecutive frames
            rvec, tvec, vis_frame, success, tracked_points = estimate_pose_with_optical_flow(
                prev_frame, frame, prev_points
            )
            prev_points = tracked_points
            method_text = "Method: Optical Flow"
        
        # Update previous frame
        prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Draw method information
        cv2.putText(vis_frame, method_text, (10, vis_frame.shape[0] - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        if success and rvec is not None and tvec is not None:
            # Draw pose information and coordinate system
            vis_frame = draw_pose_info(vis_frame, rvec, tvec)
            
            # Update trajectory
            if tvec is not None:
                trajectory.append((tvec[0][0], tvec[2][0]))  # X and Z for top-down view
            
            # Draw trajectory
            vis_frame = draw_trajectory(vis_frame, trajectory)
        else:
            cv2.putText(vis_frame, "Pose estimation failed", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display the frame
        cv2.imshow('Camera Pose Estimation', vis_frame)
        
        # Break loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Clean up
    cap.release()
    cv2.destroyAllWindows()
    print("Program terminated")

if __name__ == "__main__":
    main()