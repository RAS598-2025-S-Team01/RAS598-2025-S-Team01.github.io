import cv2
import numpy as np
import matplotlib.pyplot as plt
import requests
from collections import deque

# === Configuration ===
VIDEO_URL = 'http://100.81.43.28:5000/video_feed'
ULTRASONIC_URL = 'http://100.81.43.28:5000/ultrasonic_data'
camera_matrix = np.array([[800, 0, 320],
                          [0, 800, 240],
                          [0,   0,   1]], dtype=np.float32)

# === ORB and matcher ===
orb = cv2.ORB_create(1000)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# === Plotting setup ===
trajectory = deque(maxlen=500)
fig, ax = plt.subplots()
sc = ax.scatter([], [], c='blue')
ax.set_title('Camera Trajectory')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel('x (m)')
ax.set_ylabel('z (m)')
plt.ion()
plt.show()

# === Ultrasonic fetch ===
def get_ultrasonic_distance():
    try:
        r = requests.get(ULTRASONIC_URL, timeout=0.2)
        data = r.json()
        return float(data['distance_cm']) / 100.0  # cm to meters
    except:
        return None

# === Video stream ===
cap = cv2.VideoCapture(VIDEO_URL)

prev_frame = None
R_total = np.eye(3)
t_total = np.zeros((3, 1))
frame_idx = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if prev_frame is None:
        prev_frame = gray
        frame_idx += 1
        continue

    # === Feature detection and matching ===
    kp1, des1 = orb.detectAndCompute(prev_frame, None)
    kp2, des2 = orb.detectAndCompute(gray, None)

    if des1 is None or des2 is None:
        prev_frame = gray
        frame_idx += 1
        continue

    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)[:100]

    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    # === Pose estimation ===
    E, mask = cv2.findEssentialMat(pts1, pts2, camera_matrix, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None:
        prev_frame = gray
        frame_idx += 1
        continue

    _, R, t, mask_pose = cv2.recoverPose(E, pts1, pts2, camera_matrix)

    # === Scale correction using ultrasonic sensor ===
    if frame_idx % 30 == 0:
        ultrasonic_d = get_ultrasonic_distance()
        if ultrasonic_d:
            estimated_scale = np.linalg.norm(t)
            if estimated_scale > 0:
                scale_factor = ultrasonic_d / estimated_scale
                t *= scale_factor
                t[1] *= -1  # flip Y if upside-down

    # === Accumulate motion ===
    t_total += R_total @ t
    R_total = R @ R_total

    x, z = t_total[0][0], t_total[2][0]
    trajectory.append((x, z))

    # === Plot update ===
    traj_np = np.array(trajectory)
    sc.set_offsets(traj_np)
    ax.set_xlim(traj_np[:, 0].min() - 0.5, traj_np[:, 0].max() + 0.5)
    ax.set_ylim(traj_np[:, 1].min() - 0.5, traj_np[:, 1].max() + 0.5)
    plt.pause(0.001)

    prev_frame = gray
    frame_idx += 1

    # Show video
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
plt.ioff()
plt.show()
