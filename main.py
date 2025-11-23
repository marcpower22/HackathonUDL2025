import time
import cv2
import numpy as np
import csv
import datetime
import mediapipe as mp

def main():
    # Prefer V4L2 backend on Linux when available
    backend = cv2.CAP_V4L2 if hasattr(cv2, 'CAP_V4L2') else 0
    cap = cv2.VideoCapture(0, backend)

    if not cap.isOpened():
        print('ERROR: No se pudo abrir la cámara. Comprueba el índice del dispositivo y permisos.')
        return

    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils

    tracker = CentroidTracker(maxDisappeared=40, maxDistance=80)

    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    # Setup CSV logging
    camera_name = "Camera_01"
    csv_filename = f"skeleton_log_{int(time.time())}.csv"
    csv_file = open(csv_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # Header: Timestamp, Camera, BodyID, then 33 landmarks (x, y, z, visibility)
    header = ['Timestamp', 'Camera', 'BodyID']
    for i in range(33):
        header.extend([f'L{i}_x', f'L{i}_y', f'L{i}_z', f'L{i}_v'])
    csv_writer.writerow(header)

    prev_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print('WARNING: no se recibió frame desde la cámara.')
            break

        orig = frame.copy()

        # MediaPipe expects RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb)

        centroids = []

        if results.pose_landmarks:
            h, w, _ = frame.shape
            # Draw landmarks and connections
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # Try to compute a stable centroid from hips and shoulders when available
            lm = results.pose_landmarks.landmark
            indices = []
            # left/right shoulders and hips in MediaPipe Pose
            for idx in (11, 12, 23, 24):
                indices.append(idx)

            pts = []
            for idx in indices:
                l = lm[idx]
                if l.visibility < 0.3:
                    continue
                px = int(l.x * w)
                py = int(l.y * h)
                pts.append((px, py))

            if len(pts) == 0:
                # fallback: average all visible landmarks
                for l in lm:
                    if l.visibility < 0.3:
                        continue
                    px = int(l.x * w)
                    py = int(l.y * h)
                    pts.append((px, py))

            if len(pts) > 0:
                avg_x = int(sum([p[0] for p in pts]) / len(pts))
                avg_y = int(sum([p[1] for p in pts]) / len(pts))
                centroids.append((avg_x, avg_y))

        # Update tracker with pose centroids (empty list allowed)
        objects = tracker.update_centroids(centroids)

        # Log data if we have a detection and a tracked object
        if len(centroids) > 0 and results.pose_landmarks:
            # The current detection corresponds to centroids[0]
            # Find which object ID owns this centroid
            current_c = centroids[0]
            matched_id = -1
            for oid, oc in objects.items():
                if oc == current_c:
                    matched_id = oid
                    break
            
            if matched_id != -1:
                row = [datetime.datetime.now().isoformat(), camera_name, matched_id]
                for lm in results.pose_landmarks.landmark:
                    row.extend([lm.x, lm.y, lm.z, lm.visibility])
                csv_writer.writerow(row)

        # Draw object IDs near centroids
        for (objectID, centroid) in objects.items():
            text = f'ID {objectID}'
            cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)
            cv2.circle(frame, (centroid[0], centroid[1]), 6, (0, 0, 255), -1)

        # FPS
        now = time.time()
        fps = 1.0 / (now - prev_time) if now != prev_time else 0.0
        prev_time = now
        cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        cv2.imshow('Body Tracking', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # save snapshot
            timestamp = int(time.time())
            fname = f'snapshot_{timestamp}.png'
            cv2.imwrite(fname, orig)
            print(f'Snapshot saved to {fname}')

    pose.close()
    csv_file.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()