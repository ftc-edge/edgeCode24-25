import cv2
import numpy as np
from collections import deque

# Function to detect red color in the frame
def detect_red(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    mask = mask1 + mask2
    return mask

# Function to find and draw contours, convex hull, and center of mass
def find_and_draw_contours(frame, mask, angle_queue):
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest_area = 0
    largest_polygon = None

    for contour in contours:
        epsilon = 0.01 * cv2.arcLength(contour, True)  # Adjusted epsilon value
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Calculate the area of the polygon
        area = cv2.contourArea(approx)

        if area > largest_area:
            largest_area = area
            largest_polygon = approx

    if largest_polygon is not None:
        cv2.drawContours(frame, [largest_polygon], 0, (0, 0, 255), 2)  # Draw largest polygon in red

        # Calculate the center of mass
        M = cv2.moments(largest_polygon)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # Draw the center of mass
            cv2.circle(frame, (cX, cY), 3, (255, 0, 0), -1)  # Draw center of mass in blue

            # Calculate the bounding box height
            _, _, _, h = cv2.boundingRect(largest_polygon)
            new_cY = cY + int(0.25 * h)
            # Draw the new point
            cv2.circle(frame, (cX, new_cY), 3, (0, 255, 255), -1)  # Draw new point in yellow

        # Mark all vertices of the polygon
        vertices = largest_polygon.reshape(-1, 2)
        for vertex in vertices:
            cv2.circle(frame, tuple(vertex), 4, (0, 255, 0), -1)  # Draw vertices in green

        # Calculate the minimum area rectangle
        rect = cv2.minAreaRect(largest_polygon)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)  # Draw the rectangle in blue

        # Get the angle of the rectangle
        angle = rect[2]       

        if rect[1][0] < rect[1][1]:
            detected_angle = rect[2] + 90
        else:
            detected_angle = rect[2]

        angle_queue.append(detected_angle)
        if len(angle_queue) > 5:
            angle_queue.popleft()
        rolling_avg_angle = sum(angle_queue) / len(angle_queue)
        print(f"Rolling average angle of the rectangle: {rolling_avg_angle} degrees")

        # Calculate the average coordinate of the rectangle
        avg_rect_x = np.mean(box[:, 0])
        avg_rect_y = np.mean(box[:, 1])

        # Calculate the average between the yellow dot and the average coordinate of the rectangle
        avg_x = int((cX + avg_rect_x) / 2)
        avg_y = int((new_cY + avg_rect_y) / 2)

        # Draw the average point
        cv2.circle(frame, (avg_x, avg_y), 5, (255, 255, 0), -1)  # Draw average point in cyan

        avg_rect_y = np.mean(box[:, 1])

        # Calculate the average between the yellow dot and the average coordinate of the rectangle
        avg_x = int((cX + avg_rect_x) / 2)
        avg_y = int((new_cY + avg_rect_y) / 2)

        # Draw the average point
        cv2.circle(frame, (avg_x, avg_y), 5, (255, 255, 0), -1)  # Draw average point in cyan

        # Draw a vector from the cyan point in the direction of the angle
        length = 50  # Length of the vector
        angle_rad = np.deg2rad(rolling_avg_angle)
        end_x = int(avg_x + length * np.cos(angle_rad))
        end_y = int(avg_y + length * np.sin(angle_rad))
        cv2.arrowedLine(frame, (avg_x, avg_y), (end_x, end_y), (0, 255, 255), 2)  # Draw vector in yellow

# Main function to capture video and process frames
def main():
    cap = cv2.VideoCapture(1)
    angle_queue = deque()
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        mask = detect_red(frame)
        find_and_draw_contours(frame, mask, angle_queue)

        cv2.imshow('Frame', frame)
        cv2.imshow('Mask', mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
