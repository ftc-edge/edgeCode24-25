package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.*;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

import org.firstinspires.ftc.teamcode.components.ProjectionPosition;

@Config
public class RedObjectTracking {
    private OpenCvWebcam webcam;
    private RedPipeline redPipeline;

    public static String colorSetting = "red";

    public static double cameraElevation = 14.5;
    public static double cameraAngleDeg = 30; // in Degrees
    public static double cameraHeightCM = 0.5;
    public static double cameraWidthCM = 0.667;
    public static double cameraHeightPX = 480;
    public static double cameraWidthPX = 640;
    public static double cameraFocalLength = 0.367;

    public RedObjectTracking(HardwareMap hardwareMap){
        Scalar lowerBound;
        Scalar upperBound;

        switch (colorSetting.toLowerCase()) {
            case "yellow":
                lowerBound = new Scalar(20, 100, 100);
                upperBound = new Scalar(30, 255, 255);
                break;
            case "blue":
                lowerBound = new Scalar(100, 150, 0);
                upperBound = new Scalar(140, 255, 255);
                break;
            case "red":
            default:
                lowerBound = new Scalar(0, 50, 50);
                upperBound = new Scalar(10, 255, 255);
                break;
        }

        redPipeline = new RedPipeline(lowerBound, upperBound);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "logiCam"), cameraMonitorViewId);

        webcam.setPipeline(redPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode) {

            }
        });
    }

    static class RedPipeline extends OpenCvPipeline {
        private final Deque<Double> angleQueue = new ArrayDeque<>();
        private final List<MatOfPoint> contours = new ArrayList<>();
        private Point centerPoint = new Point(-1, -1);
        private double detectedAngle = 0.0;
        private Scalar lowerBound;
        private Scalar upperBound;

        public RedPipeline(Scalar lowerBound, Scalar upperBound) {
            this.lowerBound = lowerBound;
            this.upperBound = upperBound;
        }

        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();

            // Convert image to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create mask for the selected color
            Core.inRange(hsv, lowerBound, upperBound, mask);

            // Find contours
            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            double largestArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                double epsilon = 0.01 * Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
                Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), approxCurve, epsilon, true);

                double area = Imgproc.contourArea(approxCurve);
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = new MatOfPoint(approxCurve.toArray());
                }
            }

            if (largestContour != null) {
                // Draw largest contour
                List<MatOfPoint> largestContoursList = new ArrayList<>();
                largestContoursList.add(largestContour);
                Imgproc.drawContours(input, largestContoursList, 0, new Scalar(0, 0, 255), 2);

                // Compute the center of mass
                Moments M = Imgproc.moments(largestContour);
                if (M.m00 != 0) {
                    int cX = (int) (M.m10 / M.m00);
                    int cY = (int) (M.m01 / M.m00);
                    centerPoint = new Point(cX, cY);

                    // Draw the center of mass
                    Imgproc.circle(input, centerPoint, 3, new Scalar(255, 0, 0), -1);

                    // Compute bounding box and angle
                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
                    Point[] boxPoints = new Point[4];
                    rect.points(boxPoints);

                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(255, 0, 0), 2);
                    }

                    double angle = rect.angle;
                    if (rect.size.width < rect.size.height) {
                        detectedAngle = angle + 90;
                    } else {
                        detectedAngle = angle;
                    }

                    // Maintain a rolling average of angles
                    angleQueue.add(detectedAngle);
                    if (angleQueue.size() > 5) {
                        angleQueue.poll();
                    }

                    double rollingAvgAngle = angleQueue.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);

                    // Draw direction vector
                    int length = 50;
                    double angleRad = Math.toRadians(rollingAvgAngle);
                    int endX = (int) (cX + length * Math.cos(angleRad));
                    int endY = (int) (cY + length * Math.sin(angleRad));
                    Imgproc.arrowedLine(input, new Point(cX, cY), new Point(endX, endY), new Scalar(0, 255, 255), 2);
                }
            }

            // Release memory
            hsv.release();
            mask.release();
            hierarchy.release();

            return input;
        }

        public int getContoursCount() {
            return contours.size();
        }

        public double getDetectedAngle() {
            return detectedAngle;
        }

        public Point getBoundingBoxCenter() {
            return centerPoint;
        }
    }

    public int getContoursCount() {return redPipeline.getContoursCount();}

    public double getDetectedAngle() {return redPipeline.getDetectedAngle();}

    public double getBoundingBoxCenterX() {
        return redPipeline.getBoundingBoxCenter().x;
    }
    public double getBoundingBoxCenterY() {
        return redPipeline.getBoundingBoxCenter().y;
    }

    public ProjectionPosition getProjection(){
        ProjectionPosition cameraPoint = new ProjectionPosition(redPipeline.getBoundingBoxCenter().x, redPipeline.getBoundingBoxCenter().y, cameraWidthCM, cameraHeightCM, cameraWidthPX, cameraHeightPX);
        return cameraPoint.project(Math.toRadians(cameraAngleDeg), cameraElevation, cameraFocalLength);
    }
}
