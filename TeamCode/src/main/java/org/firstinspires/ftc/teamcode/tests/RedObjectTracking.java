// this is new code

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

@TeleOp(name = "RedObjectTracking", group = "Vision")
public class RedObjectTracking extends LinearOpMode {
    private OpenCvWebcam webcam;
    private RedPipeline redPipeline = new RedPipeline();

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(redPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Contours Found", redPipeline.getContoursCount());
            telemetry.addData("Detected Angle (deg)", redPipeline.getDetectedAngle());
            telemetry.addData("Bounding Box Center", "(%.2f, %.2f)", redPipeline.getBoundingBoxCenter().x, redPipeline.getBoundingBoxCenter().y);
            telemetry.update();
            sleep(100);
        }
    }

    static class RedPipeline extends OpenCvPipeline {
        private final Deque<Double> angleQueue = new ArrayDeque<>();
        private final List<MatOfPoint> contours = new ArrayList<>();
        private Point centerPoint = new Point(-1, -1);
        private double detectedAngle = 0.0;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();

            // Convert image to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define red color range
            Scalar lowerRed1 = new Scalar(0, 50, 50);
            Scalar upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 50, 50);
            Scalar upperRed2 = new Scalar(180, 255, 255);

            // Create masks for red color
            Core.inRange(hsv, lowerRed1, upperRed1, mask1);
            Core.inRange(hsv, lowerRed2, upperRed2, mask2);
            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

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
                Imgproc.drawContours(input, List.of(largestContour), 0, new Scalar(0, 0, 255), 2);

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
            mask1.release();
            mask2.release();
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
}
