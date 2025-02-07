package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.tests.ProjectionPosition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;
import java.util.List;

@TeleOp
public class RedObjectDetection extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "logiCam"), cameraMonitorViewId);

        RedPipeline redPipeline = new RedPipeline();
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

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            Point center = redPipeline.getBoundingBoxCenter();
            ProjectionPosition projection = redPipeline.getProjection();
            telemetry.addData("Contours Found", redPipeline.getContoursCount());
            telemetry.addData("Detected Angle (deg)", redPipeline.getDetectedAngle());
            telemetry.addData("Bounding Box Center", "(%.2f, %.2f)", center.x, center.y);
            try {
                telemetry.addData("Projection", "(%.2f, %.2f)", projection.x, projection.y);
            } catch (NullPointerException exception) {
                telemetry.addData("exception", "exception");
            }

            telemetry.update();
            sleep(100);
        }

    }

    static class RedPipeline extends OpenCvPipeline {
        private List<MatOfPoint> contours = new ArrayList<>();
        private double detectedAngle = 0.0;
        private Point centerPoint = new Point(-1, -1);  // Default to (-1, -1) if no object is detected
        private ProjectionPosition projection;
        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();

            // Convert image to HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Apply Gaussian blur to reduce noise
            Imgproc.GaussianBlur(hsv, hsv, new Size(5, 5), 0);

            // Define red color ranges (both low and high hue values)
            Scalar lowerRed1 = new Scalar(0, 120, 70);
            Scalar upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 120, 70);
            Scalar upperRed2 = new Scalar(180, 255, 255);

            // Create masks for both ranges and merge them
            Core.inRange(hsv, lowerRed1, upperRed1, mask1);
            Core.inRange(hsv, lowerRed2, upperRed2, mask2);
            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

            // Find contours
            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                // Remove small noise
                double minContourArea = 500.0;  // Adjust as needed
                contours.removeIf(contour -> Imgproc.contourArea(contour) < minContourArea);

                if (!contours.isEmpty()) {
                    // Find the largest contour
                    MatOfPoint largestContour = contours.get(0);
                    double maxArea = Imgproc.contourArea(largestContour);
                    for (MatOfPoint contour : contours) {
                        double area = Imgproc.contourArea(contour);
                        if (area > maxArea) {
                            maxArea = area;
                            largestContour = contour;
                        }
                    }

                    // Compute minimum area rectangle
                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));

                    // Draw the bounding box
                    Point[] boxPoints = new Point[4];
                    rect.points(boxPoints);
                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 0, 255), 2);
                    }

                    // Calculate the bounding box center
                    centerPoint.x = (boxPoints[0].x + boxPoints[1].x + boxPoints[2].x + boxPoints[3].x) / 4.0;
                    centerPoint.y = (boxPoints[0].y + boxPoints[1].y + boxPoints[2].y + boxPoints[3].y) / 4.0;

                    ProjectionPosition cameraPoint = new ProjectionPosition(centerPoint.x, centerPoint.y, 1, 0.75, 640, 480);
                    projection = cameraPoint.project(Math.toRadians(45), 10, 0.367);;
                    // Draw the center point
                    Imgproc.circle(input, centerPoint, 5, new Scalar(255, 0, 0), -1); // Blue dot at center

                    // Compute the detected angle
                    if (rect.size.width < rect.size.height) {
                        detectedAngle = rect.angle + 90;
                    } else {
                        detectedAngle = rect.angle;
                    }
                }
            }

            // Release resources
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

        public ProjectionPosition getProjection() {
            return projection;
        }
    }

}