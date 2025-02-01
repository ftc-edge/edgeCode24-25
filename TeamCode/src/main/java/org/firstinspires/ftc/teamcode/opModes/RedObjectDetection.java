package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
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
            telemetry.addData("Contours Found", redPipeline.getContoursCount());
            telemetry.addData("Detected Angle (deg)", redPipeline.getDetectedAngle());
            telemetry.update();
            sleep(100);
        }
    }

    static class RedPipeline extends OpenCvPipeline {
        private List<MatOfPoint> contours = new ArrayList<>();
        private double detectedAngle = 0.0;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();

            // Convert to HSV
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define red color range
            Scalar lowerRed = new Scalar(0, 120, 70);
            Scalar upperRed = new Scalar(10, 255, 255);

            // Create mask
            Core.inRange(hsv, lowerRed, upperRed, mask);

            // Find contours
            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                // Find the largest contour based on area
                MatOfPoint largestContour = contours.get(0);
                double maxArea = Imgproc.contourArea(largestContour);
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                // Compute bounding rectangle
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint(largestContour.toArray()));
                Point[] boxPoints = new Point[4];
                rect.points(boxPoints);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 0, 255), 2);
                }

                // Calculate angle
                detectedAngle = rect.angle;
                if (detectedAngle < -45) {
                    detectedAngle = -(90 + detectedAngle);
                } else {
                    detectedAngle = -detectedAngle;
                }
            }

            // Release resources
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
    }
}
