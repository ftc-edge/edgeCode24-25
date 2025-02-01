package org.firstinspires.ftc.teamcode.util;

public class ProjectionPosition {
    double x; // in cm
    double y; // in cm
    boolean validProjection;

    ProjectionPosition() {
        this.validProjection = false;
        this.x = -50000;
        this.y = -50000;
    }

    ProjectionPosition(double _x, double _y){
        this.x = _x;
        this.y = _y;
    }

    ProjectionPosition(double _xPixel, double _yPixel, double _cameraWidthCM, double _cameraHeightCM, double _cameraWidthPx, double _cameraHeightPx){
        this.x = _xPixel / _cameraWidthPx * _cameraWidthCM - _cameraWidthCM / 2;
        this.y = _yPixel / _cameraHeightPx * _cameraHeightCM - _cameraHeightCM / 2;
    }

    public void ProjectToGrid(double heading){
        double _x = this.x;
        double _y = this.y;
        this.x = (_y * Math.cos(heading)) - (_x * Math.sin(heading));
        this.y = (_x * Math.cos(heading)) + (_y * Math.sin(heading));
    }

    public ProjectionPosition project(double theta, double cameraHeight, double focalLength){
        double RIGHTANGLE = Math.PI / 2;
        ProjectionPosition projection = new ProjectionPosition();

        double xGroundDisplacement = x * cameraHeight / (focalLength * Math.sin(theta));
        double xAngleAtCamera = Math.atan(x/focalLength);
        double xCenterGroundLocation = 0;

        if (xAngleAtCamera >= RIGHTANGLE) {
            return projection;
        }

        double yPrime = y * cameraHeight / (focalLength * Math.sin(theta));
        double yAngleAtCamera = Math.atan(y/focalLength);
        double yGroundDisplacement = yPrime * (Math.sin(RIGHTANGLE-yAngleAtCamera) / Math.sin(theta - yAngleAtCamera));
        double yCenterGroundLocation = cameraHeight / Math.tan(theta);

        if (yAngleAtCamera >= theta) {
            return projection;
        }

        projection.x = xCenterGroundLocation + xGroundDisplacement;
        projection.y = yCenterGroundLocation + yGroundDisplacement;
        projection.validProjection = true;
        return projection;
    }
}
