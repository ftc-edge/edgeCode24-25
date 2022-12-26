package org.firstinspires.ftc.teamcode.robotbase;

import static com.qualcomm.robotcore.util.Range.clip;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import java.util.function.DoubleSupplier;

public class BalancingSubsystem extends SubsystemBase {
    private DoubleSupplier gyroPitchValue;
    private DoubleSupplier gyroRollValue;

    private boolean enabled = false;

    private final double kP = 1.0;
    private final double kI = 0;
    private final double kD = 0;

    PIDController pitchController;
    PIDController rollController;

    private final double pitch_kP = 1,
            pitch_kI = 0,
            pitch_kD = 0,
            roll_kP = 1,
            roll_kI = 0,
            roll_kD = 0;

    private double pitchCorrection, rollCorrection;

    public BalancingSubsystem (DoubleSupplier gyroPitchValue, DoubleSupplier gyroRollValue) {
        pitchController = new PIDController(pitch_kP, pitch_kI, pitch_kD);
        rollController = new PIDController(roll_kP, roll_kI, roll_kD);

        this.gyroPitchValue = gyroPitchValue;
        this.gyroRollValue = gyroRollValue;
    }

    public void periodic() {
       pitchCorrection = pitchController.calculate(gyroPitchValue.getAsDouble());
       rollCorrection = rollController.calculate(gyroRollValue.getAsDouble());

       pitchCorrection = clip(pitchCorrection, -1, 1);
       rollCorrection = clip(rollCorrection, -1, 1);
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void toggleState() {
        enabled = !enabled;
    }

    public double getPitchCorrection() {
        return pitchCorrection;
    }

    public double getRollCorrection() {
        return rollCorrection;
    }
}