package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
    private final VictorSPX leftIntake;
    private final VictorSPX rightIntake;
    private final TalonFX leftIntakePivot;
    private final TalonFX rightIntakePivot;
    private final DutyCycleOut leftPivotRequest;
    private final Follower rightPivotRequest;

    public IntakeSubsystem() {
        leftIntake = new VictorSPX(LEFT_INTAKE_ID);
        rightIntake = new VictorSPX(RIGHT_INTAKE_ID);
        leftIntakePivot = new TalonFX(LEFT_INTAKE_PIVOT_ID, CANBUS_NAME);
        rightIntakePivot = new TalonFX(RIGHT_INTAKE_PIVOT_ID, CANBUS_NAME);
        leftPivotRequest = new DutyCycleOut(0.0, true, false, false, false);
        rightPivotRequest = new Follower(LEFT_INTAKE_PIVOT_ID, false);
    }

    public Command chomp() {
        return this.runOnce(() -> {
            leftIntake.set(ControlMode.PercentOutput, 1);
            rightIntake.set(ControlMode.Follower, LEFT_INTAKE_ID);
        });
    }

    public Command vomit() {
        return this.runOnce(() -> {
            leftIntake.set(ControlMode.PercentOutput, -1);
            rightIntake.set(ControlMode.Follower, LEFT_INTAKE_ID);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            leftIntake.set(ControlMode.PercentOutput, 0);
            rightIntake.set(ControlMode.Follower, LEFT_INTAKE_ID);
        });
    }

    public Command pivot(double speed) {
        return this.run(() -> {
            leftIntakePivot.setControl(leftPivotRequest.withOutput(speed));
            rightIntakePivot.setControl(rightPivotRequest);
        });
    }
}