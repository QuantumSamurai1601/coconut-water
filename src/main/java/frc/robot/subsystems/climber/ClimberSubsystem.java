package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX leftClimber;
    private final TalonFX rightClimber;
    private final DutyCycleOut leftRequest;
    private final Follower rightRequest;

    public ClimberSubsystem() {
        leftClimber = new TalonFX(LEFT_CLIMBER_ID, CANBUS_NAME);
        rightClimber = new TalonFX(RIGHT_CLIMBER_ID, CANBUS_NAME);

        leftClimber.setNeutralMode(NeutralModeValue.Brake);
        rightClimber.setNeutralMode(NeutralModeValue.Brake);

        leftRequest = new DutyCycleOut(0.0, true, false, false, false);
        rightRequest = new Follower(LEFT_CLIMBER_ID, false);
    }

    public Command climb(double speed) {
        return this.run(() -> {
            leftClimber.setControl(leftRequest.withOutput(speed));
            rightClimber.setControl(rightRequest);
        });
    }
}