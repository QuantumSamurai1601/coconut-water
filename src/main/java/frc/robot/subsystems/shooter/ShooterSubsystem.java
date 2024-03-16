package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX topShooter;
    private final TalonFX bottomShooter;
    private final TalonFX feeder;
    private final TalonFX leftPivot;
    private final TalonFX rightPivot;
    
    private final DutyCycleOut topRequest;
    private final Follower bottomRequest;

    public ShooterSubsystem() {
        topShooter = new TalonFX(TOP_SHOOTER_ID, CANBUS_NAME);
        bottomShooter = new TalonFX(BOTTOM_SHOOTER_ID, CANBUS_NAME);
        feeder = new TalonFX(FEEDER_ID, CANBUS_NAME);
        leftPivot = new TalonFX(LEFT_SHOOTER_PIVOT_ID, CANBUS_NAME);
        rightPivot = new TalonFX(RIGHT_SHOOTER_PIVOT_ID, CANBUS_NAME);

        topRequest =  new DutyCycleOut(0.0, true, false, false, false);
        bottomRequest = new Follower(TOP_SHOOTER_ID, false);
    }
    
    public Command shootTest() {
        return this.run(() -> {
        topShooter.setControl(topRequest.withOutput(-0.75));
        bottomShooter.setControl(bottomRequest);
        });
    }
}
