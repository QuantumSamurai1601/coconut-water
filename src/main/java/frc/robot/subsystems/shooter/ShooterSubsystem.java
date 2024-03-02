package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftShooter;
    private final TalonFX rightShooter;
    private final TalonFX feeder;
    private final TalonFX leftPivot;
    private final TalonFX rightPivot;

    public ShooterSubsystem() {
        leftShooter = new TalonFX(LEFT_SHOOTER_ID, CANBUS_NAME);
        rightShooter = new TalonFX(RIGHT_SHOOTER_ID, CANBUS_NAME);
        feeder = new TalonFX(FEEDER_ID, CANBUS_NAME);
        leftPivot = new TalonFX(LEFT_SHOOTER_PIVOT_ID, CANBUS_NAME);
        rightPivot = new TalonFX(RIGHT_SHOOTER_PIVOT_ID, CANBUS_NAME);
    }
    
}
