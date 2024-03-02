package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leftTopMotor;
    private final TalonFX leftBottomMotor;
    private final TalonFX rightTopMotor;
    private final TalonFX rightBottomMotor;

    public ShooterSubsystem() {
        leftTopMotor = new TalonFX(LEFT_TOP_SHOOTERFEEDER_ID, CANBUS_NAME);
        leftBottomMotor = new TalonFX(LEFT_BOTTOM_SHOOTER_ID, CANBUS_NAME);
        rightTopMotor = new TalonFX(RIGHT_TOP_SHOOTERFEEDER_ID, CANBUS_NAME);
        rightBottomMotor = new TalonFX(RIGHT_BOTTOM_SHOOTER_ID, CANBUS_NAME);
    }
    
}
