package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();
    static {
    topShooterConfig.Slot0.kS = 0.21123;
    topShooterConfig.Slot0.kV = 0.13574;
    topShooterConfig.Slot0.kA = 0.029807;    
    topShooterConfig.Slot0.kP = 0.16118;
    topShooterConfig.Slot0.kI = 0.0;
    topShooterConfig.Slot0.kD = 0.0;
    topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    public static final TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();
    static {
    bottomShooterConfig.Slot0.kS = 0.39951;
    bottomShooterConfig.Slot0.kV = 0.14081;
    bottomShooterConfig.Slot0.kA = 0.037933;
    bottomShooterConfig.Slot0.kP = 0.18765;
    bottomShooterConfig.Slot0.kI = 0.0;
    bottomShooterConfig.Slot0.kD = 0.0;
    bottomShooterConfig.Slot1.kS = 14.85;
    bottomShooterConfig.Slot1.kP = 41.25;
    bottomShooterConfig.Feedback.SensorToMechanismRatio = 0.6666666667;
    bottomShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    public static final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    static{
    pivotConfig.Slot0.kG = 0.03;
    pivotConfig.Slot0.kS = 0.32;
    pivotConfig.Slot0.kV = 0.125;
    pivotConfig.Slot0.kA = 0.02069;
    pivotConfig.Slot0.kP = 300.0;
    pivotConfig.Slot0.kI = 0.0;
    pivotConfig.Slot0.kD = 0.1;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.MotionMagic.MotionMagicAcceleration = 2.5;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Feedback.SensorToMechanismRatio = 133.33;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.19;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    }

    public static final InterpolatingDoubleTreeMap shooterTreeMap = new InterpolatingDoubleTreeMap();
    static {
        shooterTreeMap.put(1.45, 0.185);
        shooterTreeMap.put(2.352, 0.13);
        shooterTreeMap.put(3.254, 0.115);
    }
}
