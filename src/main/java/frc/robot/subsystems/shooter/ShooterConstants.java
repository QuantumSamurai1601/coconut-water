package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    // Shooter Motor Configs
    public static final TalonFXConfiguration topShooterConfig = new TalonFXConfiguration();
    static {
    topShooterConfig.Slot0.kS = 0.31;
    topShooterConfig.Slot0.kV = 0.09;
    topShooterConfig.Slot0.kA = 0.0;
    topShooterConfig.Slot0.kP = 0.4;
    topShooterConfig.Slot0.kI = 0.0;
    topShooterConfig.Slot0.kD = 0.001;
    topShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    topShooterConfig.Feedback.SensorToMechanismRatio = 0.625;
    topShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    public static final TalonFXConfiguration bottomShooterConfig = new TalonFXConfiguration();
    static {
    bottomShooterConfig.Slot0.kS = 0.2;
    bottomShooterConfig.Slot0.kV = 0.08;
    bottomShooterConfig.Slot0.kA = 0.0;
    bottomShooterConfig.Slot0.kP = 0.4;
    bottomShooterConfig.Slot0.kI = 0.0;
    bottomShooterConfig.Slot0.kD = 0.001;
    bottomShooterConfig.Feedback.SensorToMechanismRatio = 0.625;
    bottomShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
        shooterTreeMap.put(1.45, 0.175);
        shooterTreeMap.put(2.352, 0.13);
        shooterTreeMap.put(3.254, 0.115);
    }
}
