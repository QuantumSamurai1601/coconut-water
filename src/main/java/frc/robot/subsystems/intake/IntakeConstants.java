package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class IntakeConstants {
    // Intake DIOs
    public static final int INTAKE_FORWARD_LIMIT = 0;
    public static final int INTAKE_BEAM_BREAK = 1;

    // Intake Gains
    public static final double INTAKE_ROLLER_VOLTAGE = 8;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // Intake Motor Configs
    public static final TalonFXConfiguration pivotConfig =  new TalonFXConfiguration();
    static {
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.Slot0.kG = 0.3;
        pivotConfig.Slot0.kS = 0.5;
        pivotConfig.Slot0.kV = 0.15;
        pivotConfig.Slot0.kA = 0.02;
        pivotConfig.Slot0.kP = 45;
        pivotConfig.Slot0.kI = 0;
        pivotConfig.Slot0.kD = 0.2; 
        pivotConfig.MotionMagic.MotionMagicAcceleration = 4.2;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 0.95;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.345;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        pivotConfig.Feedback.SensorToMechanismRatio = 25; // Intake pivot ratio is 25:1
    }

    public static final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    static {
        rollerConfig.Slot0.kS = 0.37043;
        rollerConfig.Slot0.kV = 0.31607;
        rollerConfig.Slot0.kA = 0.0090014;
        rollerConfig.Slot0.kP = 0.1267;
        rollerConfig.Slot1.kS = 16;
        rollerConfig.Slot1.kP = 29;
        rollerConfig.Feedback.SensorToMechanismRatio = 2.5;
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}
 