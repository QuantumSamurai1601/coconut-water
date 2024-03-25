package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    // Intake Motor
    private final TalonFX intake;
    // Intake Pivot Motors and Sensors
    private final TalonFX leaderIntakePivot;
    private final TalonFX followerIntakePivot;
    private final DigitalInput forwardLimit;
    private final DigitalInput beamBreak;
    // Intake Control Requests
    private final VoltageOut intakeRequest;
    // Intake Pivot Control Requests
    private final PositionTorqueCurrentFOC leaderPivotRequest;
    private final Follower followerPivotRequest;
    // Neutral request, determined by NeutralModeValue of motor
    private final NeutralOut neutral;
    // Motor Telemetry
    private final StatusSignal<Double> rollerVelocity;
    private final StatusSignal<Double> rollerAppliedVoltage;
    private final StatusSignal<Double> rollerTempC;
    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotTorqueCurrent;
    private final StatusSignal<Double> pivotLeaderTempC;
    // Leader pivot config
    private final TalonFXConfiguration pivotConfig;
    // Current limit config
    private final TalonFXConfiguration currentLimit;

    public IntakeSubsystem() {
        //Intake Rollers
        intake = new TalonFX(INTAKE_ID, CANBUS_NAME);
        beamBreak = new DigitalInput(INTAKE_BEAM_BREAK);
        intakeRequest = new VoltageOut(0).withEnableFOC(true).withLimitReverseMotion(beamBreak.get());
        intake.setInverted(true);
                
        // Intake Pivot
        leaderIntakePivot = new TalonFX(LEFT_INTAKE_PIVOT_ID, CANBUS_NAME);
        followerIntakePivot =  new TalonFX(RIGHT_INTAKE_PIVOT_ID, CANBUS_NAME);
        forwardLimit = new DigitalInput(INTAKE_FORWARD_LIMIT);
        leaderPivotRequest = new PositionTorqueCurrentFOC(0);
        followerPivotRequest = new Follower(LEFT_INTAKE_PIVOT_ID, true);
        neutral = new NeutralOut();

        // Current Limit Config
        currentLimit = new TalonFXConfiguration();
        currentLimit.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        currentLimit.CurrentLimits.StatorCurrentLimitEnable = true;
        intake.getConfigurator().apply(currentLimit);
        leaderIntakePivot.getConfigurator().apply(currentLimit);

        // Left/Leader Pivot Config
        pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kS = 0.45;
        pivotConfig.Slot0.kV = 0.12;
        pivotConfig.Slot0.kA = 0.02069;
        pivotConfig.Slot0.kP = 7;
        pivotConfig.Slot0.kI = 0;
        pivotConfig.Slot0.kD = 0.1;
        pivotConfig.MotionMagic.MotionMagicAcceleration = 2;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 96.667*0.5;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.34;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        pivotConfig.Feedback.SensorToMechanismRatio = 25; // Intake pivot ratio is 25:1
        leaderIntakePivot.getConfigurator().apply(pivotConfig);

        // Intake retracted for start, set position to 0.34 rotations, around 130 deg.
        leaderIntakePivot.setPosition(0.34);

        // Motor Telemetry
        pivotPosition = leaderIntakePivot.getPosition();
        pivotTorqueCurrent = leaderIntakePivot.getTorqueCurrent();
        pivotLeaderTempC = leaderIntakePivot.getDeviceTemp();
        rollerVelocity = intake.getVelocity();
        rollerAppliedVoltage = intake.getMotorVoltage();
        rollerTempC = intake.getDeviceTemp();
        followerIntakePivot.setControl(followerPivotRequest);
    }

    public void intake() {
        intake.setControl(intakeRequest.withOutput(INTAKE_ROLLER_VOLTAGE));
    }
    public void eject() {
        intake.setControl(intakeRequest.withOutput(-INTAKE_ROLLER_VOLTAGE));
    }
    public void stop() {
        intake.setControl(neutral);
    }
    public void pivot(double radians) {
        leaderIntakePivot.setControl(leaderPivotRequest.withPosition(radians));
        followerIntakePivot.setControl(followerPivotRequest);
    }
    public boolean getForwardLimit() {
        return forwardLimit.get();
    }
    public boolean getBeamBreak() {
        return beamBreak.get();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(pivotPosition, pivotTorqueCurrent, pivotLeaderTempC, rollerVelocity, rollerAppliedVoltage, rollerTempC);

        SmartDashboard.putNumber("Intake/Pivot Position", pivotPosition.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Torque Current", pivotTorqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Left Motor TempC", pivotLeaderTempC.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Velocity", rollerVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Output Voltage", rollerAppliedVoltage.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Motor TempC", rollerTempC.getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Forward Limit", forwardLimit.get());
        SmartDashboard.putBoolean("Intake/Has Note", beamBreak.get());
    }
}