package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    // Shooter and Feeder Motors
    private final TalonFX leaderShooter;
    private final TalonFX followerShooter;
    private final TalonFX feeder;
    // Shooter Pivot Motors
    private final TalonFX leaderPivot;
    private final TalonFX followerPivot;
    private final DigitalInput forwardLimit;
    // Shooter and Feeder Control Requests
    private final VelocityVoltage leaderShooterRequest;
    private final Follower followerShooterRequest;
    private final VoltageOut feederRequest;
    private final Follower tempFeederFollower;
    // Shooter Pivot Control Requests
    private final MotionMagicVoltage leaderPivotRequest;
    private final Follower followerPivotRequest;
    // Leader Configs
    private final TalonFXConfiguration shooterConfig;
    private final TalonFXConfiguration pivotConfig;
    private final TalonFXConfiguration currentLimit;
    // Neutral request, determined by NeutralModeValue of motor
    private final NeutralOut neutral;
    // Motor Telemetry
    private final StatusSignal<Double> shooterLeaderVelocity;
    private final StatusSignal<Double> shooterLeaderTorqueCurrent;
    private final StatusSignal<Double> shooterLeaderTempC;
    private final StatusSignal<Double> shooterFollowerVelocity;
    private final StatusSignal<Double> shooterFollowerTorqueCurrent;
    private final StatusSignal<Double> shooterFollowerTempC;

    private final StatusSignal<Double> pivotLeaderPosition;
    private final StatusSignal<Double> pivotLeaderTorqueCurrent;
    private final StatusSignal<Double> pivotLeaderTempC;
    private final StatusSignal<Double> pivotFollowerPosition;
    private final StatusSignal<Double> pivotFollowerTorqueCurrent;
    private final StatusSignal<Double> pivotFollowerTempC;

    public ShooterSubsystem() {
        leaderShooter = new TalonFX(TOP_SHOOTER_ID, CANBUS_NAME);
        followerShooter = new TalonFX(BOTTOM_SHOOTER_ID, CANBUS_NAME);
        leaderShooterRequest =  new VelocityVoltage(0.0);
        followerShooterRequest = new Follower(TOP_SHOOTER_ID, false);

        feeder = new TalonFX(FEEDER_ID, CANBUS_NAME);
        feederRequest = new VoltageOut(0.0);
        feeder.setNeutralMode(NeutralModeValue.Brake);
        tempFeederFollower = new Follower(INTAKE_ID, false);

        forwardLimit = new DigitalInput(3);
        leaderPivot = new TalonFX(LEFT_SHOOTER_PIVOT_ID, CANBUS_NAME);
        followerPivot = new TalonFX(RIGHT_SHOOTER_PIVOT_ID, CANBUS_NAME);
        leaderPivotRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
        followerPivotRequest = new Follower(LEFT_SHOOTER_PIVOT_ID, true);

        neutral = new NeutralOut();

        currentLimit = new TalonFXConfiguration();
        currentLimit.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        currentLimit.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderShooter.getConfigurator().apply(currentLimit);
        leaderPivot.getConfigurator().apply(currentLimit);

        shooterConfig = new TalonFXConfiguration();
        shooterConfig.Slot0.kS = 0.0;
        shooterConfig.Slot0.kV = 0.0;
        shooterConfig.Slot0.kP = 0.0;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leaderShooter.getConfigurator().apply(shooterConfig);

        pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kG = 0.03;
        pivotConfig.Slot0.kS = 0.32;
        pivotConfig.Slot0.kV = 0.125;
        pivotConfig.Slot0.kA = 0.02069;
        pivotConfig.Slot0.kP = 90.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.1;
        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfig.MotionMagic.MotionMagicAcceleration = 2;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = 96.667*0.85;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Feedback.SensorToMechanismRatio = 133.33;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.181885;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        leaderPivot.getConfigurator().apply(pivotConfig);

        leaderPivot.setPosition(0);

        shooterLeaderVelocity = leaderShooter.getVelocity();
        shooterLeaderTorqueCurrent = leaderShooter.getTorqueCurrent();
        shooterLeaderTempC = leaderShooter.getDeviceTemp();
        shooterFollowerVelocity = followerShooter.getVelocity();
        shooterFollowerTorqueCurrent = followerShooter.getTorqueCurrent();
        shooterFollowerTempC = followerShooter.getDeviceTemp();

        pivotLeaderPosition = leaderPivot.getPosition();
        pivotLeaderTorqueCurrent = leaderPivot.getTorqueCurrent();
        pivotLeaderTempC = leaderPivot.getDeviceTemp();
        pivotFollowerPosition = followerPivot.getPosition();
        pivotFollowerTorqueCurrent = followerPivot.getTorqueCurrent();
        pivotFollowerTempC = followerPivot.getDeviceTemp();

        followerShooter.setControl(followerShooterRequest);
        followerPivot.setControl(followerPivotRequest);
        // feeder.setControl(tempFeederFollower);
    }
    // Method takes rotations per second, standardizing revolutions per minute
    public void shootVelocity(double rps) {
        leaderShooter.setControl(leaderShooterRequest.withVelocity(rps));
        followerShooter.setControl(followerShooterRequest);
    }
    public void shootVoltage(double voltage) {
        leaderShooter.setControl(new VoltageOut(voltage));
        followerShooter.setControl(followerShooterRequest);
    }
    public void shooterIntake() {
        leaderShooter.setControl(new VoltageOut(2.7));
        followerShooter.setControl(followerShooterRequest);
    }
    public void feedFromSource() {
        feeder.setControl(new VoltageOut(0.375));
    }
    public void feedFromIntake() {
        feeder.setControl(feederRequest.withOutput(-1));
    }
    public void feedToShooter() {
        feeder.setControl(new VoltageOut(-3));
    }
    public void feedStop() {
        feeder.setControl(neutral);
    }
    public void pivot(double angle) {
        leaderPivot.setControl(leaderPivotRequest.withPosition(angle));
        followerPivot.setControl(followerPivotRequest);
    }
    public void stop() {
        leaderShooter.setControl(neutral);
        followerShooter.setControl(followerShooterRequest);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(shooterLeaderVelocity, shooterLeaderTorqueCurrent, shooterLeaderTempC, shooterFollowerVelocity, shooterFollowerTorqueCurrent, shooterFollowerTempC, pivotLeaderPosition, pivotLeaderTorqueCurrent, pivotLeaderTempC, pivotFollowerPosition, pivotFollowerTorqueCurrent, pivotFollowerTempC);

        SmartDashboard.putNumber("Shooter/Top Velocity", shooterLeaderVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Bottom Velocity", shooterFollowerVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Top Torque Current", shooterLeaderTorqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Bottom Torque Current", shooterFollowerTorqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Top Temp C", shooterLeaderTempC.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Bottom Temp C", shooterFollowerTempC.getValueAsDouble());
        
        SmartDashboard.putBoolean("Shooter/Pivot Forward Limit", !forwardLimit.get());
        SmartDashboard.putNumber("Shooter/Left Pivot Position", pivotLeaderPosition.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Position", pivotFollowerPosition.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Pivot Torque Current", pivotLeaderTorqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Torque Current", pivotFollowerTorqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Pivot Temp C", pivotLeaderTempC.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Temp C", pivotFollowerTempC.getValueAsDouble());
    }
}
