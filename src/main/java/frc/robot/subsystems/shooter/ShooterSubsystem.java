package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

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
    // Shooter and Feeder Control Requests
    private final VelocityTorqueCurrentFOC leaderShooterRequest;
    private final Follower followerShooterRequest;
    private final VelocityTorqueCurrentFOC feederRequest;
    // Shooter Pivot Control Requests
    private final PositionTorqueCurrentFOC leaderPivotRequest;
    private final Follower followerPivotRequest;
    // Leader Configs
    private final TalonFXConfiguration shooterConfig;
    private final TalonFXConfiguration pivotConfig;
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
        leaderShooterRequest =  new VelocityTorqueCurrentFOC(0.0);
        followerShooterRequest = new Follower(BOTTOM_SHOOTER_ID, false);

        feeder = new TalonFX(FEEDER_ID, CANBUS_NAME);
        feederRequest = new VelocityTorqueCurrentFOC(0.0);

        leaderPivot = new TalonFX(LEFT_SHOOTER_PIVOT_ID, CANBUS_NAME);
        followerPivot = new TalonFX(RIGHT_SHOOTER_PIVOT_ID, CANBUS_NAME);
        leaderPivotRequest = new PositionTorqueCurrentFOC(0.0);
        followerPivotRequest = new Follower(LEFT_SHOOTER_PIVOT_ID, true);

        neutral = new NeutralOut();

        shooterConfig = new TalonFXConfiguration();
        shooterConfig.Slot0.kS = 0.0;
        shooterConfig.Slot0.kV = 0.0;
        shooterConfig.Slot0.kP = 0.0;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 60;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderShooter.getConfigurator().apply(shooterConfig);

        pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = 0.0;
        pivotConfig.Slot0.kI = 0.0;
        pivotConfig.Slot0.kD = 0.0;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Feedback.SensorToMechanismRatio = 133.33;
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

        followerPivot.setControl(followerPivotRequest);
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
        
        SmartDashboard.putNumber("Shooter/Left Pivot Position", pivotLeaderPosition.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Position", pivotFollowerPosition.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Pivot Torque Current", pivotLeaderTorqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Torque Current", pivotFollowerTorqueCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Pivot Temp C", pivotLeaderTempC.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Temp C", pivotFollowerTempC.getValueAsDouble());
    }
}
