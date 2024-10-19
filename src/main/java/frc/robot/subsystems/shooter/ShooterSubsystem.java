package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    // Shooter and Feeder Motors
    private final TalonFX topShooter;
    private final TalonFX bottomShooter;
    private final TalonFX feeder;
    // Shooter Pivot Motors
    private final TalonFX leaderPivot;
    private final TalonFX followerPivot;
    // Shooter and Feeder Control Requests
    private final VelocityVoltage topShooterRequest;
    private final VelocityVoltage bottomShooterRequest;
    private final VoltageOut feederRequest;
    // Shooter Pivot Control Requests
    private final MotionMagicVoltage leaderPivotRequest;
    private final Follower followerPivotRequest;
    // Leader Configs
    private final TalonFXConfiguration currentLimit;
    // Neutral request, determined by NeutralModeValue of motor
    private final NeutralOut neutral;
    // Motor Telemetry
    private final StatusSignal<Double> shooterTopVelocity;
    private final StatusSignal<Double> shooterTopSupplyCurrnet;
    private final StatusSignal<Double> shooterTopTempC;
    private final StatusSignal<Double> shooterBottomVelocity;
    private final StatusSignal<Double> shooterBottomSupplyCurrent;
    private final StatusSignal<Double> shooterBottomTempC;

    private final StatusSignal<Double> pivotLeaderPosition;
    private final StatusSignal<Double> pivotLeaderSupplyCurrent;
    private final StatusSignal<Double> pivotLeaderTempC;
    private final StatusSignal<Double> pivotFollowerPosition;
    private final StatusSignal<Double> pivotFollowerSupplyCurrent;
    private final StatusSignal<Double> pivotFollowerTempC;

    public ShooterSubsystem() {
        // Shooter Motors and Control Requests
        topShooter = new TalonFX(TOP_SHOOTER_ID, CANBUS_NAME);
        bottomShooter = new TalonFX(BOTTOM_SHOOTER_ID, CANBUS_NAME);
        topShooterRequest =  new VelocityVoltage(0.0);
        bottomShooterRequest = new VelocityVoltage(0.0).withSlot(0);

        // Feeder Motor and Control Request
        feeder = new TalonFX(FEEDER_ID, CANBUS_NAME);
        feederRequest = new VoltageOut(0.0);
        feeder.setNeutralMode(NeutralModeValue.Brake);
        feeder.setInverted(true);

        // Shooter Pivot Motors and Control Requests
        leaderPivot = new TalonFX(LEFT_SHOOTER_PIVOT_ID, CANBUS_NAME);
        followerPivot = new TalonFX(RIGHT_SHOOTER_PIVOT_ID, CANBUS_NAME);
        leaderPivotRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);
        followerPivotRequest = new Follower(LEFT_SHOOTER_PIVOT_ID, true);

        // Neutral Request
        neutral = new NeutralOut();

        // Current Limit Config
        currentLimit = new TalonFXConfiguration();
        currentLimit.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        currentLimit.CurrentLimits.SupplyCurrentThreshold = 1.5;
        currentLimit.CurrentLimits.SupplyCurrentLimitEnable = true;
        topShooter.getConfigurator().apply(currentLimit);
        bottomShooter.getConfigurator().apply(currentLimit);
        leaderPivot.getConfigurator().apply(currentLimit);

        // Apply top shooter, bottom shooter, and pivot config
        topShooter.getConfigurator().apply(topShooterConfig);
        bottomShooter.getConfigurator().apply(bottomShooterConfig);
        leaderPivot.getConfigurator().apply(pivotConfig);

        // Set position to 0 for start, full down
        leaderPivot.setPosition(0);

        // Motor Telemetry
        shooterTopVelocity = topShooter.getVelocity();
        shooterTopSupplyCurrnet = topShooter.getSupplyCurrent();
        shooterTopTempC = topShooter.getDeviceTemp();
        shooterBottomVelocity = bottomShooter.getVelocity();
        shooterBottomSupplyCurrent = bottomShooter.getSupplyCurrent();
        shooterBottomTempC = bottomShooter.getDeviceTemp();

        pivotLeaderPosition = leaderPivot.getPosition();
        pivotLeaderSupplyCurrent = leaderPivot.getSupplyCurrent();
        pivotLeaderTempC = leaderPivot.getDeviceTemp();
        pivotFollowerPosition = followerPivot.getPosition();
        pivotFollowerSupplyCurrent = followerPivot.getSupplyCurrent();
        pivotFollowerTempC = followerPivot.getDeviceTemp();

        followerPivot.setControl(followerPivotRequest);
    }
    public void shootVelocity(double topRps, double bottomRps) {
        topShooter.setControl(topShooterRequest.withVelocity(topRps));
        bottomShooter.setControl(bottomShooterRequest.withVelocity(bottomRps));
    }
    public void devPivotUp() {
        var curAng = leaderPivot.getPosition().getValueAsDouble();
        leaderPivot.setControl(leaderPivotRequest.withPosition(curAng + 0.01));
    }
    public void devPivotDown() {
        var curAng = leaderPivot.getPosition().getValueAsDouble();
        leaderPivot.setControl(leaderPivotRequest.withPosition(curAng - 0.01));
    }
    public void shootVoltage(double voltage) {
        topShooter.setControl(new VoltageOut(voltage));
        bottomShooter.setControl(new VoltageOut(voltage));
    }
    public void shooterIntake() {
        topShooter.setControl(new VoltageOut(2.7));
        bottomShooter.setControl(new VoltageOut(2.7));
    }
    public void feedFromSource() {
        feeder.setControl(feederRequest.withOutput(-0.5));
    }
    public void feedFromIntake() {
        feeder.setControl(feederRequest.withOutput(-5.5));
    }
    public void feedToShooter() {
        feeder.setControl(feederRequest.withOutput(-7.5));
    }
    public void feedStop() {
        feeder.setControl(neutral);
    }
    public void pivot(double angle) {
        leaderPivot.setControl(leaderPivotRequest.withPosition(angle));
    }
    public void idle() {
        topShooter.setControl(topShooterRequest.withVelocity(10));
        bottomShooter.setControl(topShooterRequest.withVelocity(10));
    }
    public void stop() {
        topShooter.setControl(neutral);
        bottomShooter.setControl(neutral);
    }
    public double getShootingAngle(double distance) {
        return shooterTreeMap.get(distance);
    }
    
    // public boolean flywheelsAtTarget() {
    //     return Math.abs(topShooter.getVelocity().getValueAsDouble())>= 8
    //         && topShooter.getVelocity().getValueAsDouble() > -80
    //         && bottomShooter.getVelocity().getValueAsDouble() > -80;
    // }

    public boolean flywheelsAtTarget() {
        return topShooter.getClosedLoopError().refresh().getValueAsDouble() < 5
            && bottomShooter.getClosedLoopError().refresh().getValueAsDouble() < 5;
    }
    public boolean pivotAtTarget() {
        return Math.abs(leaderPivot.getClosedLoopReference().refresh().getValueAsDouble() - pivotLeaderPosition.refresh().getValueAsDouble()) < 0.03;
    }
    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(shooterTopVelocity, shooterTopSupplyCurrnet, shooterTopTempC, shooterBottomVelocity, shooterBottomSupplyCurrent, shooterBottomTempC, pivotLeaderPosition, pivotLeaderSupplyCurrent, pivotLeaderTempC, pivotFollowerPosition, pivotFollowerSupplyCurrent, pivotFollowerTempC);

        SmartDashboard.putNumber("Shooter/Top Velocity", shooterTopVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Bottom Velocity", shooterBottomVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Top Supply Current", shooterTopSupplyCurrnet.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Bottom Supply Current", shooterBottomSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Top Temp C", shooterTopTempC.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Bottom Temp C", shooterBottomTempC.getValueAsDouble());
        
        SmartDashboard.putBoolean("Shooter/At Target Speeds", this.flywheelsAtTarget());
        SmartDashboard.putNumber("Shooter/Left Pivot Position", pivotLeaderPosition.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Position", pivotFollowerPosition.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Pivot Supply Current", pivotLeaderSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Supply Current", pivotFollowerSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Pivot Temp C", pivotLeaderTempC.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Pivot Temp C", pivotFollowerTempC.getValueAsDouble());
    }
}
