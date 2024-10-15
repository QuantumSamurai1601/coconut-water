package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    // Intake Motor
    private final TalonFX intake;
    // Intake Pivot Motors and Sensors
    private final TalonFX leaderIntakePivot;
    private final TalonFX followerIntakePivot;
    private final DigitalInput beamBreak;
    // Intake Control Requests
    private final VelocityTorqueCurrentFOC intakeRequest;
    // Intake Pivot Control Requests
    private final MotionMagicVoltage leaderPivotRequest;
    private final Follower followerPivotRequest;
    // Neutral request, determined by NeutralModeValue of motor
    private final NeutralOut neutral;
    // Motor Telemetry
    private final StatusSignal<Double> rollerVelocity;
    private final StatusSignal<Double> rollerMotorVoltage;
    private final StatusSignal<Double> rollerTempC;
    private final StatusSignal<Double> pivotPosition;
    private final StatusSignal<Double> pivotSupplyCurrent;
    private final StatusSignal<Double> pivotLeaderTempC;
    // Current limit config
    private final TalonFXConfiguration currentLimit;

    public IntakeSubsystem() {
        //Intake Rollers
        intake = new TalonFX(INTAKE_ID, CANBUS_NAME);
        beamBreak = new DigitalInput(INTAKE_BEAM_BREAK);
        intakeRequest = new VelocityTorqueCurrentFOC(0).withSlot(1);
        intake.setInverted(true);
                
        // Intake Pivot
        leaderIntakePivot = new TalonFX(LEFT_INTAKE_PIVOT_ID, CANBUS_NAME);
        followerIntakePivot =  new TalonFX(RIGHT_INTAKE_PIVOT_ID, CANBUS_NAME);
        leaderPivotRequest = new MotionMagicVoltage(0).withEnableFOC(true);
        followerPivotRequest = new Follower(LEFT_INTAKE_PIVOT_ID, true);

        // Neutral Request
        neutral = new NeutralOut();

        // Current Limit Config
        currentLimit = new TalonFXConfiguration();
        currentLimit.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        currentLimit.CurrentLimits.SupplyCurrentLimitEnable = true;
        // intake.getConfigurator().apply(currentLimit);
        leaderIntakePivot.getConfigurator().apply(currentLimit);

        // Apply pivot and roller config
        leaderIntakePivot.getConfigurator().apply(pivotConfig);
        intake.getConfigurator().apply(rollerConfig);

        // Intake retracted for start, set position to 0.34 rotations
        leaderIntakePivot.setPosition(0.345);

        // Motor Telemetry
        pivotPosition = leaderIntakePivot.getPosition();
        pivotSupplyCurrent = leaderIntakePivot.getSupplyCurrent();
        pivotLeaderTempC = leaderIntakePivot.getDeviceTemp();
        rollerVelocity = intake.getVelocity();
        rollerMotorVoltage = intake.getMotorVoltage();
        rollerTempC = intake.getDeviceTemp();
        followerIntakePivot.setControl(followerPivotRequest);
    }

    public void intake() {
        intake.setControl(intakeRequest.withVelocity(-8));
    }
    public void eject() {
        intake.setControl(intakeRequest.withVelocity(9));
    }
    public void stop() {
        intake.setControl(neutral);
    }
    public void pivotUp() {
        leaderIntakePivot.setControl(leaderPivotRequest.withPosition(0.33));
        followerIntakePivot.setControl(followerPivotRequest);
    }
    public void pivotDown() {
        leaderIntakePivot.setControl(leaderPivotRequest.withPosition(0));
        followerIntakePivot.setControl(followerPivotRequest);
    }
    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(pivotPosition, pivotSupplyCurrent, pivotLeaderTempC, rollerVelocity, rollerMotorVoltage, rollerTempC);

        SmartDashboard.putNumber("Intake/Pivot Position", pivotPosition.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Supply Current", pivotSupplyCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Pivot Left Motor TempC", pivotLeaderTempC.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Velocity", rollerVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Output Voltage", rollerMotorVoltage.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Roller Motor TempC", rollerTempC.getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Has Note", !beamBreak.get());
    }
}