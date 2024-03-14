// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1); // Subsystem controller
  // Subsystems
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();

  private final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Robot centric driving for the hallways
  
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> fieldDrive.withVelocityX((-joystick.getLeftY() * MaxSpeed) * 0.5) // Drive forward with negative Y (forward)
            .withVelocityY((-joystick.getLeftX() * MaxSpeed) * 0.5) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Climber
    operator.povUp().onTrue(climber.extend()).onFalse(climber.stop());
    operator.povDown().onTrue(climber.retract()).onFalse(climber.stop());

    //Intake
    // intake.setDefaultCommand(intake.pivot(MathUtil.applyDeadband(operator.getRawAxis(1), 0.1)));

    /* 
      new Trigger(() -> Math.abs(operator.getRawAxis(1)) > 0.1)
      .whileTrue(new InstantCommand(() -> intake.pivot(operator.getLeftY()/2))
    );

    new Trigger(() -> Math.abs(operator.getRawAxis(1)) <= 0.1)
      .whileTrue(new InstantCommand(() -> intake.pivot(0))
    ); 
    */
    
    operator.a().onTrue(intake.chomp()).onFalse(intake.stop());
    operator.y().onTrue(intake.vomit()).onFalse(intake.stop());

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return drivetrain.applyRequest(() -> fieldDrive.withVelocityY(-0.5 * MaxSpeed)).withTimeout(3);
  }
}
