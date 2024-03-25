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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

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
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  // Auto chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Robot centric driving for the hallways
  
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final Command mobilityAuto = drivetrain.applyRequest(() -> fieldDrive.withVelocityY(-0.5 * MaxSpeed)).withTimeout(2);
  private final Command nothing = null;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> fieldDrive.withVelocityX((-joystick.getLeftY() * MaxSpeed) * 1) // Drive forward with negative Y (forward)
            .withVelocityY((-joystick.getLeftX() * MaxSpeed) * 1) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Driver feed shooter
    joystick.x().whileTrue(
      Commands.startEnd(() -> shooter.feedShooter(),() -> shooter.feedStop())
    );

    // Operator climber
    operator.povUp().onTrue(climber.extend()).onFalse(climber.stop());
    operator.povDown().onTrue(climber.retract()).onFalse(climber.stop());

    // Operator source intake
    operator.x().whileTrue(
      Commands.startEnd(() -> {
        shooter.shooterIntake();
        shooter.feed();
      },
      () -> {
        shooter.stop();
        shooter.feedStop();
      })
    );
    // Operator run shooter low preset
    operator.leftBumper().whileTrue(
      Commands.startEnd(() -> shooter.shootVoltage(-2), () -> shooter.stop())   
    );
    // Operator run shooter medium preset
    operator.rightBumper().whileTrue(
      Commands.startEnd(() -> shooter.shootVoltage(-5), () -> shooter.stop())   
    );
    // Operator run shooter high preset
    operator.b().whileTrue(
      Commands.startEnd(() -> shooter.shootVoltage(-7.5), () -> shooter.stop())
    );
    // Operator pivot full up
    operator.y().onTrue(
      Commands.runOnce(() -> shooter.pivot(0.18))
    );
    // Operator pivot half
    operator.a().onTrue(
      Commands.runOnce(() -> shooter.pivot(0.1))
    );
    //operator.x().onTrue(
    //  Commands.runOnce(() -> shooter.pivot(0.0))
    //);
    // Operator run intake
    operator.povRight().whileTrue(
      Commands.startEnd(() -> {
        intake.intake();
        shooter.feedShooter();
      }, 
      () -> {
        intake.stop();
        shooter.feedStop();
      })
    );
    // Operatr pivot intake down
    operator.povLeft().onTrue(
      Commands.runOnce(()-> intake.pivot(0))
    );
    operator.rightStick().onTrue(
      Commands.runOnce(() -> intake.pivot(0.33))
    );
        if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
    autoChooser.setDefaultOption("Autonomous Disabled", nothing);
    autoChooser.addOption("Mobility Auto", mobilityAuto);
    Shuffleboard.getTab("SmartDashboard")
      .add("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
