// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autos.PrepareShooter;
import frc.robot.commands.autos.ShootNote;
import frc.robot.commands.intake.IntakeGround;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.*;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1); // Subsystem controller
  // Subsystems
  private final VisionSubsystem vision = new VisionSubsystem();
  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants, vision, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight); // My drivetrain
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final LEDSubsystem led = new LEDSubsystem();
  
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

  private Command runAuto = drivetrain.getAutoPath("Tests");

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

    // reset the field-centric heading on b press
    joystick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Driver feed shooter
    joystick.x().whileTrue(
      Commands.startEnd(() -> shooter.feedToShooter(),
      () -> {
        shooter.feedStop();
        if (!intake.getBeamBreak()) {
          led.noNote();
        } else {
          led.greenTwinkleToes();
        } 
      })
    );

    // Operator climber
    climber.setDefaultCommand(
      climber.control(operator::getLeftY, operator::getRightY)
    );

    // Operator source intake
    operator.x().whileTrue(
      Commands.startEnd(() -> {
        shooter.shooterIntake();
        shooter.feedFromSource();
      },
      () -> {
        shooter.stop();
        shooter.feedStop();
      })
    );
    // Operator run shooter medium preset
    operator.leftBumper().whileTrue(
      Commands.startEnd(() -> shooter.shootVoltage(-20), () -> shooter.stop())   
    );
    // Operator run shooter high preset
    operator.rightBumper().whileTrue(
      Commands.startEnd(() -> {
        shooter.shootVelocity(-45);
        if (shooter.flywheelsAtTarget()) {
          led.shooterRunway();
        }
      },
      () -> {
        shooter.stop();
      })    
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
      Commands.runOnce(() -> shooter.pivot(0.08))
    );
    //operator.x().onTrue(
    //  Commands.runOnce(() -> shooter.pivot(0.0))
    //);

    // Operator intake sequence
    operator.leftTrigger().whileTrue(
      new IntakeGround(intake, shooter, led)
    );
    // Operator intake retract
    operator.rightTrigger().whileTrue(
      Commands.startEnd(() -> {
        intake.pivotUp();
        intake.eject();
      },
      () -> {
        intake.pivotUp();
        intake.stop();
      })
    );

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("intakeGround", new IntakeGround(intake, shooter, led).withTimeout(1));
    NamedCommands.registerCommand("prepareShooter", new PrepareShooter(shooter, led).until(shooter::flywheelsAtTarget));
    NamedCommands.registerCommand("shootNote", new ShootNote(intake, shooter, led).withTimeout(3));

    autoChooser.setDefaultOption("Autonomous Disabled", nothing);
    autoChooser.addOption("Mobility Auto", mobilityAuto);
    autoChooser.addOption("2 Note Left A", new PathPlannerAuto("2 - (L) A ~ 2.93"));
    Shuffleboard.getTab("SmartDashboard").add("Auto Chooser", autoChooser);

    drivetrain.applyCurrentLimit(0);
    drivetrain.applyCurrentLimit(1);
    drivetrain.applyCurrentLimit(2);
    drivetrain.applyCurrentLimit(3);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
