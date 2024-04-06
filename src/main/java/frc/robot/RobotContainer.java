// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autos.PrepareShooter;
import frc.robot.commands.autos.ShootNote;
import frc.robot.commands.autos.SpoolShooter;
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
  private final CommandXboxController dev = new CommandXboxController(2);
  // Subsystems
  private final VisionSubsystem vision = new VisionSubsystem();
  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants, vision, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight); // My drivetrain
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final LEDSubsystem led = new LEDSubsystem(intake, shooter);
  
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
  private final Command nothing = Commands.none();

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> fieldDrive.withVelocityX((-joystick.getLeftY() * MaxSpeed) * 1) // Drive forward with negative Y (forward)
          .withVelocityY((-joystick.getLeftX() * MaxSpeed) * 1) // Drive left with negative X (left)
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));
    // Driver drive half speed
    joystick.rightBumper().whileTrue(
      drivetrain.applyRequest(() -> fieldDrive.withVelocityX((-joystick.getLeftY() * MaxSpeed) * 0.5) // Drive forward with negative Y (forward)
            .withVelocityY((-joystick.getLeftX() * MaxSpeed) * 0.5) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
    );

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on b press
    joystick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Driver feed shooter
    joystick.x().whileTrue(
      Commands.startEnd(() -> shooter.feedToShooter(),
      () -> {
        shooter.feedStop();
        Commands.waitSeconds(0.5);
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
    // Operator run shooter default
    operator.b().whileTrue(new SpoolShooter(shooter, led))
      .whileFalse(
      Commands.runOnce(() -> shooter.stop()));
    // Operator pivot sub base
    operator.y().onTrue(
      Commands.runOnce(() -> shooter.pivot(0.185))
    );
    // Operator pivot 6 feet
    operator.a().onTrue(
      Commands.runOnce(() -> shooter.pivot(0.115))
    );
    // Operator intake sequence
    operator.leftTrigger().whileTrue(
      new IntakeGround(intake, shooter, led)
    );
    // Operator intake retract/eject
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
    
    // Dev controls
    dev.povUp().onTrue(
      Commands.runOnce(() -> shooter.devPivotUp())
    );
    dev.povDown().onTrue(
      Commands.runOnce(() -> shooter.devPivotDown())
    );
    dev.x().whileTrue(
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
    joystick.y().whileTrue(
      AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("AlignToScore"), new PathConstraints(3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720)))
    );
    dev.b().onTrue(
      Commands.runOnce(() -> {
        shooter.pivot(0);
        intake.pivotUp();
      })
    );
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("intakeGround", new IntakeGround(intake, shooter, led));
    NamedCommands.registerCommand("prepareShooter", new PrepareShooter(shooter, led, drivetrain.getState().Pose::getX).withTimeout(1));
    NamedCommands.registerCommand("spoolShooter", new SpoolShooter(shooter, led).withTimeout(1.2));
    NamedCommands.registerCommand("shootNote", new ShootNote(intake, shooter, led).withTimeout(2));

    autoChooser.setDefaultOption("Autonomous Disabled", nothing);
    autoChooser.addOption("Mobility Auto", mobilityAuto); 
    autoChooser.addOption("Test P", new PathPlannerAuto("TP"));
    autoChooser.addOption("2 Note Left A Mid", new PathPlannerAuto("2 - (L) A ~ 2.93"));
    autoChooser.addOption("2 Note Left A Left" , new PathPlannerAuto("2 - (L) A ~ 1.50"));
    autoChooser.addOption("2 Note Front B", new PathPlannerAuto("2 - (Fr) B ~ 0.98"));
    autoChooser.addOption("3 Note Front BF", new PathPlannerAuto("3 - (Fr) BF ~ 6.20"));
    autoChooser.addOption("3 Note Right CF", new PathPlannerAuto("3 - (R) CF ~ 6.74"));
    autoChooser.addOption("4 Note Right CBA", new PathPlannerAuto("4 - (R) CBA ~ 4.35"));
    autoChooser.addOption("4 Note Right CBF", new PathPlannerAuto("4 - (R) CBF ~ 8.60"));
    autoChooser.addOption("5 Note Left ABCF", new PathPlannerAuto("5 - (L) ABCF ~ 10.20"));
    autoChooser.addOption("5 Note Left BCEF", new PathPlannerAuto("5 - (L) BCEF ~ 13.96"));
    autoChooser.addOption("6 Note Left ABCFE", new PathPlannerAuto("6 - (L) ABCFE ~ 15.27"));
    autoChooser.addOption("6 Note Front ABCGF", new PathPlannerAuto("6 - (Fr) ABCGF ~ 14.59"));
    autoChooser.addOption("2 Note Right H", new PathPlannerAuto("2 - (R) H ~ 6.77"));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    drivetrain.applyCurrentLimit(0);
    drivetrain.applyCurrentLimit(1);
    drivetrain.applyCurrentLimit(2);
    drivetrain.applyCurrentLimit(3);
    PortForwarder.add(5800, "photonvision.local", 5800);

    PathfindingCommand.warmupCommand().schedule();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
