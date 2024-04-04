// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootNote extends Command {
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final LEDSubsystem led;
  /** Creates a new shootNote. */
  public ShootNote(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
    intake = intakeSubsystem;
    shooter = shooterSubsystem;
    led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.shootingNote();
    shooter.feedToShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.feedStop();
    if (!intake.getBeamBreak()) {
      led.noNote();
    } else {
      led.greenTwinkleToes();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getBeamBreak();
  }
}
