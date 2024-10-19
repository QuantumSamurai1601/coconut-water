// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PrepareShooterLower extends Command {
  private final ShooterSubsystem shooter;
  private final LEDSubsystem led;
  /** Creates a new PivotLower. */
  public PrepareShooterLower(ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
    shooter = shooterSubsystem;
    led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.pivot(0.145);
    Commands.waitSeconds(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.flywheelsAtTarget() && shooter.pivotAtTarget()) {led.rainbow();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
