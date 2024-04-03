// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PrepareShooter extends Command {
  private final ShooterSubsystem shooter;
  private final LEDSubsystem led;
  /** Creates a new PrepareShooter. */
  public PrepareShooter(ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
    shooter = shooterSubsystem;
    led = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.pivot(0.18);
    shooter.shootVelocity(-40);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.flywheelsAtTarget()) {led.shooterRunway();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
