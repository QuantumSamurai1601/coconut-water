package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeGround extends Command {
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    public IntakeGround(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        intake = intakeSubsystem;
        shooter = shooterSubsystem;
        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        intake.pivotDown();
        shooter.feedFromIntake();
        intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.feedStop();
        intake.stop();
        if (intake.getBeamBreak()) {
            intake.pivotUp();
        }
    }

    @Override
    public boolean isFinished() {
        return intake.getBeamBreak();
    }
    
}
