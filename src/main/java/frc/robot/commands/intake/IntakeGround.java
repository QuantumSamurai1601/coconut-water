package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;

public class IntakeGround extends Command {
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final LEDSubsystem led;

    public IntakeGround(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem) {
        intake = intakeSubsystem;
        shooter = shooterSubsystem;
        led = ledSubsystem;
        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        shooter.pivot(0.005);
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
            led.greenTwinkleToes();
            shooter.pivot(0.1);
        }
    }

    @Override
    public boolean isFinished() {
        return intake.getBeamBreak();
    }
    
}
