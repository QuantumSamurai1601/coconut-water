package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle;

    public LEDSubsystem() {
        candle = new CANdle(1, "Drivetrain");
        candle.configLEDType(LEDStripType.GRB);
        candle.configBrightnessScalar(1);
        candle.setLEDs(0, 0, 255);
    }
    
}
