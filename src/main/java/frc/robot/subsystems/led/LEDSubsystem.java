package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle;

    public LEDSubsystem() {
        candle = new CANdle(1, "Drivetrain");
        candle.configLEDType(LEDStripType.GRB);
        candle.configBrightnessScalar(1);
        candle.setLEDs(0, 0, 255);
    }
    //
    public void rainbow() {
        candle.animate(new RainbowAnimation());
    }
    // Note inside the robot
    public void greenTwinkleToes(){
        candle.animate(new TwinkleOffAnimation(0, 255, 0)); 
    }
    // Spooling shooter wheels
    public void shooterRunway(){ 
        candle.animate(new ColorFlowAnimation(227, 64, 175));
    }
    // Note shooting
    public void shootingNote(){
        candle.animate(new StrobeAnimation(227, 64, 64));
    }
    // No note inside robot
    public void noNote(){
        candle.setLEDs(0, 0, 255);
        
    }
}
        

