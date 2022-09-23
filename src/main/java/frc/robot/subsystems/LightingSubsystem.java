package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightingSubsystem extends SubsystemBase {

    private CANdle candle;
    private CANdleConfiguration config;

    public LightingSubsystem() {
        candle = new CANdle(21, "canivore");
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }
    public void lightUpGreen() {
        candle.setLEDs(0, 255, 0, 0, 0, 128);
    }
    public void lightUpRed() {
        candle.setLEDs(255, 0, 0, 0, 0, 128);
    }
    public void lightUpYellow() {
        candle.setLEDs(219, 103, 0, 0, 0, 128);
    }
}