package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    int ledBufferLength;
    int rainbowFirstPixelHue = 0;

    public LED() {
        led = new AddressableLED(0); // change port
        ledBuffer = new AddressableLEDBuffer(5);
        led.setLength(ledBuffer.getLength());
        ledBufferLength = ledBuffer.getLength();
        led.setData(ledBuffer);
        led.start();
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    public void green() {
        for (int i = 0; i < ledBufferLength; i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
        led.setData(ledBuffer);
    }

    public void red() {
        for (int i = 0; i < ledBufferLength; i++) {
            ledBuffer.setRGB(i, 255, 0, 0);
        }
        led.setData(ledBuffer);
    }
}
