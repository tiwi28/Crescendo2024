package frc.robot.subsystems.Lights;

//imports
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
  
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  public int[] colors = new int[] { 0, 0, 0 };

  public Lights() {
    led = new AddressableLED(Constants.LightsConstants.port);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    buffer = new AddressableLEDBuffer(Constants.LightsConstants.length);
    led.setLength(buffer.getLength());

    // Set the data
    led.setData(buffer);
    led.start();
  }

  public void clearBuffer() {
    colors = Constants.LightsConstants.Colors.GOLD;
  }

  @Override
  public void periodic() {
     // Sets the specified LED to the RGB values stated 
    for (int i = 0; i < Constants.LightsConstants.length; i++) {
      setRGB(i, colors[0], colors[1], colors[2]);
    }
    sendBuffer();
    logValues();
  }

  private void logValues() {
  }

  public void sendBuffer() {
    led.setData(buffer);
  }

  //Sets LED color using RGB values 
  public void setRGB(int index, int r, int g, int b) {
    buffer.setRGB(index, g, r, b);
  }

  //Sets LED color using HSV values
  public void setHSV(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
  }
}
