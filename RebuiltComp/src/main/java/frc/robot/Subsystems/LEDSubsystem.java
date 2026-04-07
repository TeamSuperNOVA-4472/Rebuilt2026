package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class LEDSubsystem extends SubsystemBase
{
    //Assigning variables
    private int mLEDCount = Constants.LEDSubsystemConstants.kLEDCount;
    private int mPort = Constants.LEDSubsystemConstants.kKWMPort;
   
    //Defining LEDs & LEDBuffers 
    private AddressableLED mLight = new AddressableLED(mPort);
    private AddressableLEDBuffer mBuffer = new AddressableLEDBuffer(mLEDCount);

    //Assigning names to portions of LED strip
    private AddressableLEDBufferView exampleBufferView = mBuffer.createView(10, 16);
    //TO-DO: Create different buffer views according to the actual placement of LEDs on the robot.

    private boolean isActive;
    private LEDPattern currentPattern;
    LEDPattern defaultPattern = LEDPattern.solid(Color.kWhite);



    //public static kLEDSubsystem = new LEDSubsystem();

    
    private LEDSubsystem()
    {
        mLight.setLength(mBuffer.getLength());
        mLight.setData(mBuffer);
        mLight.start();

        //setDefaultCommand goes here
    }

    @Override
    public void periodic() 
    {
      if (!isActive)
      {
        //Applying the default pattern (all white) to the current whenever the robot is disabled. 
        currentPattern = defaultPattern;
      }
      currentPattern.applyTo(mBuffer);
      mLight.setData(mBuffer);
  }
  
  public void enableLEDS() {
    isActive = true;
  }
  
  public void disableLEDS() 
  {
    isActive = false; 
  }
  
  public void setPattern(LEDPattern pattern) 
  {
    currentPattern = pattern;
  }
  
    //LED Patterns go here
  public LEDPattern animAuton()
  {
    return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kBlack);    
  }
  
  public LEDPattern animRedAllianceEnabled()
  {
    return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kDarkRed, Color.kMistyRose);
  }

  public LEDPattern animBlueAllianceEnabled()
  {
    return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRoyalBlue, Color.kBlue);
  }

  public LEDPattern animDisabled()
  {
    return (LEDPattern.solid(Color.kDarkRed)).blink(Seconds.of(1.5));
  }

  /* TO-DO: Find variables for climb height and insert here
  public LEDPattern animClimb()
  {
    return LEDPattern.progressMaskLayer(() -> current height of climb / total height);
  }*/

  public LEDPattern animShooting()
  {
    return (LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed, Color.kDarkMagenta, Color.kBlue));
  }

  
    
}
