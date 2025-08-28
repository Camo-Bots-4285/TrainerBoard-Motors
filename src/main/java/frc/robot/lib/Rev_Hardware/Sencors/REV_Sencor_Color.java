package frc.robot.lib.Rev_Hardware.Sencors;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.REV_Motor_Charactoristics;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class REV_Sencor_Color extends SubsystemBase{

/**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
//   private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
//   private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
//   private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
//   private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  private int array_length = 0;

  private Color[] Color_Idenification;

  private String[] Color_String;

    public REV_Sencor_Color(
        Color[] Color_Idenification,
        String[] Color_String
    ){
        this.Color_Idenification=Color_Idenification;
        this.Color_String=Color_String;

       array_length =Color_Idenification.length;

       /*Creates each color to look for */
        for (int i = 0; i < array_length; ++i) {
            m_colorMatcher.addColorMatch(Color_Idenification[i]);
        }

    }
    

public Color getSencorReadings(){
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    return m_colorSensor.getColor();
}

public String getCalculatedColor_String(){
    /**
     * Run the color match algorithm on our detected color
     */
    ColorMatchResult match = m_colorMatcher.matchClosestColor(getSencorReadings());

    String Color="Unkown";

    //If color is ditected define Color to no longer be unkown
    for (int i = 0; i < array_length; ++i) {

    if (match.color == Color_Idenification[i]) {
      Color = Color_String[i];
    } 
    }

    return Color;
}

public int getCalculatedColor_Index(){
    /**
     * Run the color match algorithm on our detected color
     */
    ColorMatchResult match = m_colorMatcher.matchClosestColor(getSencorReadings());

    int Color = -1;

    //If color is ditected define Color to no longer be unkown
    for (int i = 0; i < array_length; ++i) {

    if (match.color == Color_Idenification[i]) {
      Color = i;
    } 
    }

    return Color;
}

public double getConfidence(){
     /**
     * Run the color match algorithm on our detected color
     */
    ColorMatchResult match = m_colorMatcher.matchClosestColor(getSencorReadings());

    return match.confidence;
}

public double getRedReading(){
    return getSencorReadings().red;
}

public double getGreenReading(){
    return getSencorReadings().green;
}

public double getBlueReading(){
    return getSencorReadings().blue;
}

  
}
