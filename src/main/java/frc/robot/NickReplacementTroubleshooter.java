package frc.robot;

/**
 * Use this class to troubleshoot the robot when Nick is not available.
 */
public class NickReplacementTroubleshooter {

  /**
   * If claw intake speed is too fast, set this to a lower value.
   */
public static  double CLAW_SPEED = .9;

/**
 * Feedforward values used to compensate for gravity's effect on the arm<br>
 * Before adjusting these values make sure that the Encoder has been zeroed in the horizontal and tucked position<br>
 * <br>Also make sure that the battery is fully charged
 *
 *
 *
 *  <br> If The wrist appears to be sagging at any point during the match without any human input make usre
 */

public static double FEED_FORWARD_WRIST = 0.07;
public static double FEED_FORWARD_ARM = 0.09;
public static double FEED_FORWARD_ARM_EXTENDED =  0.13;


}
