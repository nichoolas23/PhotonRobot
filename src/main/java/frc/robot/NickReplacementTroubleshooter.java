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
 * <br>Feedforward values used to compensate for gravity's effect on the arm</br>
 * <br>Before adjusting these values make sure that:</br>
 * <li>Both the arm and wrist encoders have been zeroed in the vertical and tucked position</li>
 * <li>The battery is fully charged</li>
 * <br>Increase these values if you notice any sag or increase in the arms position without any input
 * Please keep in mind that this will only compensate for a change in velocity not position
 * So it will not perfectly hold, I simply did not have the time to make it hold a set position, because of
 * how the arm and encoder was set up it was really difficult to make the position accurate.
 *
 */

public static double FEED_FORWARD_WRIST = 0.07;
public static double FEED_FORWARD_ARM = 0.09;
public static double FEED_FORWARD_ARM_EXTENDED =  0.13;

// Slew Limiters effect the ramp rate of the motors (how fast they can change speed) between 0.0 and 1.0
public static double ROTATE_SLEW_LIMITER = 0.95; // this one is not implemented because I didn't like the effects it had
public static double FORWARD_SLEW_LIMITER = .95;
public static double REVERSE_SLEW_LIMITER = .95;



public static double ROTATE_SPEED_COEFFICIENT = 1; // between 0.0 and 1.0 [0.0 = no power, 1.0 = full power]
public static double FORWARD_SPEED_COEFFICIENT = 1; // I wouldn't recommend changing this
public static double REVERSE_SPEED_COEFFICIENT = 1;// I wouldn't recommend changing this



}
