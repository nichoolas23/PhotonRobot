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

/*
*
* Tips and Tricks for le robot
*
* When starting the robot it is extremely extremely important that the arm is vertical and the wrist is tucked otherwise the encoders will not calibrate
* Also the claw doesn't account for the angle of the arm when compensating feed forward I will fix that when I arrive it really isn't that critical
*
* When picking up the cones open the wrist and make sure the arm is on the lower thicker part of the cone and then close it
* for the cubes you can honestly try and close the wrist around them hopefully they don't pop but when I tested them they only squished
*
* I don't have autonomous doing anything right now other than driving out which really sucks, but I simply could not get the arm to be reliably controlled in time.
* Also the charging station auto balance is a bit dangerous since there was no testing done on that with the actual robot and I do not want risk the robot being tipped on its first try. I will play around with it when I get there
*
* please for the love of god do not put the robot in front of the charging station in preparation for autonomous because as of now it just drives out until the navx thinks its gone 3 meters
*
 */

}
