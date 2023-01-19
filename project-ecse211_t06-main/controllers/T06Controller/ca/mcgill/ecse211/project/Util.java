package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.BASE_WIDTH;
import static ca.mcgill.ecse211.project.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.project.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;

public class Util {
  //Methods from previous labs
  /**
   * Moves the robot straight for the given distance.
   * 
   * @param distance in cm
   */
  public static void moveStraightFor(double distance) {
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
  }
  
  /**
   * Moves the robot straight for the given distance
   * @param distance
   * @param b
   */
  public static void moveStraightFor(double distance, boolean b) {
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), b);
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * 
   * @param angle the input angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * BASE_WIDTH* angle / 360.0);
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int)Math.round((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    rightMotor.stop(true);
    leftMotor.stop(false);
  }

  /**
   * Sets the speed of both motors to the same values.
   * 
   * @param speed the speed in degrees per second
   */
  public static void setSpeed(int speed) {
    setSpeeds(speed, speed);
  }

  /**
   * Sets the speed of both motors to different values.
   * 
   * @param leftSpeed the speed of the left motor in degrees per second
   * @param rightSpeed the speed of the right motor in degrees per second
   */
  public static void setSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }

  /**
   * Sets the acceleration of both motors.
   * 
   * @param acceleration the acceleration in degrees per second squared
   */
  public static void setAcceleration(int acceleration) {
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);
  }

  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code Navigation.turnTo()}. For example, if the robot is facing 90 degrees, calling
   * {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   * {@code Navigation.turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }

  /**
   * Turns the robot by a specified angle.
   * 
   * @param angle in degrees to turn by
   * @param x boolean corresponding to immediate return
   */
  public static void turnBy(double angle, boolean x) {
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), x);
  }


  /**
   * turns the robot to a specified angle.
   * 
   * @param angle (in degrees) to turnTo
   */

  public static void turnTo(double angle) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // adjusting the angle in order to have an optimal turn (a turn with the
    // minimum angle)
    if (angle <= -180) {
      angle += 360;
    } else if (angle > 180) {
      angle -= 360;
    }

    // turn to the left if angle is negative and turn to the right if angle is positive
    if (angle < 0) {
      leftMotor.rotate(-convertAngle(-angle), true);
      rightMotor.rotate(convertAngle(-angle), false);
    } else {
      leftMotor.rotate(convertAngle(angle), true);
      rightMotor.rotate(-convertAngle(angle), false);
    }
  }
}