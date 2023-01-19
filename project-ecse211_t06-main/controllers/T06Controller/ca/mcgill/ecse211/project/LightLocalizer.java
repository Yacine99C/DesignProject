package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.ACCELERATION;

import static ca.mcgill.ecse211.project.Resources.BLACK_MAX;
import static ca.mcgill.ecse211.project.Resources.BLACK_MIN;
import static ca.mcgill.ecse211.project.Resources.COLOUR_DIFFERENCE_THRESHOLD;
import static ca.mcgill.ecse211.project.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.project.Resources.HALF_ROBOT_LENGTH;
import static ca.mcgill.ecse211.project.Resources.LINE_THRESH;
import static ca.mcgill.ecse211.project.Resources.ROBOT_DISPLACEMENT;
import static ca.mcgill.ecse211.project.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.leftColor;
import static ca.mcgill.ecse211.project.Resources.leftColorSensorPort;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightColor;
import static ca.mcgill.ecse211.project.Resources.rightColorSensorPort;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static ca.mcgill.ecse211.project.Resources.averageLeft;
import static ca.mcgill.ecse211.project.Resources.averageRight;



public class LightLocalizer {


  private static int prevLeftColour;
  private static int prevRightColour;

  private static int leftColour;
  private static int rightColour;

  private static boolean leftIsBlack;
  private static boolean rightIsBlack;

  private static final int LEFT = 0;
  private static final int RIGHT = 1;

  /**
   * Method called to localize for LAB5
   */
  public static void localize() {
    //set speedslocalize
    setRobotSpeed();

    //move forward until see black on left and right sensor
    passBlackLine(false);
    odometer.setTheta(0.0);
    odometer.setX(TILE_SIZE + ROBOT_DISPLACEMENT);
    moveIntoPosition(false);  
  }

  /**
   * Method used to localize for the final project
   */
  public static void localizeForFinal(boolean isBridge) {
    int ninty = 90;

    //set speedslocalize
    setRobotSpeed();

    //move forward until see black on left and right sensor
    passBlackLine(false);
    //odometer.setTheta(0.0);
    //odometer.setX(TILE_SIZE + ROBOT_DISPLACEMENT);
    moveIntoPosition(isBridge);  

    if (isBridge) {
      ninty = -ninty;
    }

    turnBy(ninty);

    Util.moveStraightFor(-0.15);

    turnBy(-ninty);
  }
  
  /**
   * localizes and comes back
   */
  public static void localizeAndComeBack() {
    int ninty = 90;

    //set speedslocalize
    setRobotSpeed();

    //move forward until see black on left and right sensor
    passBlackLine(false);
    //odometer.setTheta(0.0);
    //odometer.setX(TILE_SIZE + ROBOT_DISPLACEMENT);
    moveIntoPosition(false);  
  }

  /**
   * Sets the robot speed. Internally sets the speed and the acceleration of the robot
   */
  public static void setRobotSpeed() {
    setSpeed(FORWARD_SPEED);
    setAcceleration(ACCELERATION);
  }

  /**
   * Takes a sample of both the left and right light sensors. Sets the values of @param prevLeftColour and @param prevRightColour
   */
  public static void passBlackLine(boolean isBridge) {
    prevLeftColour = lightSensorSample(LEFT);
    prevRightColour = lightSensorSample(RIGHT);

    setRobotSpeed();

    if (isBridge) {
      leftMotor.backward();
      rightMotor.backward();
    } else {      
      leftMotor.forward();
      rightMotor.forward();
    }

    //boolean we use to indicate if we see a black line
    boolean undetected = true;

    //int for colorDetection, once 2 we know both have seen color
    int lightDetected = 0;

    while(undetected) {

      leftColour = lightSensorSample(LEFT);
      rightColour = lightSensorSample(RIGHT);

      //keep moving left until detected
      leftIsBlack = isBlack(prevLeftColour, leftColour);
      if(leftIsBlack) {
        // leftMotor.stop(true);
        leftMotor.setSpeed(0);
        lightDetected += 1;
      }

      //keep moving right until detected
      rightIsBlack = isBlack(prevRightColour, rightColour);
      if(rightIsBlack) {
        // rightMotor.stop();
        rightMotor.setSpeed(0);
        lightDetected += 1;
      }

      if(lightDetected == 2) {
        undetected = false;
      } else {
        prevLeftColour = leftColour;
        prevRightColour = rightColour;
      }
    }

  }

  /**
   *  Sets the robot into position (AT the 1, 1 position) after hitting the first line. Also updates odometer.
   */
  public static void moveIntoPosition(boolean isBridge) {

    setRobotSpeed();
    //back robot to midway over line
    Util.moveStraightFor(-HALF_ROBOT_LENGTH-0.01);
    //turn towards cross section
    Util.turnBy(90.0);
    //move towards next line
    passBlackLine(isBridge);

    odometer.setY(TILE_SIZE + ROBOT_DISPLACEMENT);

    setRobotSpeed();
    //back robot to midway over line
    Util.moveStraightFor(-HALF_ROBOT_LENGTH-0.01 - LINE_THRESH);
    //turn forward again
    Util.turnBy(-90);

    Util.moveStraightFor(-LINE_THRESH);
  }

  /**
   * 
   * @param lightSensor
   * @return returns the value of the light sample
   */
  private static int lightSensorSample(int lightSensor) {

    float[] sample = null;

    if (lightSensor == LEFT) {
      sample = new float[leftColorSensorPort.sampleSize()];
      leftColorSensorPort.fetchSample(sample, 0);
    } else if (lightSensor == RIGHT) {
      sample = new float[rightColorSensorPort.sampleSize()];
      rightColorSensorPort.fetchSample(sample, 0);
    } else {
      throw new RuntimeException("Unexpected value in lightSensorSample");
    }

    return (int) (sample[0]);

  }

  /**
   * Will check based on previous and current colors if a black line was detected
   * @param prevColour previous color taken by light sensor
   * @param colour current color taken by light sensor
   * @return boolean value indicating if a black line was detected
   */
  private static boolean isBlack(int prevColour, int colour) {
    int colourDiff = Math.abs(colour - prevColour);
    if (colourDiff >= COLOUR_DIFFERENCE_THRESHOLD || (colour >= BLACK_MIN && colour <= BLACK_MAX)) {
      return true;
    } else {
      return false;
    }
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
   * turns the left and right motor by a specific angle. Angle is converted to a distance internally
   * @param angle
   */
  public static void turnBy(double angle) {
    leftMotor.rotate(Util.convertAngle(angle), true);
    rightMotor.rotate(-Util.convertAngle(angle), false);
  }
  
  
  public static boolean isRed(double prevColour, double colour) {

    if((colour >= Resources.RED_MIN && colour <= Resources.RED_MAX)) return true;
    else return false;
  }
  
  /**
   * 
   * @param prevColour first color reading
   * @param colour second color reading
   * @return true if green false if not
   */
  public static boolean isGreen(double prevColour, double colour) {
    System.out.println(colour);
    if((colour >= Resources.GREEN_MIN && colour <= Resources.GREEN_MAX)) return true;
    else return false;
  }

/**
 * The method fills up an leftColor array and rightColor array with the floor color values
 * @param color takes in the default color
 * 
 */
  public static void readFloorColour(String color) {
    int i =0;
    while (i<100) {
      if (color.equals("red")) {
        if(lightSensorSample(0)>= Resources.RED_MIN && lightSensorSample(0) <= Resources.RED_MAX) {
          leftColor.add((double)lightSensorSample(0));
        }
        if(lightSensorSample(1) >= Resources.RED_MIN && lightSensorSample(1) <= Resources.RED_MAX) {
          rightColor.add((double)lightSensorSample(1)) ;

        }
      }
      else if (color.equals("green")) {
        if (lightSensorSample(0) >= Resources.GREEN_MIN && lightSensorSample(0) <= Resources.GREEN_MAX) {

          leftColor.add((double)lightSensorSample(0));
        }
        if(lightSensorSample(1) >= Resources.GREEN_MIN && lightSensorSample(1) <= Resources.GREEN_MAX) {
          rightColor.add((double)lightSensorSample(1)) ;
        }
      }
      i++;
    }
    for (Double d: leftColor) {

      averageLeft = averageLeft + d;
    }

    for (Double d: rightColor) {
      averageRight = averageRight + d;
    }

    averageLeft /= leftColor.size();
    averageRight /= rightColor.size();
    leftColor.clear();
    rightColor.clear();
  }



}
