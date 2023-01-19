package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.ACCELERATION;
import static ca.mcgill.ecse211.project.Resources.BASE_WIDTH;
import static ca.mcgill.ecse211.project.Resources.MOTOR_LOW;
import static ca.mcgill.ecse211.project.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.project.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static ca.mcgill.ecse211.project.Resources.usSensor;
import static ca.mcgill.ecse211.project.Resources.usSensorTop;
import static ca.mcgill.ecse211.project.Resources.leftColor;
import static ca.mcgill.ecse211.project.Resources.leftColorSensorPort;
import static ca.mcgill.ecse211.project.Resources.rightColor;
import static ca.mcgill.ecse211.project.Resources.rightColorSensorPort;


public class UltrasonicLocalizer {

  // TODO SAMI change this
  private static final int WALL_DISTANCE = 55; //35 distance sensor detects before turning away
  private static final int ERROR_MARGIN = 5; //6 error margin is introduced to counter the effect of noise
  /** Buffer (array) to store US samples. */
  private static float[] usData = new float[usSensor.sampleSize()];
  private static float[] usDataTop = new float[usSensorTop.sampleSize()];


  /** The distance remembered by the filter() method. */
  private static int prevDistance;
  /** The number of invalid samples seen by filter() so far. */
  private static int invalidSampleCount;
  
  private static final int THRESHOLD_VALUE = 38;
  private static final int MARGIN = 3;
  private static final int FILTER_OUT = 10;
  private static final int maxDistance = 255; // Filter out any distance over 100 cm
  private static int filterControl;
  private static double deltaT = 0; 
  
  
  /** 
   * NEW DATA FOR NEW METHOD
   */
  private static final int WALL_DISTANCE1 = 55; //35 distance sensor detects before turning away
  private static final int ERROR_MARGIN1 = 10; //6 error margin is introduced to counter the effect of noise
  private static final int WALL_DISTANCE2 = 100; //35 distance sensor detects before turning away
  private static final int ERROR_MARGIN2 = 5; //6 error margin is introduced to counter the effect of noise
  
  public void TestOdometer() {
    setup();

    while(true) {
      turnCounterclockwise();
      System.out.println(readUsDistance());
    }
  }
  
  
  /**
   * Localizes the robot to 0 degrees
   */
  public void Testlocalize() {
    setup();
    int loopcount = 0;
    double firstAngle;
    double secondAngle;
    boolean wasfacing = false;
    boolean turning = true;
    int count = 0;
    int other_count = 100; 
    int initial_count = 100;
    
    
    /**
     * initialize the us sensor
     */
    while (turning) {
      readUsDistance();
      if (initial_count == 0 ) {
        turning = false;
      }
      turnCounterclockwise();
      initial_count --;
    }
    
    /**
     * Start your turn, determine if you are looking at wall
     */
    turning = true;
    while (turning) {
      if ( readUsDistance() < 80 ) {
        count += 1;
        if(count == 100) {
         wasfacing = true;
         turning = false; 
        }
      }
      turnCounterclockwise();
      other_count--;
      if (other_count == 0) {
        turning = false;
      }
    }
    
    /**
     * If you are looking at the wall then turn away from the wall until you do not face wall again
     */
    turning= true;
    count = 0;
    while(turning && wasfacing) {
      //System.out.println(readUsDistance());
      if ( readUsDistance() > WALL_DISTANCE2 - ERROR_MARGIN2 && readUsDistance() < WALL_DISTANCE2 + (ERROR_MARGIN2) ) {
      //if ( readUsDistance() < WALL_DISTANCE1 - ERROR_MARGIN1 ) {
        count += 1;
        if(count == 10) {
         turning = false;
        } 
      }
      turnCounterclockwise();
    }
    
    /**
     * Find first wall angle
     */
    turning= true;
    count = 0;
    while(turning) {
      //System.out.println(readUsDistance());
      if ( readUsDistance() > WALL_DISTANCE1 - ERROR_MARGIN1 && readUsDistance() < WALL_DISTANCE1 + ERROR_MARGIN1 ) {
        count += 1;
        if(count == 6) {
         turning = false; 
        } 
      }
      turnCounterclockwise();
    }
    
    
    stop();
    firstAngle = odometer.getXyt()[2]; //store first theta value in a double variable
    System.out.println("first angle :" + firstAngle);
    turning= true;
    count = 0;
    setup();
    
    // until it is out of this range 80 to 90 
    while( readUsDistance() < 90 ) {
      turnCounterclockwise();
    }
    //System.out.println("LEFT WHILE loop ");

    count = 0;
    turning = true; 
    
    /**
     * Intermediate detection
     */
    while( readUsDistance() < 70 ) {
      turnCounterclockwise();
    }
    
    /**
     * Final wall angle detection
     */
    while(turning) {
      //System.out.println(readUsDistance());
      if ( readUsDistance() > WALL_DISTANCE2 - ERROR_MARGIN2 && readUsDistance() < WALL_DISTANCE2 + (ERROR_MARGIN2) ) {
      //if ( readUsDistance() < WALL_DISTANCE1 - ERROR_MARGIN1 ) {
        count += 1;
        if(count == 6) {
         turning = false; 
        } 
      }
      turnCounterclockwise();
    }
    
    stop();
    secondAngle = odometer.getXyt()[2];//store second theta value in a double variable
    //System.out.println("Second angle :" + secondAngle);
    
    //System.out.println("Second angle :" + secondAngle);
    System.out.println("first angle :" + firstAngle);
    
    System.out.println("Second angle :" + secondAngle);
    deltaT = (firstAngle - secondAngle) /2;
    double turn = 0;
    System.out.println("WAS FACING " + wasfacing);
    
    if (wasfacing == true) {
      turn = deltaT + 135 + 180; 
    }
    else {
      turn = deltaT + 135 ; 
    }
    
    //double turnAngle = turn + odometer.getXyt()[2] ;
    setup();
    turnBy(-turn);
    
    // set odometer to theta = 0
    System.out.println(odometer.getXyt()[2]);
    odometer.setXyt(0.0, 0.0, 0.0);
    
  }
  /**
   * Localizes the robot to 0 degrees
   */
  public void localize() {    
    setup();
    int loopcount = 0;
    double firstAngle;
    double secondAngle;
    boolean wasfacing = false;
    System.out.println(readUsDistance());
    
    while (readUsDistance() < WALL_DISTANCE) {
      wasfacing = true;
      System.out.println("in while loop");
      int hello = readUsDistance();
      System.out.println("this is hello" + hello);
      turnCounterclockwise();
      //readUsDistance();
    }
    int count = 0;
    boolean turning = true;
    
    while(readUsDistance() > WALL_DISTANCE - ERROR_MARGIN || loopcount == 0) {
       // call method to turn counter-clockwise
      int hello = readUsDistance();
      System.out.println("this is hello" + hello);
      count += 1;
      if(count == 6) {
       turning = false; 
      } 
      loopcount = 1;
      turnCounterclockwise();
    }

    stop();
    firstAngle = odometer.getXyt()[2]; //store first theta value in a double variable
    System.out.println("first angle :" + firstAngle);
    setup();
    turning = true;
    count = 0;
    while( readUsDistance() < WALL_DISTANCE + ERROR_MARGIN) {
      count += 1;
      if(count == 6) {
       turning = false; 
      } 
      turnCounterclockwise(); // call method to turn counter-clockwise
      System.out.println(readUsDistance());
    }
    stop();
    secondAngle = odometer.getXyt()[2];//store second theta value in a double variable
    //System.out.println("Second angle :" + secondAngle);
    //System.out.println("first angle :" + firstAngle);

    System.out.println("Second angle :" + secondAngle);
    deltaT = (firstAngle - secondAngle) /2;
    double turn = 0;
    if (wasfacing == true) {
      turn = deltaT + 135 + 180 + 8; 
    }
    else {
      turn = deltaT + 135 + 8; 
    }
    
    //double turnAngle = turn + odometer.getXyt()[2] ;
    setup();
    turnBy(-turn);
    
    // set odometer to theta = 0
    System.out.println(odometer.getXyt()[2]);
    odometer.setXyt(0.0, 0.0, 0.0);
    
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
    leftMotor.rotate(-convertAngle(angle), true); //set true first so motor waits for execution
    rightMotor.rotate(convertAngle(angle), false);
  }
  
  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * 
   * @param angle the input angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0);
  }
  
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  private static double filter(double distance) {
    if ((distance >= maxDistance || distance <= THRESHOLD_VALUE-MARGIN) && filterControl < FILTER_OUT) {
          // Bad value, do not set the distance var, however do increment the filter value
          filterControl++;
        } else if (distance >= maxDistance) {
          // We have repeated large values, so there must actually be nothing there: leave the distance alone
          return distance;
        } else if (distance <= THRESHOLD_VALUE-MARGIN){
          // We have repeated small values, so there must actually be something really close: leave distance alone
          return distance;
        } else {
          // distance went below 100: reset filter and leave distance alone.
          filterControl = 0;
          return distance;
    } 
    return distance;
  }
  
  public static int readUsDistance() {
    int distance;
    double distance1;
    double[] values = new double[10];
    for(int i = 0; i < 10; i++) {
      usSensor.fetchSample(usData, 0);
      values[i] = (double) usData[0];
    }
    
    distance1 = getMean(values);
    distance = (int) filter(distance1);
    //System.out.println(distance);
    if (distance < 57 && distance != 0 || (prevDistance-100 > distance)) { // was 45
      //System.out.println("DONE THISSSSS");
      return prevDistance;
    }
    else {
      prevDistance = distance;
      return distance; 
    }
    
  }
  
  public static int readUsDistanceRaw() {
    int distance;
    double[] values = new double[10];
    for(int i = 0; i < 10; i++) {
      usSensor.fetchSample(usData, 0);
      values[i] = (double) usData[0];
    }
    
    distance = (int)getMean(values);
    
    return distance;
  }
  
  public static int readUsDistanceRawTop() {
    int distance;
    double[] values = new double[10];
    for(int i = 0; i < 10; i++) {
      usSensorTop.fetchSample(usDataTop, 0);
      values[i] = (double) usDataTop[0];
    }
    
    distance = (int)getMean(values);
    
    return distance;
  }
  
  /**
   * Gets the average of an array of doubles
   * @param lightReadings An array of 10 light readings
   * @return the average of these 10 light readings
   */
  public static double getMean(double[] lightReadings) { 
    double total = 0;
    for(int i=0; i< lightReadings.length; i++){
        total = total + lightReadings[i];
    }
  
    double average = total / lightReadings.length;
    average = (int) (average* 100.0);
    return average;
    
  }
  
  /**
   * This method allows the robot to turn clockwise
   */
  public void turnClockwise() {
    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * This method allows the robot to turn counterclockwise
   */
  public void turnCounterclockwise() {
    leftMotor.backward();
    rightMotor.forward();
  }
  
  /**
   * This method allows the robot to turn counterclockwise
   */
  public void turnCounterclockwise1() {
    leftMotor.setSpeed(-MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
  }

  /**
   * This method allows the robot to stop
   */
  public void stop() {
    leftMotor.stop();
    rightMotor.stop();
  }

  /**
   * This method sets up the acceleration and the speeds of the motors at the beginning
   */
  public void setup() {
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
  }
  
  
}
