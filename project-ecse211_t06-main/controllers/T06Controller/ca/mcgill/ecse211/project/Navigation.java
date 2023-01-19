package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.sleepFor;
import java.util.ArrayList;
import java.util.List;
import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.RampEdge;
import ca.mcgill.ecse211.playingfield.Region;
import simlejos.hardware.ev3.LocalEV3;

/**
 * The Navigation class is used to make the robot navigate around the playing field.
 */
public class Navigation {
  private static float[] sampleL = {100};
  private static float[] sampleR = {100};

  private static Point dest;

  private static boolean isRightAngle;

  private static int counter;

  private static boolean stopBefore = false;

  private static int lineCorrectionCnt = 0;


  /** Do not instantiate this class. */
  private Navigation() {}


  /**
   * Travels to the given destination. we isTail is true, will use the US_SENSOR_TO_WHEEL_DIST to correct the movement
   * so car will not push too much
   * 
   * @param destination the point the robot must end up
   * @param isTail whether we are in the process of pushing a box
   */
  public static boolean travelTo(Point destination, boolean isTail, boolean careObs, boolean rightCorrection, boolean careObsHard) {
    lineCorrectionCnt = 0;
    dest = destination;
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);


    Point robotPosition = new Point(odometer.getXyt()[0], odometer.getXyt()[1]); // get current location of the robot
    double destinationAngle = getDestinationAngle(robotPosition, destination); // angle in degrees, 4 come from nowhere

    double initialAngle = odometer.getXyt()[2];
    double minimalAngle = minimalAngle(initialAngle, destinationAngle);

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    
    System.out.println("Im turning by " + minimalAngle + "My initial angle is : " + initialAngle + "  Dest: " + destinationAngle);
    turnTo(minimalAngle);
    
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    double tailCorrection = (isTail) ? WHEEL_TO_CENTER_DIST : 0; // checks if need a tail correction

    double hypothenus = distanceBetween(robotPosition, destination) - tailCorrection; // both in meters robotPosition is from the odometer could be some
    
    System.out.println("SUPPOSED TO GO THIS DISTANCE -->  " + hypothenus);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    boolean seeObs = goStraightFor(convertDistance(hypothenus), isTail, careObs, rightCorrection, careObsHard, destination);

    rightMotor.stop();
    leftMotor.stop();
    
    return seeObs;
  }

  /**
   * This method moves the robot a certain distance. If isTail is true, will apply the line correction
   * 
   * @param convertDistance
   * @param isTail
   */
  private static boolean goStraightFor(int destinatedTacho, boolean isTail, boolean careObs, boolean rightCorrection, boolean careObsHard, Point destination) {
    int leftInitialTacho = leftMotor.getTachoCount();
    int rightInitialTacho = rightMotor.getTachoCount();
    int leftRemain;
    int rightRemain;

    boolean corrected = false;
    leftMotor.stop();
    rightMotor.stop();
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    
    leftMotor.forward();
    rightMotor.forward();

    if (isTail) {
      try {
        Thread.sleep(2000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    do {
      try {
        Thread.sleep(10);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      rightRemain = rightMotor.getTachoCount() - rightInitialTacho - destinatedTacho;
      leftRemain = leftMotor.getTachoCount() - leftInitialTacho - destinatedTacho;

      if (isTail) {

        double RT = rightMotor.getTorque();
        double LT = leftMotor.getTorque();
        counter++;
        if (counter == 25) {
          counter = 0;
          if (BOX_NUMBER == 1) {
            double average = average(RT, LT);
            ARR_BOX_1.add(average);
          }
          if (BOX_NUMBER == 2) {
            double average = average(RT, LT);
            ARR_BOX_2.add(average);
          }
          if (BOX_NUMBER == 3) {
            double average = average(RT, LT);
            ARR_BOX_3.add(average);
          }

        }

      }


      if (isTail || (isRightAngle && rightCorrection)) {
        lineCorrection(leftRemain, rightRemain);
      }
      
//      if (isTail) {
//        lineCorrection(leftRemain, rightRemain);
//      }
      
      if(careObs) {
        if (UltrasonicLocalizer.readUsDistanceRaw() < 25 && UltrasonicLocalizer.readUsDistanceRawTop() < 25) {
          System.out.println("Im not goign to crash into an obstacle");
          Util.moveStraightFor(-0.10);
          leftMotor.stop();
          rightMotor.stop();
          
          double angle = odometer.getXyt()[2];
          leftMotor.setSpeed(ROTATE_SPEED);
          rightMotor.setSpeed(ROTATE_SPEED);
          
          if (angle % 90 < 45) {
            turnTo(45-angle);   
            Util.moveStraightFor(0.6);
          }else {
            turnTo(angle-45);
            Util.moveStraightFor(0.6);
          }
          return true;
        }
      }
      
      if(careObsHard) {
        if (UltrasonicLocalizer.readUsDistanceRaw() < 35 && UltrasonicLocalizer.readUsDistanceRawTop() < 35) {
          System.out.println("Im not goign to crash into an obstacle hard");
          Util.moveStraightFor(-0.10);
          leftMotor.stop();
          rightMotor.stop();
          leftMotor.setSpeed(ROTATE_SPEED);
          rightMotor.setSpeed(ROTATE_SPEED);
          turnTo(15);
          if (UltrasonicLocalizer.readUsDistanceRaw() < 35 && UltrasonicLocalizer.readUsDistanceRawTop() < 35) {
            turnTo(-15-90);
          } else {
            turnTo(75);
          }
          Util.moveStraightFor(0.30);
          
          travelTo(destination, false, false, false, true);
          return true;
        }
      }
      
    } while ((!stopBefore && !isTail && (leftRemain < 0 || rightRemain < 0))
        || (!stopBefore && isTail && leftRemain < 0 && rightRemain < 0)); // could be changed here
    if (isTail) {
      stopBefore = false;
      leftMotor.stop();
      rightMotor.stop();
      leftMotor.setSpeed(ROTATE_SPEED);
      rightMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(-360, true);
      rightMotor.rotate(-360, false);
    }
    
    leftMotor.stop();
    rightMotor.stop();
    return false;
  }

  /**
   * This method will stop left motor if left color sensor sees a black line and same for right motor. once both are
   * stopped, it's going make them move again
   * 
   * @param leftRemain
   * @param rightRemain
   */
  public static void lineCorrection(int leftRemain, int rightRemain) {

    // note down the previous values
    float prevLColor = sampleL[0];
    float prevRColor = sampleR[0];

    stopBefore = false;

    // get the new values
    sampleL = new float[leftColorSensorPort.sampleSize()];
    sampleR = new float[rightColorSensorPort.sampleSize()];

    leftColorSensorPort.fetchSample(sampleL, 0);
    rightColorSensorPort.fetchSample(sampleR, 0);

    if (prevLColor - sampleL[0] > BLACK_MIN) {
      leftMotor.stop();

//      if (rightMotor.isMoving()) {
//        rightMotor.setSpeed(2 * ROTATE_SPEED);
//      }
    }

    if (prevRColor - sampleR[0] > BLACK_MIN) {
      rightMotor.stop();

//      if (leftMotor.isMoving()) {
//        leftMotor.setSpeed(2 * ROTATE_SPEED);
//      }
    }

    if (!leftMotor.isMoving() && !rightMotor.isMoving()) {
//      lineCorrectionCnt++;

      // TODO we need to have some line correction but not destructive

      // if ( lineCorrectionCnt % 2 == 0 && lineCorrectionCnt != 0) {
      // LightLocalizer.localizeForFinal();
      // System.out.println("Hello work;");
      // }

      // if im close to the box and this is the last line i hit,
      // i correct my location to the line that is right before the box. (in odo)
//      if ((leftRemain >= -500 || rightRemain >= -500)) {
        double roughAngle = odometer.getXyt()[2];
        if ((roughAngle <= 15 && roughAngle >= 0) || (roughAngle <= 360 && roughAngle >= 345)) {
          odometer.setTheta(0);
//          odometer.setX(dest.x * 0.3048);
//          odometer.setY((dest.y) * 0.3048 - (ODO_CORRECTION_SHOVEL));
        } else if (roughAngle < 105 && roughAngle > 75) {
          odometer.setTheta(90);
//          odometer.setX((dest.x) * 0.3048 - (ODO_CORRECTION_SHOVEL));
//          odometer.setY(dest.y * 0.3048);
        } else if (roughAngle < 195 && roughAngle > 165) {
          odometer.setTheta(180);
//          odometer.setX(dest.x * 0.3048);
//          odometer.setY((dest.y) * 0.3048 + (ODO_CORRECTION_SHOVEL));
        } else if (roughAngle < 285 && roughAngle > 255) {
          odometer.setTheta(270);
//          odometer.setX((dest.x) * 0.3048 + (ODO_CORRECTION_SHOVEL));
//          odometer.setY(dest.y * 0.3048);
        }
//        stopBefore = true;
//
//        leftMotor.stop();
//        rightMotor.stop();
//
//        leftMotor.setSpeed(ROTATE_SPEED);
//        rightMotor.setSpeed(ROTATE_SPEED);
//
//        leftMotor.rotate(205, true);
//        rightMotor.rotate(205, false);
//      }
        
      System.out.println("I did a line correction");

      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);

      leftMotor.forward();
      rightMotor.forward();
    }
  }


  /**
   * Turns the robot with a minimal angle towards the given input angle in degrees, no matter what its current
   * orientation is. This method is different from {@code turnBy()}.
   * 
   * @param angle angle where the robot must turn to
   */
  public static void turnTo(double angle) {
    // TODO
    // Hint: You can do this in one line by reusing some helper methods declared in this class
    leftMotor.rotate(convertAngle(angle), true);
    rightMotor.rotate(-convertAngle(angle), false);
  }

  /** Returns the angle that the robot should point towards to face the destination in degrees. */
  public static double getDestinationAngle(Point current, Point destination) {
    double x1 = current.x; // From the odometer already in meters
    double y1 = current.y;
    double x2 = destination.x * TILE_SIZE; // form the text file just coordinates need to covert to meters
    double y2 = destination.y * TILE_SIZE;
    double dx = x2 - x1; // difference in X
    double dy = y2 - y1; // difference in y
    double angle = Math.atan(dx / dy); // This angle in degrees
    angle = Math.toDegrees(angle);

    if ((angle >= 0 && angle <= 2) || (angle >= 358 && angle <= 360)) {
      isRightAngle = true;
    } else {
      isRightAngle = false;
    }

    if (dx > 0 && dy == 0) {
      return 90.0;
    } else if (dx < 0 && dy == 0) {
      return 270.0;
    }
    
    if (dy != 0) {
      if ((dx >= 0.0 && dy < 0.0) || (dx < 0.0 && dy < 0.0)) { // quadrant 4 and 3, almost working
        System.out.println("Before i correct, it was " + angle);
        angle += 180;
      }
    }

//    if ((dx > 0.0 && dy <= 0.0) || (dx <= 0.0 && dy < 0.0)) { // quadrant 4 and 3
//      angle += 180;
//    }
    
    return angle;
  }


  /**
   * finds the minimal using trigonmetry
   * 
   * @param initialAngle current heading of the robot
   * @param destAngle final heading of the robot
   * @return the signed minimal angle in degrees from initial angle to destination angle (deg).
   */

  public static double minimalAngle(double initialAngle, double destAngle) { // COULD BE PROBLEM HERE
    // ZIMU destAngle - initial
    double minAngle = destAngle - initialAngle;
    if (minAngle > 180) {
      return minAngle - 360.0;
    } else if (minAngle < -180) {
      return minAngle + 360;
    }
    return minAngle;// TODO
  }

  /**
   * 
   * @param p1 first point
   * @param p2 second point
   * @return the distance between the two points in tile lengths (feet).
   */
  public static double distanceBetween(Point p1, Point p2) {
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x * TILE_SIZE; // form the text file just coordinates need to covert to meters
    double y2 = p2.y * TILE_SIZE;
    double dx = x1 - x2; // difference in X
    double dy = y1 - y2; // difference in y
    double vector = Math.sqrt(dx * dx + dy * dy); // gets the hypothenus
    return vector; // TODO
  }

  /**
   * calculates the average of two numbers
   * 
   * @param one first number
   * @param two second number
   * @return the average of two numbers
   */
  public static double average(double one, double two) {
    // double first = Math.abs(one);
    // double second = Math.abs(two);
    // return (first + second) / 2;
    return one + two;
  }


  /**
   * Converts input distance to a rotation distance.
   * 
   * @param distance
   * @return the wheel rotations for input distance
   */
  public static int convertDistance(double distance) {
    return (int) Math.round((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation in degrees
   * 
   * @param angle
   * @return degrees neccesary for for rotation
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0);
  }
  
  
  /**
   * Gets the point to which the robot must travel to to get from its start location to start point of the tunnel
   * @param ll lower left corner of the tunnel
   * @param ur upper right corner of the tunnel
   * @param start start corner of the robot 
   * @return the start point of the tunnel
   */
  public static Point getTunnelStart(Point ll, Point ur, Point startCorner) {
    int diffX = (int) (ll.x - ur.x);
    int diffY = (int) (ll.y - ur.y);

    Point start = new Point(0, 0);

    if (diffX == 1 || diffX == -1) {
      start.x = (ll.x + ur.x) / 2.0;

      if (Math.abs(startCorner.y - ll.y) < Math.abs(startCorner.y - ur.y)) {
        start.y = ll.y;
      } else {
        start.y = ur.y;
      }

      if (start.y - startCorner.y < 0) {
        start.y += 1;
      } else {
        start.y -= 1;
      }
    } else if (diffY == 1 || diffY == -1) {
      start.y = (ll.y + ur.y) / 2.0;

      if (Math.abs(startCorner.x - ll.x) < Math.abs(startCorner.x - ur.x)) {
        start.x = ll.x;
      } else {
        start.x = ur.x;
      }

      if (start.x - startCorner.x < 0) {
        start.x += 1;
      } else {
        start.x -= 1;
      }
    }

    return start;
  }
  
  
  /**
   * Returns the end point of the tunnel base on the ll and ur corners
   * @param ll lower left corner of the tunnel
   * @param ur upper right corner of the tunnel
   * @param start Start corner of the robot
   * @return end point of the tunnel
   */
  public static Point getTunnelEnd(Point ll, Point ur, Point start) {
    Point endPoint = new Point(0, 0);

    if (ll.x == start.x + 1 || ll.x == start.x - 1) {
      endPoint.x = ur.x + ll.x - start.x;
      endPoint.y = start.y;
      return endPoint;
    }
    if (ll.y == start.y + 1 || ll.y == start.y - 1) {
      endPoint.y = ur.y + ll.y - start.y;
      endPoint.x = start.x;
      return endPoint;
    }
    if (ur.x == start.x + 1 || ur.x == start.x - 1) {
      endPoint.x = ll.x + ur.x - start.x;
      endPoint.y = start.y;
      return endPoint;
    }
    if (ur.y == start.y + 1 || ur.y == start.y - 1) {
      endPoint.y = ll.y + ur.y - start.y;
      endPoint.x = start.x;
      return endPoint;
    }

    return null;
  }

  public static double getStartAngle(Point p) {
    if (p.x <= 7 && p.y <= 4) {
      return 0;
    }

    if (p.x <= 7 && p.y >= 5) {
      return 90;
    }

    if (p.x >= 8 && p.y >= 5) {
      return 180;
    }

    if (p.x >= 8 && p.y <= 4) {
      return 270;
    }

    return 0;

  }
  
  
  /**
   * After the localization phase, the robot is directed to navigate across the tunnel. 
   * @param TNG_LL lower left point of the tunnel
   * @param TNG_UR upper right corner of the tunnel
   * @param TNG_startPoint start point of the robot based on the corner it has started in
   */
  public static void crossBridge(Point TNG_LL, Point TNG_UR, Point TNG_startPoint) {
    Point TNG_tunnelStart = getTunnelStart(TNG_LL, TNG_UR, TNG_startPoint);
    Point TNG_tunnelEnd = getTunnelEnd(TNG_LL, TNG_UR, TNG_tunnelStart);
    
    odometer.setXyt(0.3048 * TNG_startPoint.x, 0.3048 * TNG_startPoint.y, getStartAngle(TNG_startPoint));
    odometer.printPosition();

    System.out.println("goto start tng: " + TNG_tunnelStart.toString());
    travelTo(TNG_tunnelStart, false, true, true, false);

    // copy pasted from travleTo()
    double destinationAngle = getDestinationAngle(new Point(odometer.getXyt()[0], odometer.getXyt()[1]), TNG_tunnelEnd);
    double initialAngle = odometer.getXyt()[2];
    double minimalAngle = minimalAngle(initialAngle, destinationAngle);

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    System.out.println(minimalAngle);
    turnTo(minimalAngle);

    LightLocalizer.localizeForFinal(true);

    odometer.setXyt(0.3048 * TNG_tunnelStart.x, 0.3048 * TNG_tunnelStart.y,
        tunnleOrientation(TNG_tunnelStart, TNG_tunnelEnd));
    
    System.out.print("BEFORE TUNNEL   ");
    odometer.printPosition();

    System.out.println("goto end tng: " + TNG_tunnelEnd.toString());
    travelTo(TNG_tunnelEnd, false, true, true, false);
    
    System.out.println("I ARRIVED JAJAJAJAJAJA");
    
    LightLocalizer.localizeForFinal(false);
    odometer.setXyt(0.3048 * TNG_tunnelEnd.x, 0.3048 * TNG_tunnelEnd.y,
        tunnleOrientation(TNG_tunnelStart, TNG_tunnelEnd));
    
    System.out.print("After TUNNEL   ");
    odometer.printPosition();
  }
  
  /**
   * calculates the orientation of the tunnel
   * @param tn_start start point of the tunnel
   * @param tn_end end point of the tunnel
   * @return the orientation on degrees of the tunnel
   */
  public static double tunnleOrientation(Point tn_start, Point tn_end) {
    double angle = 0;
    if (tn_end.y - tn_start.y > 0)
      angle = 0;
    if (tn_end.x - tn_start.x > 0)
      angle = 90;
    if (tn_end.y - tn_start.y < 0)
      angle = 180;
    if (tn_end.x - tn_start.x < 0)
      angle = 270;

    return angle;
  }
  
  /**
   * forces the robot to fund a box
   * @return
   */
  public static boolean findBox() {
    System.out.println("Start to look for point");
    boolean found = false;

    double initialTacho = leftMotor.getTachoCount();
    double angle = convertAngle(360);

    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();

    int foundCnt = 0;
    while (leftMotor.getTachoCount() - initialTacho < angle) {
      int rawDown = UltrasonicLocalizer.readUsDistanceRaw();
      int rawUp = UltrasonicLocalizer.readUsDistanceRawTop();
      
      if (rawDown > 40) rawDown = 255;

      if (rawDown < 20 && rawUp > rawDown + 15) {
        foundCnt++;
        System.out.println("Raw Down is : " + rawDown + "and UP is " + rawUp);
        odometer.printPosition();
      }

      if (foundCnt >= 20) {
        found = true;
        break;
      }
    }
    leftMotor.stop();
    rightMotor.stop();
    
    return found;
  }
  
  /**
   * 
   * @param ramp
   */
  public static void getBox(RampEdge ramp) {
    boolean found = Navigation.findBox();

    if (found) {
      int oldLeft = leftMotor.getTachoCount();
      int oldRight = rightMotor.getTachoCount();

      double distance = UltrasonicLocalizer.readUsDistanceRaw() / 100.0;

      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);

      while (true) {
        distance /= 2.0;
        leftMotor.rotate(convertDistance(distance), true);
        rightMotor.rotate(convertDistance(distance), false);

        System.out.println(UltrasonicLocalizer.readUsDistanceRaw());

        if (distance <= 0.005 || UltrasonicLocalizer.readUsDistanceRaw() < 3)
          break;
      }
      System.out.println("i found a box.");
      
      LocalEV3.getAudio().beep(); // beeps once
      sleepFor(PHYSICS_STEP_PERIOD*40);
      LocalEV3.getAudio().beep(); // beeps once
      sleepFor(PHYSICS_STEP_PERIOD*40);
      LocalEV3.getAudio().beep(); // beeps once
      
      putInPit(ramp);      
    }


  }

  // ================================
  // search zone related
  // ================================
  
  /**
   * calculates all the points found in a specific region
   * @param r region corresponds to the search zone
   * @return a list of all the way points the robot must travel to in the search zone
   */
  public static List<Point> getAllPointsInRegion(Region r) {
    List<Point> list = new ArrayList<>();

    for (int i = 0; i <= (int) (r.ur.x - r.ll.x); i++) {
      for (int j = 0; j <= (int) (r.ur.y - r.ll.y); j++) {
        Point p = new Point(r.ll.x + i, r.ll.y + j);

        System.out.println(p.x + " <-- X , Y --> " + p.y);
        list.add(p);
      }
    }

    return list;
  }

  /**
   * once in search zone, I move to each point, then i do getBox in each point. Make sure to avoid obstacle during the
   * movement.
   */
  
  /**
   * This method allows the robot to traverse to and from each grid point in the search zone if it sees the box it will bring it to the pi.
   * If there is no time to continue on the search path the robot is directed home.
   * @param sz search zone region
   * @param ramp coordinates of the ramp
   * @param initialTime the simulation time
   */
  public static void moveInSearchZone(Region sz, RampEdge ramp, long initialTime) {
    Point small_LL = new Point(sz.ll.x + 1, sz.ll.y + 1);
    Point small_UR = new Point(sz.ur.x - 1, sz.ur.y - 1);

    Region smallSZ = new Region(small_LL, small_UR);
    
    RampEdge rampDeepCopy = new RampEdge(new Point(ramp.left.x, ramp.left.y), new Point(ramp.right.x, ramp.right.y));
    Region rampRegion = rampRegionFromEdge(ramp);

    List<Point> SZList = getAllPointsInRegion(smallSZ);
    List<Point> rampList = getAllPointsInRegion(rampRegion);
    
    List<Point> list = removeDuplicated(SZList, rampList);

    for (Point p : list) {
      if ((System.currentTimeMillis() / 1000) - initialTime > 210) {
        break;
      }
      
      System.out.println("going to  " + p.x + " <-- X , Y --> " + p.y);
      boolean seeObs = travelTo(p, false, true, true, false);
      
      if (seeObs) {
        System.out.println("I see an obs");
        continue;
      }
      
      if ((odometer.getXyt()[2] % 90 > 5 && odometer.getXyt()[2] % 90 < 45) ||
          (odometer.getXyt()[2] % 90 >= 45 && odometer.getXyt()[2] % 90 < 85)) {
        System.out.println("I do turn back to 0, myCurrent angle is  " + odometer.getXyt()[2]);
        leftMotor.stop();
        rightMotor.stop();
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        
        turnTo(minimalAngle((odometer.getXyt()[2]), 0));
        odometer.setTheta(0);
      }
      
      LightLocalizer.localizeAndComeBack();
      odometer.setXyt(p.x*0.3048, p.y*0.3048, roughtTheta(odometer.getXyt()[2]));
      odometer.printPosition();
      
      getBox(rampDeepCopy);
    }
  }
  
  /**
   * The robot is directed to its initial start position. The robot will cross the bridge back to its start position
   * @param TNG_LL lower left corner of the tunnel
   * @param TNG_UR upper right corner of the tunnel
   * @param TNG_startPoint the start point the robot has initially started at
   */
  public static void goHome(Point TNG_LL, Point TNG_UR, Point TNG_startPoint) {
    System.out.println("Im going home now");
    
    Point TNG_tunnelStart = getTunnelStart(TNG_LL, TNG_UR, TNG_startPoint);
    Point TNG_tunnelEnd = getTunnelEnd(TNG_LL, TNG_UR, TNG_tunnelStart);
    System.out.println(TNG_tunnelStart.toString() + "<---- Tunnel start    Tunnel end----> " + TNG_tunnelEnd.toString());
    
    travelTo(TNG_tunnelEnd, false, false, false, true);
    System.out.println("Now im in front of tunnel, im going to turn so i can face tunnel");

    // copy pasted from travleTo()
    double destinationAngle = getDestinationAngle(new Point(odometer.getXyt()[0], odometer.getXyt()[1]), TNG_tunnelStart);
    double initialAngle = odometer.getXyt()[2];
    double minimalAngle = minimalAngle(initialAngle, destinationAngle);
    
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    System.out.println("I turn by this" + minimalAngle);
    turnTo(minimalAngle);
    LightLocalizer.localizeForFinal(false);
    
    odometer.setXyt(0.3048 * TNG_tunnelEnd.x, 0.3048 * TNG_tunnelEnd.y,
        tunnleOrientation(TNG_tunnelEnd, TNG_tunnelStart));

    System.out.println("Im going back to the start of tunnel, the start point is this " + TNG_tunnelStart.toString());
    travelTo(TNG_tunnelStart, false, false, false, false);
    LightLocalizer.localizeForFinal(true);
    
    odometer.setXyt(0.3048 * TNG_tunnelStart.x, 0.3048 * TNG_tunnelStart.y,
        tunnleOrientation(TNG_tunnelEnd, TNG_tunnelStart));
    
    System.out.println("I should be in homezone and im goint to home");
    travelTo(TNG_startPoint, false, false, true, false);
    
    System.out.println("Im back to hooooommmmmeeeee now");
  }
  
  /**
   * approximates an angle to the nearest 90 degrees
   * @param t angle to approximate
   * @return Angle in degrees
   */
  
  public static double roughtTheta(double t) {
    
    if ((t <= 15 && t >= 0) || (t <= 360 && t >= 345)) {
      return 0;
    } else if (t < 105 && t > 75) {
      return 90;
    } else if (t < 195 && t > 165) {
      return 180;
    } else if (t < 285 && t > 255) {
      return 270;
    }
    
    return t;
  }
  
  /**
   * Calculates the region where the ramp is located based on the ramp edge points
   * @param re ramp edge region
   * @return a region where the ramp is located
   */
  public static Region rampRegionFromEdge(RampEdge re) {
    if (re.left.x == re.right.x) {
      if (re.left.y > re.right.y) {
        Point ur = re.left;
        ur.x += 2;
        return new Region(re.right, ur);
      }else {
        Point ll = re.left;
        ll.x -= 2;
        return new Region(ll, re.right);
      }
    }else if (re.left.y == re.right.y) {
      if (re.left.x < re.right.x) {
        Point ur = re.right;
        ur.y += 2;
        return new Region(re.left, ur);
      }else {
        Point ll = re.right;
        ll.y -= 2;
        return new Region(ll, re.left);
      }
    }
    
    return null;
  }
  
  /**
   * Removes the points that are blocked off by the ramp and the bucket
   * @param SZList Initial search zone list containing all the way points to travel in the search zone
   * @param rampList the list of points that the ramp is located on.
   * @return  Returns a updated search zone list.
   */
  public static List<Point> removeDuplicated(List<Point> SZList, List<Point> rampList) {
    for (int i = 0; i < SZList.size(); i++) {
      for (int j = 0; j < rampList.size(); j++) {
        if (SZList.size() > i && SZList.get(i).x == rampList.get(j).x && SZList.get(i).y == rampList.get(j).y) {
          System.out.println("I am Removing this :::: " + SZList.get(i).toString());
          SZList.remove(i);
        }
      }
    }
    
    return SZList;
    
  }
  
  /**
   * Once the robot has located a box, it will navigate to the ramp, then on the ramp to drop the box in the pit.
   * @param re the location of the ramp
   */
  public static void putInPit(RampEdge re) {
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    // determine where should i got to first
    Point p = new Point(0, 0);
    Point frontRamp = new Point(0, 0);
    Point inBucket = new Point(0, 0);
    if (re.left.x == re.right.x) {
      if (re.left.y > re.right.y) {  // when ramp is pointing to the right 
        p.x = re.left.x - 0.5;
        p.y = odometer.getXyt()[1]/TILE_SIZE;
        
        frontRamp.x = re.left.x - 0.5;
        frontRamp.y = (re.left.y + re.right.y)/2.0;
        
        inBucket.x = frontRamp.x + 1.5;
        inBucket.y = frontRamp.y;
      }else {   // when it is pointing to left
        p.x = re.left.x + 0.5;
        p.y = odometer.getXyt()[1]/TILE_SIZE;
        
        frontRamp.x = re.left.x + 0.5; 
        frontRamp.y = (re.left.y + re.right.y)/2.0;
        
        inBucket.x = frontRamp.x - 1.5;
        inBucket.y = frontRamp.y;
      }
    }else if (re.left.y == re.right.y) {
      if (re.left.x < re.right.x) {
        p.x = odometer.getXyt()[0]/TILE_SIZE;
        p.y = re.left.y-0.5;
        
        frontRamp.x = (re.left.x + re.right.x)/2.0;
        frontRamp.y = re.left.y-0.5;
        
        inBucket.x = frontRamp.x;
        inBucket.y = frontRamp.y + 1.5;
      }else {
        Point ll = re.right;
        p.x = odometer.getXyt()[0]/TILE_SIZE;
        p.y = re.left.y+0.5;
        
        frontRamp.x = (re.left.x + re.right.x)/2.0;
        frontRamp.y = re.left.y+0.5;
        
        inBucket.x = frontRamp.x;
        inBucket.y = frontRamp.y - 1.5;
      }
    }
    
    System.out.println("Im putting the box");
    travelTo(p, false, true, false, false);
    System.out.println("Im going from p to frontRamp " + p.toString() + " << p    frontRamp>> " + frontRamp.toString());
    travelTo(frontRamp, false, true, false, false);
    
    System.out.println("im going in to bucket:" + inBucket.toString()); 
    travelTo(inBucket, false, false, false, false);
    System.out.println("I should now be backing up"); 
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    Util.moveStraightFor(-0.45);
    
    System.out.println("im going back to where i was.");
    travelTo(p, false, true, false, false);
  }
}
