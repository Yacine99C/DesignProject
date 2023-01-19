package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;

import java.lang.Thread;
import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.RampEdge;
import ca.mcgill.ecse211.playingfield.Region;
import simlejos.hardware.ev3.LocalEV3;

/**
 * Main class of the program.
 * 
 * TODO Describe your project overview in detail here (in this Javadoc comment).
 */
public class Main {
  
  /**
   * The number of threads used in the program (main, odometer), other than the one used to
   * perform physics steps.
   */
  public static final int NUMBER_OF_THREADS = 2;
  
  /** Main entry point. */
  public static void main(String[] args) {
    initialize();
    
    // Start the odometer thread
    new Thread(odometer).start();
    
    
    LocalEV3.getAudio().beep(); // beeps once
    wifiExample();
    
    long initialTime = System.currentTimeMillis() / 1000;
    
    LightLocalizer.readFloorColour("red");
    boolean b1 = LightLocalizer.isRed(averageLeft, averageLeft);
    averageLeft = 0;
    averageRight = 0;
    LightLocalizer.readFloorColour("green");
    boolean b2 = LightLocalizer.isGreen(averageLeft, averageLeft);
    System.out.println(b1 + "is RED");
    System.out.println(b2 + "is GREEN");
    

    usLocalizer.Testlocalize();    
    LightLocalizer.localize();
    
    /**
     * beep three times when at 1,1 position
     */
    LocalEV3.getAudio().beep(); // beeps once
    sleepFor(PHYSICS_STEP_PERIOD*40);

    LocalEV3.getAudio().beep(); // beeps once
    sleepFor(PHYSICS_STEP_PERIOD*40);

    LocalEV3.getAudio().beep(); // beeps once

 
    Point TN_startPoint = new Point(1, 1);
    
    if (b1 == true && redCorner == 3) { 
      TN_startPoint.x = 1;
      TN_startPoint.y = 8;      
    }
    
    else if (b1 == true && redCorner == 0) {
      TN_startPoint.x = 1;
      TN_startPoint.y = 1;  
    }
    
    else if (b2 == true && greenCorner == 2) {
      TN_startPoint.x = 14;
      TN_startPoint.y = 8;  
    }
    
    else {
      TN_startPoint.x = 14;
      TN_startPoint.y = 1; 
    }
    
    //by default set it to red
    Point TN_LL = tnr.ll;
    Point TN_UR = tnr.ur;
    
    if (b1 == true ) {
      TN_LL = tnr.ll;
      TN_UR = tnr.ur;
    }
    
    else {
      TN_LL = tng.ll;
      TN_UR = tng.ur;
    }
    
    
    System.out.println(TN_LL + "   <--- tnr.ll  tnr,ur--->   " + TN_UR);
    
    Navigation.crossBridge(TN_LL, TN_UR, TN_startPoint);
    /**
     * beep three times when cross the bridge
     */
    LocalEV3.getAudio().beep(); // beeps once
    sleepFor(PHYSICS_STEP_PERIOD*40);
    LocalEV3.getAudio().beep(); // beeps once
    sleepFor(PHYSICS_STEP_PERIOD*40);
    LocalEV3.getAudio().beep(); // beeps once

    
    // by default set to red
    Region SZ = Resources.szr;
    if (b1 == true ) {
      SZ = Resources.szr;
    }
    
    else {
      SZ = Resources.szg;
    }
    
    
    // by default set to red
    RampEdge ramp = Resources.rr;
    
    if (b1 == true ) {
      ramp = Resources.rr;
    }
    
    else {
      ramp = Resources.gr;
    }
    
    
    Navigation.moveInSearchZone(SZ, ramp, initialTime);
    
    //Navigation.getBox();
    
    Navigation.goHome(TN_LL, TN_UR, TN_startPoint);

  }
  
  
  
  /**
   * Example using WifiConnection to communicate with a server and receive data concerning the
   * competition such as the starting corner the robot is placed in.<br>
   * 
   * <p>Keep in mind that this class is an <b>example</b> of how to use the Wi-Fi code; you must use
   * the WifiConnection class yourself in your own code as appropriate. In this example, we simply
   * show how to get and process different types of data.<br>
   * 
   * <p>There are two variables you MUST set manually (in Resources.java) before using this code:
   * 
   * <ol>
   * <li>SERVER_IP: The IP address of the computer running the server application. This will be your
   * own laptop, until the beta beta demo or competition where this is the TA or professor's laptop.
   * In that case, set the IP to the default (indicated in Resources).</li>
   * <li>TEAM_NUMBER: your project team number.</li>
   * </ol>
   * 
   * <p>Note: You can disable printing from the Wi-Fi code via ENABLE_DEBUG_WIFI_PRINT.
   * 
   * @author Michael Smith, Tharsan Ponnampalam, Younes Boubekeur, Olivier St-Martin Cormier
   */
  public static void wifiExample() {
    System.out.println("Running...");

    // Example 1: Print out all received data
    System.out.println("Map:\n" + wifiParameters);

    // Example 2: Print out specific values
    System.out.println("Red Team: " + redTeam);
    System.out.println("Green Zone: " + green);
    System.out.println("Island Zone, upper right: " + island.ur);
    System.out.println("Red tunnel footprint, lower left y value: " + tnr.ll.y);

    // Example 3: Compare value
    if (szg.ll.x >= island.ll.x && szg.ll.y >= island.ll.y) {
      System.out.println("The green search zone is on the island.");
    } else {
      System.err.println("The green search zone is in the water!");
    }
    
    // Example 4: Calculate the area of a region
    System.out.println("The island area is " + island.getWidth() * island.getHeight() + ".");
  }

  /**
   * Initializes the robot logic. It starts a new thread to perform physics steps regularly.
   */
  private static void initialize() {    
    // Run a few physics steps to make sure everything is initialized and has settled properly
    for (int i = 0; i < 50; i++) {
      performPhysicsStep();
    }

    // We are going to start two threads, so the total number of parties is 2
    setNumberOfParties(NUMBER_OF_THREADS);
    
    // Does not count as a thread because it is only for physics steps
    new Thread(() -> {
      while (performPhysicsStep()) {
        sleepFor(PHYSICS_STEP_PERIOD);
      }
    }).start();
  }

}
