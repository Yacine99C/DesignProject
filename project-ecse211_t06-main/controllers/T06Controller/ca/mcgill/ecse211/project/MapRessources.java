package ca.mcgill.ecse211.project;


import java.math.BigDecimal;
import java.util.Map;
import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.Region;
import ca.mcgill.ecse211.wificlient.WifiConnection;

public class MapRessources {
  
  /**
   * The default server IP used by the profs and TA's.
   */
  public static final String DEFAULT_SERVER_IP = "192.168.2.3";
  
  /**
   * The IP address of the server that transmits data to the robot. Set this to the default for the
   * beta demo and competition.
   */
  public static final String SERVER_IP = "192.168.2.21";
  
  /**
   * Your team number.
   */
  public static final int TEAM_NUMBER = 6;
  
  /**
   * Enables printing of debug info from the WiFi class.
   */
  public static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
  
  /**
   * Enable this to attempt to receive Wi-Fi parameters at the start of the program.
   */
  public static final boolean RECEIVE_WIFI_PARAMS = true;
  
  /**
   * Container for the Wi-Fi parameters.
   */
  public static Map<String, Object> wifiParameters;
  
  // This static initializer MUST be declared before any Wi-Fi parameters.
  /*
  static {
      receiveWifiParameters();
  }
  */
  
  /**
   * Red team number.
   */
  public static int redTeam = get("RedTeam");
  
  /**
   * Red team's starting corner.
   */
  public static int redCorner = get("RedCorner");
  
  /**
   * Green team number.
   */
  public static int greenTeam = get("GreenTeam");
  
  /**
   * Green team's starting corner.
   */
  public static int greenCorner = get("GreenCorner");
  
  /**
   * The Red Zone.
   */
  public static Point R_ZONE_P1 = new Point(get("Red_LL_x"), get("Red_LL_y"));
  public static Point R_ZONE_P2 = new Point(get("Red_UR_x"), get("Red_UR_y"));
  
  public static Region red = new Region(R_ZONE_P1, R_ZONE_P2);
  
  /**
   * The Green Zone.
   */
  
  public static Point G_ZONE_P1 = new Point(get("Green_LL_x"), get("Green_LL_y"));
  public static Point G_ZONE_P2 = new Point(get("Green_UR_x"), get("Green_UR_y"));
  public static Region green = new Region(G_ZONE_P1, G_ZONE_P2);
  
  /**
   * The Island.
   */
  public static Point ISLAND_P1 = new Point(get("Island_LL_x"), get("Island_LL_y"));
  public static Point ISLAND_P2 = new Point(get("Island_UR_x"), get("Island_UR_y"));
  public static Region island = new Region(ISLAND_P1, ISLAND_P2);
  
  /**
   * The red tunnel footprint.
   */
  public static Point R_TUNNEL_P1 = new Point(get("TNR_LL_x"), get("TNR_LL_y"));
  public static Point R_TUNNEL_P2 = new Point(get("TNR_UR_x"), get("TNR_UR_y"));
  public static Region tnr = new Region(R_TUNNEL_P1, R_TUNNEL_P2);
  
  
  /**
   * The green tunnel footprint.
   */
  public static Point G_TUNNEL_P1 = new Point(get("TNG_LL_x"), get("TNG_LL_y"));
  public static Point G_TUNNEL_P2 = new Point(get("TNG_UR_x"), get("TNG_UR_y"));
  public static Region tng = new Region(G_TUNNEL_P1, G_TUNNEL_P2);
  
  /**
   * The Red search zone.
   */
  public static Point R_SZONE_P1 = new Point(get("SZR_LL_x"), get("SZR_LL_y"));
  public static Point R_SZONE_P2 = new Point(get("SZR_UR_x"), get("SZR_UR_y"));
  public static Region szr = new Region(G_TUNNEL_P1, G_TUNNEL_P2);
  
  /**
   * The Green search zone.
   */
  public static Point G_SZONE_P1 = new Point(get("SZG_LL_x"), get("SZG_LL_y"));
  public static Point G_SZONE_P2 = new Point(get("SZG_UR_x"), get("SZG_UR_y"));
  public static Region szg = new Region(G_TUNNEL_P1, G_TUNNEL_P2);
  
  
  /**
   * The location of the red target bin.
   */
  public static Point redBinEntryRampP_L = new Point(get("RRL_x"), get("RRL_y"));
  public static Point redBinEntryRampP_R = new Point(get("RRR_x"), get("RRR_y"));

  /**
   * The location of the green target bin.
   */
  public static Point greenBinEntryRampP_L = new Point(get("GRL_x"), get("GRL_y"));
  public static Point greenBinEntryRampP_R = new Point(get("GRR_x"), get("GRR_x"));

  
  /**
   * Receives Wi-Fi parameters from the server program.
   */
//  public static void receiveWifiParameters() {
//      // Only initialize the parameters if needed
//      if (!RECEIVE_WIFI_PARAMS || wifiParameters != null) {
//          return;
//      }
//      System.out.println("Waiting to receive Wi-Fi parameters.");
//      
//      // Connect to server and get the data, catching any errors that might occur
//      try (WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT)) {
//          /*
//           * getData() will connect to the server and wait until the user/TA presses the "Start" button
//           * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
//           * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
//           * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
//           * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
//           * but receives corrupted data or a message from the server saying something went wrong. For
//           * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
//           * will receive a message saying an invalid team number was specified and getData() will throw
//           * an exception letting you know.
//           */
//          wifiParameters = conn.getData();
//          System.out.println(wifiParameters);
//      } catch (Exception e) {
//          System.err.println("Error: " + e.getMessage());
//      }
//  }
  
  /**
   * Returns the Wi-Fi parameter int value associated with the given key.
   *
   * @param key the Wi-Fi parameter key
   * @return the Wi-Fi parameter int value associated with the given key
   */
  public static int get(String key) {
      if (wifiParameters != null) {
          return ((BigDecimal) wifiParameters.get(key)).intValue();
      } else {
          return 0;
      }
  }

}
