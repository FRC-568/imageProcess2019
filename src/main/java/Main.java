
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import javax.swing.Box;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

public final class Main {
  private static String configFile = "/boot/frc.json";

  @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();

  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    CameraServer inst = CameraServer.getInstance();
    UsbCamera camera = new UsbCamera(config.name, config.path);
    MjpegServer server = inst.startAutomaticCapture(camera);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      server.setConfigJson(gson.toJson(config.streamConfig));
    }

    return camera;
  }

  /**
   * Example pipeline.
   */
  public static class MyPipeline implements VisionPipeline, Sendable {
    public int val;
    public static final int CAMERA_WIDTH = 320; // 640;
    public static final int CAMERA_HEIGHT = 240; // 480;
    public static final double OFFSET_TO_FRONT = 0;
	  public static final double DISTANCE_CONSTANT = 5760; // 5738;
	  public static final double WIDTH_BETWEEN_TARGET = 13.3133853031; // inches

    GripPipeline pipeline = new GripPipeline();
    Mat output = new Mat();
    CvSink cvSink = CameraServer.getInstance().getVideo();
    CvSource cvSource = CameraServer.getInstance().putVideo("contours", CAMERA_WIDTH, CAMERA_HEIGHT);
    
    NetworkTableEntry heights;
    NetworkTableEntry widths;
    NetworkTableEntry positionX;
    NetworkTableEntry positionY;

    NetworkTableEntry sendCenterX;
    NetworkTableEntry sendDistanceFromTarget;
    NetworkTableEntry sendGetAngle;

    private double lengthBetweenContours;
  	private double distanceFromTarget;
	  private double[] centerX;
  
    public MyPipeline() {
      LiveWindow.add(this);
      LiveWindow.setEnabled(true);

       NetworkTable targetsTable = NetworkTableInstance.getDefault().getTable("Target Locations");
       heights = targetsTable.getEntry("heights");
       widths = targetsTable.getEntry("widths");
       positionX = targetsTable.getEntry("boxPositionX");
       positionY = targetsTable.getEntry("boxPositionY");

       NetworkTable dataToSendTable = NetworkTableInstance.getDefault().getTable("dataToSend");
       sendCenterX = dataToSendTable.getEntry("centerX");
       sendDistanceFromTarget = dataToSendTable.getEntry("distanceFromTarget");
       sendGetAngle = dataToSendTable.getEntry("getAngle");
    }
   
    public void process(Mat mat) {
 
      pipeline.process(mat);
      cvSink.grabFrame(mat);
      mat.copyTo(output);
      
      Integer[] boxWidths = new Integer[pipeline.filterContoursOutput().size()];
      Integer[] boxHeights = new Integer[pipeline.filterContoursOutput().size()];
      Integer[] boxPositionX = new Integer[pipeline.filterContoursOutput().size()];
      Integer[] boxPositionY = new Integer[pipeline.filterContoursOutput().size()];
      
      for(int i = 0; i < pipeline.filterContoursOutput().size(); i++ ) {
        var contour = pipeline.filterContoursOutput().get(i);
        var box = Imgproc.boundingRect(contour);
			  Imgproc.rectangle(output, new Point(box.x, box.y), new Point(box.x + box.width, box.y + box.height), new Scalar(100, 255, 227));
        //System.out.println("rectangle drawn");
        //System.out.println(box.width);
        
          pipeline.filterContoursOutput().get(i);
          
          boxWidths[i] = box.width;
          boxHeights[i] = box.height;
          boxPositionX[i] = box.x;
          boxPositionY[i] = box.y;
      }

      widths.setNumberArray(boxWidths);
      heights.setNumberArray(boxHeights);
      positionX.setNumberArray(boxPositionX);
      positionY.setNumberArray(boxPositionY);
      
      sendCenterX.setNumber(returnCenterX());
      sendDistanceFromTarget.setNumber(distanceFromTarget());
      sendGetAngle.setNumber(getAngle());

      cvSource.putFrame(output);
    }
    
    public double returnCenterX() {
      // This is the center value returned by GRIP thank WPI
      if (!pipeline.filterContoursOutput().isEmpty() && pipeline.filterContoursOutput().size() >= 2) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
        Rect r1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        centerX = new double[] { r1.x + (r1.width / 2), r.x + (r.width / 2) };
        // this again checks for the 2 shapes on the target
        if (centerX.length == 2) {
          // subtracts one another to get length in pixels
          lengthBetweenContours = Math.abs(centerX[0] - centerX[1]);
        }
      }
      return lengthBetweenContours;
    }

    public double distanceFromTarget() {
      // distance constant divided by length between centers of contours
      distanceFromTarget = DISTANCE_CONSTANT / lengthBetweenContours;
      return distanceFromTarget - OFFSET_TO_FRONT;
    }

    public double getAngle() {
      // 8.5in is for the distance from center to center from goal, then
      // divide by lengthBetweenCenters in pixels to get proportion
      double constant = WIDTH_BETWEEN_TARGET / lengthBetweenContours;
      double angleToGoal = 0;
      // Looking for the 2 blocks to actually start trig
      if (!pipeline.filterContoursOutput().isEmpty() && pipeline.filterContoursOutput().size() >= 2) {
  
        if (centerX.length == 2) {
          // this calculates the distance from the center of goal to
          // center of webcam
          double distanceFromCenterPixels = ((centerX[0] + centerX[1]) / 2) - (CAMERA_WIDTH / 2);
          // Converts pixels to inches using the constant from above.
          double distanceFromCenterInch = distanceFromCenterPixels * constant;
          // math brought to you buy Chris and Jones
          angleToGoal = Math.atan(distanceFromCenterInch / distanceFromTarget());
          angleToGoal = Math.toDegrees(angleToGoal);
          // prints angle
          // System.out.println("Angle: " + angleToGoal);
        }
      }
      //SmartDashboard.putNumber("angleToGoal", angleToGoal);
      return angleToGoal;
    }
  
    @Override
    public String getName() {
      return "cameraServer";
    }

    @Override
    public String getSubsystem() {
      return "cameraServerSubsystem";
    }

    @Override
    public void setName(String name) {

    }
    @Override
    public void setSubsystem(String subsystem) {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType(getName());
      
      builder.addDoubleProperty("upperHue", () -> pipeline.upperHue, value -> {pipeline.upperHue = value; System.out.println("upperHue");});
      builder.addDoubleProperty("lowerHue", () -> pipeline.lowerHue, value -> {pipeline.lowerHue = value;});
      builder.addDoubleProperty("upperSaturation", () -> pipeline.upperSaturation, value -> {pipeline.upperSaturation = value;});
      builder.addDoubleProperty("lowerSaturation", () -> pipeline.lowerSaturation, value -> {pipeline.lowerSaturation = value;});
      builder.addDoubleProperty("upperValue", () -> pipeline.upperValue, value -> {pipeline.upperValue = value;});
      builder.addDoubleProperty("lowerValue", () -> pipeline.lowerValue, value -> {pipeline.lowerValue = value;});		
      
    }  
  }

  /**
   * Main.
   */
  public static void main(String... args) {
    if (args.length > 0) {
      configFile = args[0];
    }

    // read configuration
    if (!readConfig()) {
      return;
    }

    // start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
    }

    // start cameras
    List<VideoSource> cameras = new ArrayList<>();
    for (CameraConfig cameraConfig : cameraConfigs) {
      cameras.add(startCamera(cameraConfig));
    }

    // start image processing on camera 0 if present
    if (cameras.size() >= 1) {
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new MyPipeline(), pipeline -> {
        // do something with pipeline results
      });
      /* something like this for GRIP:
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new GripPipeline(), pipeline -> {
        ...
      });
       */
      visionThread.start();
    }

    // loop forever
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }
}
