package frc.robot.vision;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.SerialPort;
import org.json.simple.parser.JSONParser;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.JevoisVisionSubsystem;
import org.json.simple.JSONObject;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This controls all vision actions, including vision updates, capture, and
 * interfacing with the Android phone with Android Debug Bridge. It also stores
 * all VisionUpdates (from the Android phone) and contains methods to add
 * to/prune the VisionUpdate list. Much like the subsystems, outside methods get
 * the VisionServer instance (there is only one VisionServer) instead of
 * creating new VisionServer instances.
 * 
 * @see VisionUpdate.java
 */

public class JevoisVisionServer {
    double lastMessageReceivedTime = 0;
    private boolean m_use_java_time = false;
    private SerialPort visionCam;
    boolean visionCamHasTarget;
    double visionCamZPosition;
    double visionCamYPosition;
    double visionCamDeltaTime;
    private boolean isConnected;
    public boolean attemptConnections = false;
    private JevoisVisionSubsystem visionSubsystem;

    public boolean getisConnected(){
        return isConnected;
    }

    public JevoisVisionServer(JevoisVisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        new Thread(new VisionServerThread()).start();
    }

    public SerialPort getVisionCam() {
        return visionCam;
    }

    private class VisionServerThread implements Runnable {
        int dashboardCounter = 0;
        String lastDashboardMessage = "";
        boolean attemptingConnection = false;

        private VisionServerThread() {
            isConnected = attemptJevoisConnection();
        }

        @Override
        public void run() {
            for(;;){
                String dashboardMessage = "";
                if(isConnected){
                    try{
                        dashboardMessage = "visionCamAvailable == true. Handling message";
                        String visionTargetPositions_Raw = visionCam.readString();
                        //int visionLength = visionTargetPositions_Raw.length();
                        handleMessage(visionTargetPositions_Raw, getTimestamp());
                    } catch(Exception e){
                        //Camera may not have sent anything or become disconnected
                        dashboardMessage = "isConnected, but error thrown on handle";
                    }
                } else {
                    dashboardMessage = "visionCamAvailable == false. Lost connection?";
                    //isConnected = attemptJevoisConnection();
                    if(attemptConnections && !attemptingConnection){
                        attemptJevoisConnection();
                    }
                }
                if(lastDashboardMessage != dashboardMessage){
                    if(System.currentTimeMillis() % 100 == 0){
                        SmartDashboard.putString("JevoisVisionServerOutput", dashboardMessage);
                    }
                }
                lastDashboardMessage = dashboardMessage;
                try{
                    SmartDashboard.putBoolean("isJevoisConnected", isConnected);
                    Thread.sleep(20);
                }
                catch(Exception e){
                }
           }
        }

        private boolean SafeToParse(String message) {
            if (message.length() > 1) {
                return message.charAt(0) == '{' && message.charAt(message.length() - 3) == '}'
                        && message.contains("{\"DeltaTime\"") && message.contains("\"Y\"") && message.contains("\"Z\"");
            } else {
                return false;
            }
        }

        public void handleMessage(String message, double timestamp) {
            // m_VisionSubsystem.ringLightOn();
            dashboardCounter++;
            String visionTargetPositions_Raw = message; // message should be the raw visionCam.readString()
            if (message == null || message.length() <= 0) {
                return;
            }
            // System.out.println(visionTargetPositions_Raw);
            /*
             * if(!SafeToParse(visionTargetPositions_Raw)){ return; }
             */
            if (dashboardCounter >= 10) {
                SmartDashboard.putString("Vis_TargetString",
                visionTargetPositions_Raw);
            }

            String[] visionTargetLines = visionTargetPositions_Raw.split("\n");
            ArrayList<TargetInfo> targetInfoArray = new ArrayList<>();
            for (int i = visionTargetLines.length - 1; i >= 0; i--) {
                boolean isValid = false;
                String thisTargetLine = visionTargetLines[i];
                try {
                    // if(SafeToParse(thisTargetLine, true)){
                    JSONParser parser = new JSONParser();
                    JSONObject j = (JSONObject) parser.parse(thisTargetLine);
                    visionCamDeltaTime = Double.parseDouble((String) j.get("DeltaTime"));
                    visionCamYPosition = Double.parseDouble((String) j.get("Y"));
                    visionCamZPosition = Double.parseDouble((String) j.get("Z"));
                    if (visionCamYPosition == 0.0 && visionCamZPosition == 0.0) {
                        isValid = false;
                    } else {
                        isValid = true;
                    }
                    // }
                } catch (Exception e) {
                    // System.err.println("Parse error: "+e.toString());
                }
                if (isValid) {
                    TargetInfo targetInfo = new TargetInfo(visionCamYPosition, visionCamZPosition);
                    targetInfoArray.add(targetInfo);
                    SmartDashboard.putString("Vis_TargetProcessed", thisTargetLine);
                }
            }
            // System.out.println("TargetInfoArray.size(): "+targetInfoArray.size());
            if (targetInfoArray.size() > 0) {
                if (dashboardCounter >= 10) {
                    //SmartDashboard.putString("JevoisVisionServerUpdate", "Sent: "+sentTimes);
                }
                visionSubsystem.gotUpdate(
                        new JevoisVisionUpdate(Timer.getFPGATimestamp() - visionCamDeltaTime, targetInfoArray));

                // mRobotState.addVisionUpdate(Timer.getFPGATimestamp()-visionCamDeltaTime, targetInfoArray);
            }
            if (dashboardCounter >= 10) {
                dashboardCounter = 0;
            }
        }

        public boolean attemptJevoisConnection() {
            int connectionAttempts = 0;
            int MAX_ATTEMPTS = 3;
            boolean connected = false;
            attemptingConnection = true;
            while (!connected && (++connectionAttempts <= MAX_ATTEMPTS || attemptConnections)) {
                try{
                    System.out.println("Connecting to JeVois...");
                    visionCam = new SerialPort(VisionConstants.kCameraBaudRate, SerialPort.Port.kUSB1);
                    if(visionCam != null){
                        visionCam.setTimeout(5);
                        connected = true;
                        System.out.println(">>>CONNECTED TO JEVOIS<<<");
                    }
                    else{
                        try {
                            Thread.sleep(10000);
                        } catch (InterruptedException e) {
                            // don't care if our sleep gets interrupted
                            System.err.println("Don't bother me, I'm sleeping");
                        }
                    }
                } catch(Exception e){
                    System.err.println("*Failed to connect to JeVois*" + (connectionAttempts <= MAX_ATTEMPTS ? " \nTrying "+(MAX_ATTEMPTS-connectionAttempts)+" more times" : ""));
                }
            }
            attemptingConnection = false;
            return connected;
        }
    }

    private double getTimestamp() {
        if (m_use_java_time) {
            return System.currentTimeMillis();
        } else {
            return Timer.getFPGATimestamp();
        }
    }
}
