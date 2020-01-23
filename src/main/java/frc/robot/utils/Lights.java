package frc.robot.utils;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class Lights extends AddressableLED {

    private AddressableLEDBuffer leds;
    private final int nLEDS = Constants.K_NUM_LEDS;

    public Lights() {
        //LED Setup
        super(Constants.K_VISION_STATUS_LEDS_PORT);
        super.setLength(nLEDS);
        leds = new AddressableLEDBuffer(Constants.K_NUM_LEDS);
        this.setData(leds);
        this.start();
        //Listener setup
        NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable("vision").getEntry("status color");
        entry.addListener(this::listenNetworkControl, EntryListenerFlags.kUpdate);
        entry.setString("25 0 25");
    }

    public void setAllColor(int r, int g, int b){
        System.out.println("Setting all color.");
        for(int i = 0; i < nLEDS; i++){
            leds.setRGB(i,r,g,b);
        }
        this.setData(leds);
    }

    public void listenNetworkControl(EntryNotification note){
        String color = note.value.getString();
        String[] split = color.trim().split(" ");
        int[] rgb = new int[]{0,0,0};
        try{
            for(int i = 0; i <3;i++){
                rgb[i]=Integer.parseInt(split[i]);
            }
            setAllColor(rgb[0], rgb[1], rgb[2]);
        }catch(Exception e){
            System.err.println("Vision Status color parameter malformed");
        }
    }

}