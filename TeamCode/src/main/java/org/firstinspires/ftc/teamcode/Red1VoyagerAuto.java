package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "Voyagerbot: Red 1 Auto", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class Red1VoyagerAuto extends VoyagerBotAuto {
    
    public void initOrientation() {
        startOrientation = Orientation.RED_1;
    }
}
