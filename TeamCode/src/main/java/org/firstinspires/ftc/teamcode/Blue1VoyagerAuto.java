package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "Voyagerbot: Blue 1 Auto", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class Blue1VoyagerAuto extends VoyagerBotAuto {
    
    public void initOrientation() {
        startOrientation = Orientation.BLUE_1;
    }
}
