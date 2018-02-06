package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "Voyagerbot: Blue 2 Auto", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class Blue2VoyagerAuto extends VoyagerBotAuto {
    
    public void initOrientation() {
        startOrientation = Orientation.BLUE_2;
    }
}
