package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimeOfFlightSubsystem extends SubsystemBase{
    
    public static TimeOfFlight TimeOfFlight1 = new TimeOfFlight(24);
    public static TimeOfFlight TimeOfFlight2 = new TimeOfFlight(25);

    public TimeOfFlightSubsystem(){
        TimeOfFlight1.setRangingMode(RangingMode.Short, 25);
        TimeOfFlight2.setRangingMode(RangingMode.Short, 25);
    }



}
