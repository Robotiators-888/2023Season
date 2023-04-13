package frc.robot.Autos;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public class EventMap {
   
    public EventMap() {
    }

    public HashMap<String, Command> buildEventMap(String[] strings, Command[] commands){
        HashMap<String, Command> events = new HashMap<>();

        for(int i = 0; i < strings.length; i++){
            events.put(strings[i], commands[i]);
        }

        return events;
    }


    







}
