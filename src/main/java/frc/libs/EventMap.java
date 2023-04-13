package frc.libs;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public class EventMap {
   
    public EventMap() {
    }

    /**
     * 
     * @param strings String array that matches with each event name in Path Planner path
     * @param commands Command array that deploys at each event in the path
     * @return A hash map used for events for Path planner
     */
    public static HashMap<String, Command> buildEventMap(String[] strings, Command[] commands){
        HashMap<String, Command> events = new HashMap<>();

        for(int i = 0; i < strings.length; i++){
            events.put(strings[i], commands[i]);
        }

        return events;
    }










}
