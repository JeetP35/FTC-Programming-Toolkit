package org.firstinspires.ftc.teamcode;
import java.util.HashMap;

public class CommandSystem
{
    // stores a state correlating to a subsystem
    HashMap<Character,Boolean> completedMap = new HashMap<Character,Boolean>();
    
    public boolean GetBoolsCompleted()
    {
        // for every key in the dictionary
        for (boolean value : completedMap.values())
        {
            if (!value)
            {
                return false;
            }
        }
        return true;
    }
    
    // clear entire map for next command
    public void ResetMap()
    {
        completedMap.clear();
    }       
    
    // set a given element's completion state to false
    public void SetElementFalse(char element)
    {
        completedMap.put(element,false);
    }

    // set a given element's completion state to true
    public void SetElementTrue(char element)
    {
        completedMap.put(element,true);
    }

    // gets the state of an element
    public boolean GetElementState(char element)
    {
        return completedMap.get(element);
    }
    
    // returns the map itself
    public HashMap<Character,Boolean> GetMap()
    {
        return completedMap;
    }
}
