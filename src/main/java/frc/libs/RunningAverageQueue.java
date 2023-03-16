// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs;

import java.util.ArrayDeque;

/** Add your docs here. */
public class RunningAverageQueue {

    int size = 1;
    double total = 0;
    ArrayDeque<Double> que;

    public RunningAverageQueue(int size){
        this.size = size;
        que = new ArrayDeque<Double>();
    }

    public void insert(double addVal){
        que.add(addVal);
    }

    public double getRunningAverage(){
        
        /*
         * Return the total average var by size var 
         */
        
        return 0.0;
    }

    private void doMath(){
        /* 
        keep constrained to size var
        Add to the total and que
        pop last value and subtract from total

        */
    }

}
