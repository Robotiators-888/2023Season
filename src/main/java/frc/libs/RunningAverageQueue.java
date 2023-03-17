// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs;

import java.util.ArrayDeque;

/** Add your docs here. */
public class RunningAverageQueue {

    int size = 1;
    double total = 0;
    ArrayDeque<Double> queue;

    public RunningAverageQueue(int size){
        this.size = size;
        queue = new ArrayDeque<Double>();
    }

    public void insert(double addVal){
        queue.add(addVal);
        total += addVal;
    }

    public double getRunningAverage(){
        
        /*
         * Return the total average var by size var 
         */
        if(queue.size() == 0 ){
            return 0.0;
        } else if(queue.size() < size ){
            return (total/(double)queue.size());
        }else if(queue.size() >= size){
            return shrinkQueue();
        }else{
            return total/(double)(size);
        }
        
        
    }

    private double shrinkQueue(){
        /* 
        keep constrained to size var
        Add to the total and que
        pop last value and subtract from total
        */

        if(queue.size() > size){
            while(queue.size()>size){
                total -= queue.pop().doubleValue();
            }
        }

        return (total / (double)(size));

    }

}
