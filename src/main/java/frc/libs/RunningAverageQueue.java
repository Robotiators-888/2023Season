// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs;

import java.util.ArrayDeque;

/** Class used to keep track of a running total  */
public class RunningAverageQueue {

    int size = 1;
    double total = 0;
    ArrayDeque<Double> queue;

    /**
     * 
     * @param size of queue
     */
    public RunningAverageQueue(int size){
        this.size = size;
        queue = new ArrayDeque<Double>();
    }

    /**
     *  Method that adds values to the queue
     * @param addVal value being added to the queue
     */
    public void insert(double addVal){
        
        if(queue.size() >= size){
            total -= queue.pop().doubleValue();
        }

        queue.add(addVal);
        total += addVal;
    }

    /**
     * 
     * @return Running average of the dataset 
     */
    public double getRunningAverage(){
        
        /*
         * Return the total average var by size var 
         */
        if(queue.size() == 0 ){
            return 0.0;
        } else if(queue.size() < size ){
            return (total/(double)queue.size());
        }else{
            return total/(double)(size);
        }
        
        
    }

    //Unused as of yet, keeping this here until final check over
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
