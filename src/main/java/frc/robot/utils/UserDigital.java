package frc.robot.utils;

interface UserDigital {
    /**
     * @return value - a boolean value based on the input from the user or
     *         controller.
     * 
     *         This might not necessarily be a button but can be a thresholded axis
     */
    public boolean get();
}