package frc.robot.utils;

public interface UserAnalog {

    public static UserAnalog fromDigital(UserDigital digital, double trueVal, double falseVal){
        return ()->{
            if(digital.get()){
                return trueVal;
            }else{
                return falseVal;
            }
        };
    }

    /**
     * @return value - a [-1,1] value based on the input from the user or controller
     */
    public double get();
}
