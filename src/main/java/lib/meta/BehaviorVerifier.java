package lib.meta;

import edu.wpi.first.wpilibj2.command.Command;

import java.lang.reflect.Method;

public class BehaviorVerifier {

    // Implement/use at your own risk
    // Adds lots of unnecessary runtime load
    public static void checkConformsBehavior(Method cmdMethod, Command cmdObject, CommandType commandType) throws RuntimeException, NoSuchMethodException {
        Class<?> clazz = cmdObject.getClass();

        if(cmdMethod.isAnnotationPresent(CommandBehavior.class) && cmdMethod.getReturnType().equals(Command.class)){
            CommandBehavior annotation = cmdMethod.getAnnotation(CommandBehavior.class);

            Method superInit = clazz.getSuperclass().getMethod("initialize");
            Method superExec = clazz.getSuperclass().getMethod("execute");
            Method superIsFinished = clazz.getSuperclass().getMethod("isFinished");

            Method init = clazz.getMethod("initialize");
            Method exec = clazz.getMethod("execute");
            Method isFinished = clazz.getMethod("isFinished");

            switch(annotation.behavior()){
                case INITIALIZE -> {
                    if(!superExec.equals(exec) || superInit.equals(init)){
                        throw new RuntimeException();
                    }
                }

                case SUSTAINED_EXECUTE -> {
                    if(superExec.equals(exec)){
                        throw new RuntimeException();
                    }
                }

                case INSTANT -> { // Not ideal, but checking if overridden isFinished always returns true is very complicated
                    if(!superExec.equals(exec) || superIsFinished.equals(isFinished)){
                        throw new RuntimeException();
                    }
                }
            }
        }
    }

    public static void conformsEndType(Method cmdMethod, Command cmdObject, EndType endType) throws RuntimeException, NoSuchMethodException {
        //TODO
    }
}
