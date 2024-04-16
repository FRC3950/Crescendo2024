package lib.state;


import java.util.Collections;
import java.util.LinkedList;

public class LinkedListGenerator<T> {

    @SafeVarargs
    public final LinkedList<T> generate(T...states) {

        var linkedList = new LinkedList<T>();

        Collections.addAll(linkedList, states);

        return linkedList;
    }
}
