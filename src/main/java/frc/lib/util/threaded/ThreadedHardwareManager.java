package frc.lib.util.threaded;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ThreadedHardwareManager {

    public static final ThreadedHardwareManager INSTANCE = new ThreadedHardwareManager();

    private ExecutorService service;

    private ThreadedHardwareManager() {
        service = Executors.newFixedThreadPool(10);
    }

    public void addThreadedHardwareUpdate(Runnable update) {
        service.submit(() -> {
            update.run();
            ThreadedHardwareManager.INSTANCE.addThreadedHardwareUpdate(update);
        });
    }

}
