package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Timer;

public class Blackbox {

    private static Blackbox instance = null;

    private FileOutputStream file;
    private BufferedWriter fileWriter;

    private ArrayList<String> headers;
    private ArrayList<DoubleSupplier> doubleSuppliers;
    private ArrayList<BooleanSupplier> booleanSuppliers;

    private final long freeSpaceThreshold = 50000000L;

    private Timer elapsedTime = new Timer();

    private Blackbox() {
    }

    /**
     * Get the instance of the Blackbox
     */
    public static Blackbox getInstance() {
        if (instance == null) {
            instance = new Blackbox();
        }
        return instance;
    }

    /**
     * Creates a new blackbox file
     */
    public void startLog() {
        if (fileWriter != null) {
            finishLog();
        }

        headers = new ArrayList<String>();
        doubleSuppliers = new ArrayList<DoubleSupplier>();
        booleanSuppliers = new ArrayList<BooleanSupplier>();

        try {
            File logDirectory = new File("/home/lvuser/blackBoxes");

            // Create the log directory if it doesn't exist
            if (!logDirectory.exists()) {
                logDirectory.mkdir();
            }

            // Clear old files if we're running out of space
            long freeSpace = logDirectory.getFreeSpace();
            if (freeSpace < freeSpaceThreshold) {
                File[] files = logDirectory.listFiles();

                if (files != null) {
                    Arrays.sort(files, Comparator.comparingLong(File::lastModified));

                    for (File file : files) {
                        long fileSize = file.length();

                        if (file.delete()) {
                            freeSpace += fileSize;
                            if (freeSpace >= freeSpaceThreshold) {
                                break;
                            }
                        } else {
                            System.err.println("Could not delete blackbox file: " + file.getName());
                        }
                    }
                } else {
                    System.err.println(
                            "Could not find any blackboxes to delete - check the RIO for excess files. The robot will not save any data from this run.");
                    return;
                }
            }

            // Create the writer for the file
            file = new FileOutputStream(
                    new File("/home/lvuser/blackBoxes/" + LocalDateTime.now(ZoneId.of("America/Edmonton"))
                            + ".blackbox"),
                    false);
            fileWriter = new BufferedWriter(new OutputStreamWriter(file));
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Add a log for the current time
        elapsedTime.reset();
        elapsedTime.start();
        addLog("Time", () -> elapsedTime.get());
    }

    /**
     * Closes the log file
     * 
     * MUST BE CALLED IN DISABLEDINIT
     */
    public void finishLog() {
        // Close the file if it is open
        if (fileWriter != null) {
            try {
                fileWriter.close();
                file.close();
            } catch (IOException e) {
                e.printStackTrace();
            }

            fileWriter = null;
        }

        // Cancel the timer
        elapsedTime.stop();
    }

    /**
     * Adds a logger for doubles
     */
    public void addLog(String name, DoubleSupplier supplier) {
        if (fileWriter != null) {
            headers.add(name);
            doubleSuppliers.add(supplier);

            booleanSuppliers.add(null);

            try {
                if (headers.size() > 1) {
                    fileWriter.write(",");
                }
                fileWriter.write(name);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Adds a logger for booleans
     */
    public void addLog(String name, BooleanSupplier supplier) {
        if (fileWriter != null) {
            headers.add(name);
            booleanSuppliers.add(supplier);

            doubleSuppliers.add(null);

            try {
                if (headers.size() > 1) {
                    fileWriter.write(",");
                }
                fileWriter.write(name);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Gets new values from every logger and writes them to the file
     */
    public void periodic() {
        if (fileWriter != null) {
            try {
                // Add a new line
                fileWriter.write("\n");

                // Iterate through each header
                for (int i = 0; i < headers.size(); i++) {
                    // Add a comma if this isn't the first header
                    if (i != 0) {
                        fileWriter.write(",");
                    }

                    // Get the value and save it
                    if (doubleSuppliers.get(i) != null) {
                        fileWriter.write(String.valueOf(doubleSuppliers.get(i).getAsDouble()));
                    } else if (booleanSuppliers.get(i) != null) {
                        fileWriter.write(String.valueOf(booleanSuppliers.get(i).getAsBoolean()));
                    } else {
                        fileWriter.write("ERROR: No supplier provided");
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
