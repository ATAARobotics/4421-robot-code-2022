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
import java.util.function.Supplier;

public class Blackbox {

    private static Blackbox instance = null;

    private FileOutputStream file;
    private BufferedWriter fileWriter;

    private ArrayList<String> headers = new ArrayList<String>();
    private ArrayList<DoubleSupplier> doubleSuppliers = new ArrayList<DoubleSupplier>();
    private ArrayList<BooleanSupplier> booleanSuppliers = new ArrayList<BooleanSupplier>();
    private ArrayList<Supplier<String>> stringSuppliers = new ArrayList<Supplier<String>>();

    private final long freeSpaceThreshold = 50000000L;

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
    }

    /**
     * Adds a logger for doubles
     */
    public void addLog(String name, DoubleSupplier supplier) {
        if (fileWriter != null) {
            headers.add(name);
            doubleSuppliers.add(supplier);

            booleanSuppliers.add(null);
            stringSuppliers.add(null);

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
            stringSuppliers.add(null);

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
     * Adds a logger for strings
     */
    public void addLog(String name, Supplier<String> supplier) {
        if (fileWriter != null) {
            headers.add(name);
            stringSuppliers.add(supplier);

            doubleSuppliers.add(null);
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
                    } else if (stringSuppliers.get(i) != null) {
                        fileWriter.write(stringSuppliers.get(i).get());
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
