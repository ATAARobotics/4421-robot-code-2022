package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Blackbox {

    private static Blackbox instance = null;

    private FileOutputStream file;
    private BufferedWriter fileWriter;

    private ArrayList<String> headers = new ArrayList<String>();
    private ArrayList<DoubleSupplier> doubleSuppliers = new ArrayList<DoubleSupplier>();
    private ArrayList<BooleanSupplier> booleanSuppliers = new ArrayList<BooleanSupplier>();

    private Blackbox() {
    }

    public static Blackbox getInstance() {
        if (instance == null) {
            instance = new Blackbox();
        }
        return instance;
    }

    public void startLog() {
        if (fileWriter != null) {
            finishLog();
        }

        try {
            // TODO Delete old logs if storage is getting too full

            if (!new File("/home/lvuser/blackBoxes").exists()) {
                new File("/home/lvuser/blackBoxes").mkdir();
            }

            file = new FileOutputStream(
                    new File("/home/lvuser/blackBoxes/" + LocalDateTime.now(ZoneId.of("America/Edmonton")) + ".log"),
                    false);
            fileWriter = new BufferedWriter(new OutputStreamWriter(file));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void finishLog() {
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

    public void addLog(String name, DoubleSupplier supplier) {
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

    public void addLog(String name, BooleanSupplier supplier) {
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

    public void periodic() {
        try {
            fileWriter.write("\n");
            for (int i = 0; i < headers.size(); i++) {
                if (i != 0) {
                    fileWriter.write(",");
                }

                if (doubleSuppliers.get(i) != null) {
                    fileWriter.write(String.valueOf(doubleSuppliers.get(i).getAsDouble()));
                } else if (booleanSuppliers.get(i) != null) {
                    fileWriter.write(String.valueOf(booleanSuppliers.get(i).getAsBoolean()));
                } else {
                    fileWriter.write("ERROR");
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
