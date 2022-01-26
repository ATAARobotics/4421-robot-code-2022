package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;

public class DataLogger {

    private FileOutputStream file;
    private BufferedWriter fileWriter;

    private String fileName;

    /**
     * Create a data logger for a single file
     * 
     * @param filename The name of the file to log to. Do not include a file extension!
     */
    public DataLogger(String fileName) {
        this.fileName = fileName;

        //Clear the file in case it already exists
        setupFile();
    }

    /**
     * Writes a single line of data to the file
     * 
     * @param data The data to save
     */
    public void writeLine(String data) {
        try {
            fileWriter.write(data + System.lineSeparator());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Creates and clears a file, and initializes the file writer
     */
    public void setupFile() {
        try {
            file = new FileOutputStream(new File("/home/lvuser/" + fileName + ".log"), false);
            fileWriter = new BufferedWriter(new OutputStreamWriter(file));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Closes the file. This should be called once no more data will be added.
     * If this doesn't get called, some (by some I mean most) data may (by may I mean will) be lost.
     */
    public void close() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
