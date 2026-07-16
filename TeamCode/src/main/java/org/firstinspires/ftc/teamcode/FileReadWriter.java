package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;

public class FileReadWriter {
    private File file;
    private String filePath;
    public FileReadWriter(String filePath) throws IOException, SecurityException{
        this.file = new File(filePath);
        this.filePath = filePath;
        file.createNewFile();
    }
    public String writeToFile(String write)throws IOException{
        String out = "";
        try(FileWriter writer = new FileWriter(filePath)){
            writer.write(write);
        }catch(IOException e){
            out = e.getMessage();
        }
        return out;
    }
    public String writeToFile(Double[] write){
        StringBuilder in = new StringBuilder();
        try{
            for (int i = 0; i < write.length; i++){
                if (in.length() != 0){
                    in.append("\n");
                }
                in.append(write[i].toString());
            }
            String out = writeToFile(in.toString());
            if (!out.isEmpty()){
                return out;
            }
        }catch(IOException e){
            return e.getMessage();
        }
        return "";
    }
    public String readFromFile(long timeOut) throws FileNotFoundException {
        StringBuilder out = new StringBuilder();
        try(Scanner reader = new Scanner(file)){
            long startTime = System.currentTimeMillis();
            while (reader.hasNextLine() && (System.currentTimeMillis() - startTime < timeOut || timeOut <= 0)){
                out.append(reader.nextLine());
                out.append("\n");
            }
            if (!(System.currentTimeMillis() - startTime < timeOut || timeOut <= 0)){
                return "timed out";
            }
        }catch(Exception e){
            return e.getMessage();
        }
        return out.toString();
    }
    public String readFromFile() throws FileNotFoundException{
        return readFromFile(0);
    }
    public ArrayList<Double> readToDoubles(long timeOut) throws FileNotFoundException{
        ArrayList<Double> nums = new ArrayList<Double>();
        try(Scanner reader = new Scanner(file)){
            long startTime = System.currentTimeMillis();
            while(reader.hasNextLine() && (System.currentTimeMillis() - startTime < timeOut || timeOut <= 0)){
                nums.add(Double.valueOf(reader.nextLine()));
            }
            if (!(System.currentTimeMillis() - startTime < timeOut || timeOut <= 0)){
                return null;
            }
        }catch(Exception e){
            return null;
        }
        return nums;
    }
    public ArrayList<Double> readToDoubles() throws FileNotFoundException{
        return readToDoubles(0);
    }
}
