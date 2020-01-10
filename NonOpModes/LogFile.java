package org.firstinspires.ftc.teamcode.NonOpModes;

import java.io.File;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.FileWriter;
import java.io.IOException;

public class LogFile {
    private static String path = "/sdcard/FIRST/java/src/Logging/";
    private String filename;
    private ElapsedTime runtime;
    private FileWriter fw = null;

    public void init(String fn) {
        filename=fn;
        try {
            fw = new FileWriter(path+filename);
              
        } catch (Exception e) {
                e.printStackTrace();

        }
    }

    public void write(String data){
        try {
            fw.write(data);
              
        } catch (Exception e) {
                e.printStackTrace();

        }
        
    }

    public void flush(){
        try {
            fw.flush();
              
        } catch (Exception e) {
                e.printStackTrace();

        }
        
    }
    public void close(){
        try {
            fw.close();
              
        } catch (Exception e) {
                e.printStackTrace();

        }
        
    }

    public void delete() {
        File file = new File(path+filename); 
          
        if(file.delete()) 
        { 
            System.out.println("File deleted successfully"); 
        } 
        else
        { 
            System.out.println("Failed to delete the file"); 
        } 
    }

}