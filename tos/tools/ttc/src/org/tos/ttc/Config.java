
package org.tos.ttc;

import java.io.*;



public class Config
{
    static private boolean isWindows = false;
    static private boolean isMacOS = false;
    static private boolean isUnix = false;
    static private boolean platform_initialized = false;

    static private String tos_dir   = "../tos";
    static private String bochs_dirs[] = { "../bochs", "../bochs-2.2.5" };
    static private String bochsrc_file = ".bochsrc";
    static private String bochs_dir;

    static public String get_tos_dir() { return tos_dir; }

    static private void find_platform()
    {
	String osname = System.getProperty("os.name");
	if (osname.startsWith("Windows"))
	    isWindows = true;
	else if (osname.equals("Mac OS X"))
	    isMacOS = true;
	else if (osname.equals("Linux") || osname.equals("FreeBSD")
		 || osname.equals("Solaris"))
	    isUnix = true;
	else {
	    System.err.println("Cannot figure out OS from name " + osname);
	    System.exit(-1);
	}
	platform_initialized = true;
    }

    static public void run_bochs()
    {
	if (!platform_initialized)
	    find_platform();

	String cmd;
	if (isWindows) {
	    File bat = new File("run_bochs.bat");
	    if (!bat.exists()) {
		try {
		    bat.createNewFile();

		    FileWriter batwrite = new FileWriter(bat);
		    batwrite.write("@echo off\n");
		    batwrite.write("cd " + bochs_dir + "\n");
		    batwrite.write("bochs.exe -q\n");
		    batwrite.close();
		}
		catch (Exception e) {
		    System.err.println("Can't write run_bochs.bat: "
				       + e.toString());
		    return;
		}
	    }
	    
	    cmd = "cmd /c run_bochs.bat";
	}
	else if (isMacOS)
	    cmd = bochs_dir + "/bochs.scpt";
	else
	    cmd = bochs_dir + "/bochs -q";

	try {
	    java.lang.Runtime.getRuntime().exec(cmd);
	}
	catch (Exception e) {
	    System.err.println("Caught exception while trying to start bochs: "
			       + e.toString());
	}
    }

    static public void readConfig(String[] args)
    {
	// first look for a config.rc file
        try {
            File f = new File("config.rc");
            FileReader in = new FileReader(f);
            BufferedReader br = new BufferedReader(in);
            String home_path = br.readLine();
            tos_dir = home_path + "/tos";
            bochs_dir = home_path + "/bochs";
            in.close();
        }
        catch (Exception ex) {
            // Do nothing. Leave the default values.
        }

	// if command line arguments were specified, read them next
    	if (args.length>0) {
	    // XXX should look for --bochs-dir or some such
	    bochs_dirs = new String[1];
	    bochs_dirs[0] = args[0];
	    bochsrc_file = args[1];
    	}

        // make sure we know where the tos source code is
        try {
            File f = new File(tos_dir + "/VERSION");
            FileReader in = new FileReader(f);
            in.close();
        }
        catch (Exception ex) {
            System.err.println("Could not find TOS.");
            System.exit(-1);
        }

	// now find bochs
	boolean success = false;
	for (int i=0; i<bochs_dirs.length; i++) {
	    String path = bochs_dirs[i] + File.separator + bochsrc_file;
	    try {
		FileReader in = new FileReader(new File(path));
		in.close();
		success = true;
		bochs_dir = bochs_dirs[i];
		break;
	    }
	    catch (Exception ex) {
	    }
	}
	if (!success) {
	    System.err.println("Could not find .bochsrc");
	    //System.exit(-1);
	}
    }
}
