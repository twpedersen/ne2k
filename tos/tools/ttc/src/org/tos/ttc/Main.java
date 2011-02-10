
package org.tos.ttc;

/*
 * Created on Oct 18, 2004
 * 
 * TODO To change the template for this generated file go to Window -
 * Preferences - Java - Code Style - Code Templates
 */


import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;

import java.awt.BorderLayout;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.*;



public class Main
{

    static private TestConsole testConsole;
    static private JFrame      frame;      // Made public so exit button can



    // access



    public static JFrame getFrame()
    {
        return frame;
    }



    static private String processCommand(String cmd)
    {
        if (cmd.startsWith("INIT")) {
            return testConsole.getSelectedTests();
        }

        if (cmd.startsWith("RESULT=")) {
            testConsole.setResults(cmd.substring(7));
        }

        return "";
    }



    static private void start()
    {
        ServerSocket ss = null;

        try {
            ss = new ServerSocket(8899);
        }
        catch (Exception ex) {
            System.err.println(ex);
            System.exit(-1);
        }

        while (true) {
            try {
                Socket client = ss.accept();
                BufferedReader in = new BufferedReader(
                                                       new InputStreamReader(
                                                                             client
                                                                                     .getInputStream()));
                PrintWriter out = new PrintWriter(new OutputStreamWriter(client
                        .getOutputStream()));
                String line;

                while ((line = in.readLine()) != null) {
                    String ret = processCommand(line);
                    if (!ret.equals("")) {
                        ret += "\n";
                        out.print(ret);
                        out.flush();
                    }
                }

                in.close();
                out.close();
                client.close();
            }
            catch (Exception ex) {
                // Do nothing. We get an exception when Bochs terminates
                // We should probably check for a specific exception such
                // as "Connection reset"
            }
        }
    }



    private static void createAndShowGUI()
    {
        frame = new JFrame("TOS Test Center");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        frame.addWindowListener(new WindowAdapter() {

            public void windowClosing(WindowEvent w)
            {
                javax.swing.table.TableModel model = TestConsole.getTable()
                        .getModel();
                ((TableModel) model).saveSelection();
                frame.setVisible(false);
                frame.dispose();
            }
        });

        JTabbedPane tabbedPane = new JTabbedPane();
        tabbedPane.setOpaque(true);

        Screenshot screenshot = new Screenshot();
        ErrorCode errorCode = new ErrorCode();
        Buttons buttons = new Buttons();

        SourceView sourceView = new SourceView();

        JPanel errorAndSourcePanel = new JPanel();
        errorAndSourcePanel.setLayout(new BoxLayout(errorAndSourcePanel,
                                                    BoxLayout.PAGE_AXIS));
        // JPanel errorAndSourcePanel = new JPanel(new BorderLayout (5,0));
        // errorAndSourcePanel.setBorder(BorderFactory.createCompoundBorder(BorderFactory
        // .createTitledBorder(""), BorderFactory
        // .createEmptyBorder(1, 1, 1, 1)));

        // errorAndSourcePanel.add(errorCode, BorderLayout.NORTH);
        // errorAndSourcePanel.add(sourceView, BorderLayout.SOUTH);
        errorAndSourcePanel.add(errorCode);
        errorAndSourcePanel.add(sourceView);


        testConsole = new TestConsole(screenshot, errorCode, sourceView);
        JPanel testCenterPanel = new JPanel(new BorderLayout(5, 5));
        testCenterPanel.add(buttons, BorderLayout.NORTH);
        testCenterPanel.add(testConsole, BorderLayout.WEST);
        // testCenterPanel.add(errorCode, BorderLayout.SOUTH);
        // testCenterPanel.add(sourceView, BorderLayout.EAST);

        testCenterPanel.add(errorAndSourcePanel, BorderLayout.CENTER);

        tabbedPane.addTab("Test Center", testCenterPanel);

        tabbedPane.addTab("Screenshot", screenshot);

        frame.setContentPane(tabbedPane);
        frame.pack();
        frame.setVisible(true);
    }



    static public void main(String[] args)
    {
        Config.readConfig(args);
        javax.swing.SwingUtilities.invokeLater(new Runnable() {

            public void run()
            {
                createAndShowGUI();
            }
        });
        start();
    }
}