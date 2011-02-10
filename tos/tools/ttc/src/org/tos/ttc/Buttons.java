
package org.tos.ttc;

/*
 * Created on 19.12.2004
 * 
 * TODO To change the template for this generated file go to Window -
 * Preferences - Java - Code Style - Code Templates
 */
/**
 * Author: Ilias Konstantinidis
 */

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.JButton;
import javax.swing.JPanel;
import java.awt.BorderLayout;



public class Buttons
    extends JPanel
{

    /**
     * 
     */
    private static final long serialVersionUID = 6661044351222563129L;
    private JButton           clearAll;
    // private JButton cygwin;
    private JButton           bochs;
    private JButton           selectAll;
    private JButton           exit;
    private Box               box;



    /*
     * private FlowLayout topfLayout; private FlowLayout leftfLayout; private
     * FlowLayout rightfLayout;
     */

    public Buttons()
    {

        this.setLayout(new BorderLayout());
        clearAll = new JButton("Clear All");
        selectAll = new JButton("Select All");
        // cygwin = new JButton("Cygwin");
        bochs = new JButton("Bochs");
        exit = new JButton("Exit");

        box = Box.createHorizontalBox();

        /*
         * topfLayout = new FlowLayout(); leftfLayout = new FlowLayout();
         * rightfLayout = new FlowLayout();
         * 
         * this.setLayout(leftfLayout);
         * topfLayout.setAlignment(FlowLayout.LEFT);
         * leftfLayout.setAlignment(FlowLayout.LEFT);
         * rightfLayout.setAlignment(FlowLayout.RIGHT);
         */
        clearAll.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent ev)
            {
                javax.swing.table.TableModel model = TestConsole.getTable()
                        .getModel();
                ((TableModel) model).setAllTestCases(false);
            }
        });

        selectAll.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent ev)
            {
                javax.swing.table.TableModel model = TestConsole.getTable()
                        .getModel();
                ((TableModel) model).setAllTestCases(true);
            }
        });

        bochs.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent ev)
            {
                System.out.println("Bochs Button Pressed!");
		Config.run_bochs();
            }
        });

        exit.addActionListener(new ActionListener() {

            public void actionPerformed(ActionEvent ev)
            {
                javax.swing.table.TableModel model = TestConsole.getTable()
                        .getModel();
                ((TableModel) model).saveSelection();
                Main.getFrame().setVisible(false);
                Main.getFrame().dispose();
                System.exit(1);
            }
        });

        box.add(clearAll);
        box.add(selectAll);
        box.add(bochs);
        box.add(Box.createHorizontalGlue());
        box.add(Box.createHorizontalStrut(10));
        box.add(exit);
        this.add(box);

        /*
         * add(clearAll); add(selectAll); // add(cygwin); add(bochs); add(exit);
         */
    }
}
