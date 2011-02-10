
package org.tos.ttc;


import java.awt.BorderLayout;
import java.awt.Dimension;

import javax.swing.BorderFactory;
import javax.swing.JScrollPane;

import java.io.*;
import javax.swing.JPanel;
import javax.swing.text.*;

import org.tos.ttc.*;


import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;



/*
 * Created on 09.01.2005
 * 
 * TODO To change the template for this generated file go to Window -
 * Preferences - Java - Code Style - Code Templates
 */

/**
 * @author Olenka
 * 
 * TODO To change the template for this generated type comment go to Window -
 * Preferences - Java - Code Style - Code Templates
 */
public class SourceView
    extends JPanel
{

    // private JTextPane sourceCode;
    private CustomJTextPane sourceCode;
    private JScrollPane     scrollPanel;
    private int             MAX_SIZE = 62;
    private int             highLine = 0;

    private String current_testcase = TestCases.testCases[0];


    public SourceView()
    {
        this.setLayout(new BorderLayout());
        this.setBorder(BorderFactory.createCompoundBorder(BorderFactory
                .createTitledBorder("Source"), BorderFactory.createEmptyBorder(
                2, 2, 2, 2)));
        this.addComponentListener(new ComponentListener() {

            public void componentHidden(ComponentEvent arg0)
            {
                // TODO Auto-generated method stub

            }



            public void componentMoved(ComponentEvent arg0)
            {
                // TODO Auto-generated method stub

            }



            public void componentResized(ComponentEvent arg0)
            {
                // TODO Auto-generated method stub
                SourceView.this.revalidate();
		setSourceCode(current_testcase, highLine);
            }



            public void componentShown(ComponentEvent arg0)
            {
                // TODO Auto-generated method stub

            }
        });

        // Source Code Description
        // sourceCode = new JTextPane();

        // olga: instead of JTextPane() I am adding
        // new class that inherits from JTextPane, but disables word wrap
        sourceCode = new CustomJTextPane();
        Font f = new Font("Courier New", Font.PLAIN, 14);
        sourceCode.setFont(f);
        // sourceCode.setSize(scrollPanel.getWidth(), scrollPanel.getHeight());
        sourceCode.setEditable(false);
        sourceCode.setBackground(Color.WHITE);
        // sourceCode.setBorder(BorderFactory.createCompoundBorder(BorderFactory
        // .createLoweredBevelBorder(), BorderFactory.createEmptyBorder(2,
        // 2, 2, 2)));


        scrollPanel = new JScrollPane(sourceCode);
        // scrollPanel.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
        scrollPanel.setPreferredSize(new Dimension(500, 320));
        // sourceCode.setBackground(new Color(204, 204, 204));
        // scrollPanel.setSize(this.getWidth(), this.getHeight());
        scrollPanel
                .setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        scrollPanel
                .setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
        // scrollPanel.getAutoscrolls();
        this.add(scrollPanel);

        Style def = StyleContext.getDefaultStyleContext().getStyle(
                StyleContext.DEFAULT_STYLE);

        Style listingStyle = sourceCode.addStyle("listing", def);

        Style listingHighlightedStyle = sourceCode.addStyle(
                "listing-highlighted", listingStyle);
        StyleConstants.setBackground(listingHighlightedStyle, Color.YELLOW);

        Style numberStyle = sourceCode.addStyle("number", def);
        StyleConstants.setBold(numberStyle, true);

        Style numberHighlightedStyle = sourceCode.addStyle(
                "number-highlighted", numberStyle);
        StyleConstants.setBackground(numberHighlightedStyle, Color.YELLOW);
        // setSourceCode("0", -1);
    }



    public void setSourceCode(String testCase, int lineToHighlight)
    {
        highLine = lineToHighlight;
	current_testcase = testCase;
        // trying to fill out an empty JTextPane with white spaces
        // when we first open TTC, so that it won't be so ugly gray
        if (testCase.equals("0")) {
            // sourceCode.setText(" " +
            // " ");
            String text = "";
            for (int i = 0; i < MAX_SIZE; i++) {
                text += " ";
            }
            sourceCode.setText(text);
            sourceCode.setCaretPosition(0);
            return;
        }

        // System.out.println(this.getWidth());
        // System.out.println(sourceCode.getWidth());
	String path = Config.get_tos_dir() + File.separator + "test";
	path += File.separator + testCase + ".c";
        File testfile = new File(path);

        // declared here only to make visible to finally clause
        BufferedReader input = null;
        String line;
        sourceCode.setText("");
        Document doc = sourceCode.getDocument();
        String listingStyle = null;
        try {
            input = new BufferedReader(new FileReader(testfile));
            line = null;
            int lineCounter = 1;
            while ((line = input.readLine()) != null) {
                String pad = lineCounter < 10 ? " " : "";
                listingStyle = "listing";
                String numberStyle = "number";
                if (lineCounter == lineToHighlight) {
                    numberStyle = "number-highlighted";
                    listingStyle = "listing-highlighted";

                    // System.out.println("This is the line that causes error: "
                    // + line);
                }
                doc.insertString(doc.getLength(), pad + lineCounter + ": ",
                        sourceCode.getStyle(numberStyle));
                doc.insertString(doc.getLength(), line + "\n", sourceCode
                        .getStyle(listingStyle));
                lineCounter++;
            }
            // Add the necessary spacing on the last line
            // doc.insertString(doc.getLength(), " " +
            // " \n", sourceCode.getStyle(listingStyle));
            /*
             * int width = this.getWidth() - 24; String text = ""; int spaces =
             * 0; if (scrollPanel.getVerticalScrollBar().isVisible()) { for (;
             * spaces < (width/8 - 2); spaces++) { text += " "; } } else { for (;
             * spaces < width/8; spaces++) { text += " "; } }
             * //System.out.println(spaces); MAX_SIZE = spaces;
             */
            int width = this.getWidth() - 32;
            String text = "";
            int spaces = 0;
            for (; spaces < width / 8; spaces++) {
                text += " ";
            }
            doc.insertString(doc.getLength(), text, sourceCode
                    .getStyle(listingStyle));


            // Scroll visible area to highlighted line
            int y;
            // System.out.println("line to highlight: " + lineToHighlight);
            if (lineToHighlight < 60)
                y = lineToHighlight * 15;
            else
                y = lineToHighlight * 20;
            // System.out.println("y = " + y);
            sourceCode.scrollRectToVisible(new Rectangle(0, y, 320, 320));
        }
        catch (FileNotFoundException ex) {
            try {
                int width = this.getWidth() - 24;
                doc.insertString(doc.getLength(), "**File Not Found**",
                        sourceCode.getStyle(listingStyle));
                // Add the necessary spacing on the last line
                String text = "";
                for (int i = doc.getLength(); i < width / 8; i++) {
                    text += " ";
                }
                // doc.insertString(doc.getLength(), " " +
                // " \n", sourceCode.getStyle(listingStyle));
                doc.insertString(doc.getLength(), text + "\n", sourceCode
                        .getStyle(listingStyle));
            }
            catch (BadLocationException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        catch (IOException ex) {
            ex.printStackTrace();
        }
        catch (BadLocationException ex) {
            ex.printStackTrace();
        }
        finally {
            try {
                if (input != null) {
                    // flush and close both "input" and its underlying
                    // FileReader
                    input.close();
                }
            }
            catch (IOException ex) {
                ex.printStackTrace();
            }
        }
        sourceCode.setCaretPosition(0);
    }
}