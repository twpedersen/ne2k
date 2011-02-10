package org.tos.ttc;

import java.awt.BorderLayout;
import java.awt.Dimension;
import javax.swing.*;
import javax.swing.BorderFactory;
import javax.swing.JPanel;

public class ErrorCode
    extends JPanel
{

    private JEditorPane errorCodeDescription;
    //olga: trying to make ErrorDescription scalable
    private JScrollPane scrollpane;

    public ErrorCode()
    {
    	this.setLayout(new BorderLayout());
        this.setBorder(BorderFactory.createCompoundBorder(BorderFactory
                .createTitledBorder("Error Description"), BorderFactory
                .createEmptyBorder(2, 2, 2, 2)));

        // Error Code Description
        errorCodeDescription = new JEditorPane();
        errorCodeDescription.setContentType("text/html");
        errorCodeDescription.setEditable(false);
       // errorCodeDescription.setBorder(BorderFactory.createCompoundBorder(
        //        BorderFactory.createLoweredBevelBorder(), BorderFactory
        //                .createEmptyBorder(5, 5, 5, 5)));
        scrollpane = new JScrollPane (errorCodeDescription);
        scrollpane.setPreferredSize(new Dimension(500, 160));
        this.add(scrollpane);
        setErrorCode("0");
    }



    public void setErrorCode(String errno)
    {
        errorCodeDescription.setText(Resource.getErrorCode(errno));
        errorCodeDescription.setCaretPosition(0);
    }
}