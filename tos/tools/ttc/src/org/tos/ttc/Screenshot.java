
package org.tos.ttc;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.ImageIcon;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;



public class Screenshot
    extends JPanel
    implements ActionListener
{

    /**
     * 
     */
    private static final long serialVersionUID = -4145843918873014038L;
    private JLabel            screenshot;
    private JComboBox         testCases;



    public Screenshot()
    {
        this.setLayout(new BorderLayout());
        this.setBorder(BorderFactory.createCompoundBorder(BorderFactory
                .createTitledBorder("Screenshot"), BorderFactory
                .createEmptyBorder(5, 5, 5, 5)));

        // Top panel
        JPanel topPanel = new JPanel();
        topPanel.add(new JLabel("Select test case: "));
        testCases = new JComboBox(TestCases.testCases);
        testCases.addActionListener(this);
        topPanel.add(testCases);
        this.add(topPanel, BorderLayout.LINE_START);

        // Actual screenshot
        screenshot = new JLabel("");
        screenshot.setHorizontalAlignment(JLabel.CENTER);
        screenshot.setVerticalAlignment(JLabel.CENTER);
        screenshot.setVerticalTextPosition(JLabel.CENTER);
        screenshot.setHorizontalTextPosition(JLabel.CENTER);
        screenshot.setBorder(BorderFactory.createCompoundBorder(BorderFactory
                .createLoweredBevelBorder(), BorderFactory.createEmptyBorder(
                10, 5, 10, 5)));
        screenshot.setText("");
        setScreenshot(TestCases.testCases[0]);
        this.add(screenshot, BorderLayout.PAGE_END);
    }



    public void setScreenshot(String testCase)
    {
        testCases.setSelectedItem(testCase);
        ImageIcon i = Resource.getImage(testCase);
        screenshot.setIcon(i);
    }



    public void actionPerformed(ActionEvent e)
    {
        // Event could only have come from the JComboBox
        String str = (String) testCases.getSelectedItem();
        setScreenshot(str);
    }
}