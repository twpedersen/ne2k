
package org.tos.ttc;


import java.io.*;

import javax.swing.ImageIcon;
import javax.swing.table.AbstractTableModel;



class TableModel
    extends AbstractTableModel
{

    private static final long serialVersionUID = -8826741568473592816L;

    static private String     fileName         = "ttc.save";

    String[]                  columnNames      = {"Test Case", "Run", "Result"};

    Object[]                  runTest;
    Object[]                  result;

    ImageIcon                 emptyIcon;
    ImageIcon                 crossIcon;
    ImageIcon                 checkIcon;



    public TableModel()
    {
        runTest = new Object[TestCases.testCases.length];
        for (int i = 0; i < TestCases.testCases.length; i++) {
            runTest[i] = new Boolean(false);
        }

        emptyIcon = Resource.getImage("empty");
        crossIcon = Resource.getImage("cross");
        checkIcon = Resource.getImage("check");

        result = new Object[TestCases.testCases.length];
        for (int i = 0; i < TestCases.testCases.length; i++) {
            result[i] = emptyIcon;
        }
        restoreSelection();
    }



    // returns how many columns in the table
    public int getColumnCount()
    {
        return columnNames.length;
    }



    // returns how many rows in the table
    public int getRowCount()
    {
        return TestCases.testCases.length;
    }



    public String getColumnName(int col)
    {
        return columnNames[col];
    }



    public Object getValueAt(int row, int col)
    {
        if (col == 1)
            return runTest[row];

        if (col == 2)
            return result[row];

        return TestCases.testCases[row];
    }



    // returns the class that is in the row 0, column c in the table
    public Class getColumnClass(int c)
    {
        return getValueAt(0, c).getClass();
    }



    /*
     * Don't need to implement this method unless your table's editable.
     */
    public boolean isCellEditable(int row, int col)
    {
        return col == 1;
    }



    public void setAllTestCases(boolean value)
    {
        javax.swing.table.TableModel model = TestConsole.getTable().getModel();
        for (int i = 0; i < model.getRowCount(); i++) {
            runTest[i] = new Boolean(value);
            fireTableCellUpdated(i, 1);
        }
    }



    public void setValueAt(Object value, int row, int col)
    {
        if (col == 1) {
            // User clicked checkbox
            // ILIAS
            int start = TestConsole.getTable().getSelectionModel()
                    .getMinSelectionIndex();
            int end = TestConsole.getTable().getSelectionModel()
                    .getMaxSelectionIndex();

            for (int i = start; i <= end; i++) {
                runTest[i] = value;
                fireTableCellUpdated(i, 1);
            }
        }

        if (col == 2) {
            // Test results come back
            if (value instanceof Boolean) {
                if (((Boolean) value).booleanValue())
                    result[row] = checkIcon;
                else
                    result[row] = crossIcon;
            }
            else {
                result[row] = emptyIcon;
            }
            fireTableCellUpdated(row, 2);
        }
    }



    public void reset()
    {
        for (int i = 0; i < TestCases.testCases.length; i++) {
            runTest[i] = "";
        }
    }



    public void restoreSelection()
    {
        File inputFile = new File(fileName);
        FileReader in = null;
        try {
            in = new FileReader(inputFile);
            BufferedReader br = new BufferedReader(in);
            String s = new String();
            s = br.readLine();
            in.close();

            for (int i = 0; i < TestCases.testCases.length; i++) {
                char c = s.charAt(i);
                if (c == '1') {
                    runTest[i] = new Boolean(true);
                    fireTableCellUpdated(i, 1);
                }
                else {
                    runTest[i] = new Boolean(false);
                    fireTableCellUpdated(i, 1);
                }
            }
        }
        catch (FileNotFoundException e) {
            // TODO Auto-generated catch block
            // e.printStackTrace();
            // Do nothing. This is ok for the first time
            // we run the console, in which case the file
            // doesn't exist. In this case, we will do a
            // clean start with no checkboxes.
        }
        catch (IOException e1) {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
    }



    public void saveSelection()
    {
        File outputFile = new File(fileName);
        FileWriter out = null;
        try {
            out = new FileWriter(outputFile);

            for (int i = 0; i < runTest.length; i++) {
                boolean value = ((Boolean) runTest[i]).booleanValue();
                if (value) {
                    out.write('1');
                }
                else {
                    out.write('0');
                }
            }
            out.close();
        }
        catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

}