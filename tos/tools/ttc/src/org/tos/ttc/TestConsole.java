package org.tos.ttc;


import javax.swing.*;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import javax.swing.table.TableColumn;
import java.awt.BorderLayout;
import java.awt.Dimension;


class TableSelectionHandler
    implements ListSelectionListener
{

    Screenshot screenshot;
    SourceView source;
    JTable     table;
    public static int select = 0;



    public TableSelectionHandler(Screenshot screenshot, SourceView source,
            JTable table)
    {
        this.screenshot = screenshot;
        this.source = source;
        this.table = table;
    }



    //olga
    //here I should add display of a testCase code that is being run
    public void valueChanged(ListSelectionEvent e)
    {
        if (e.getValueIsAdjusting())
            return;
        ListSelectionModel lsm = (ListSelectionModel) e.getSource();
        if (!lsm.isSelectionEmpty()) {
            int selection = lsm.getMinSelectionIndex();
            screenshot.setScreenshot(TestCases.testCases[selection]);

            //If user selects a particular testCase for view, we display it!
            int result;
            result = table.getSelectedColumn();
            if (result == 0) {
                source.setSourceCode(TestCases.testCases[selection], 0);
                select = selection;
            }

        }
    }
}



public class TestConsole
    extends JPanel
{

    private static JTable    table;

    final private Screenshot screenshot;
    final private ErrorCode  errorCode;
    //adding SourceView variable so that if error occurs, test case code is
    // displayed.
    final private SourceView sourceview;



    public TestConsole(Screenshot screenshot, ErrorCode errorCode,
            SourceView sourceview)
    {
        this.screenshot = screenshot;
        this.errorCode = errorCode;
        this.sourceview = sourceview;

        this.setLayout(new BorderLayout());
        this.setBorder(BorderFactory.createCompoundBorder(BorderFactory
                .createTitledBorder("Test Center"), BorderFactory
                .createEmptyBorder(5, 5, 5, 5)));

        table = new JTable(new TableModel());
        table.setPreferredScrollableViewportSize(new Dimension(300, 448));
        
        table.setPreferredSize(new Dimension(300, 448));
        
        table.getTableHeader().setReorderingAllowed(false);
        table.getTableHeader().setResizingAllowed(false);
        
        //Olga: trying to add horizontal scrollbar to appear when 
        //user resizes the window.
       // table.setAutoResizeMode(JTable.AUTO_RESIZE_OFF); 

        
        //table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        table.setSelectionMode(ListSelectionModel.SINGLE_INTERVAL_SELECTION); //ADDED

        //olga added cell selection
        table.setCellSelectionEnabled(true);

        ListSelectionModel rowSM = table.getSelectionModel();
        rowSM.addListSelectionListener(new TableSelectionHandler(screenshot,
                                                                 sourceview,
                                                                 table));
        TableColumn column = table.getColumnModel().getColumn(1);
        column.setPreferredWidth(7);
        column = table.getColumnModel().getColumn(2);
        column.setPreferredWidth(7);

        JScrollPane scrollPane = new JScrollPane(table);
        
        // Olga: added scrollbars to make it resizable
        scrollPane.setHorizontalScrollBarPolicy
						(JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
        scrollPane.setVerticalScrollBarPolicy
						(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED);
        
        scrollPane.setPreferredSize(new Dimension(303, 448));
        add(scrollPane);
    }



    public String getSelectedTests()
    {
        String tests = "";
        int numRows = table.getRowCount();
        javax.swing.table.TableModel model = table.getModel();

        for (int i = 0; i < numRows; i++) {
            tests += ((Boolean) model.getValueAt(i, 1)).booleanValue() ? "1"
                    : "0";
        }
        return tests;
    }



    public void setResults(String result)
    {
        String args[] = result.split(",");
        String errno = args[0];

        int lineno = Integer.parseInt(args[1]);

        String test_results = args[2];
        javax.swing.table.TableModel model = table.getModel();

        errorCode.setErrorCode(errno);
        for (int i = 0; i < test_results.length(); i++) {
            char r = test_results.charAt(i);
            if (r == 'P')
                model.setValueAt(new Boolean(true), i, 2);
            else if (r == 'F') {
                sourceview.setSourceCode(TestCases.testCases[i], lineno);
                model.setValueAt(new Boolean(false), i, 2);
                screenshot.setScreenshot(TestCases.testCases[i]);
            }
            else
                model.setValueAt("", i, 2);
        }
    }



    /**
     * @return Returns the table.
     */
    public static JTable getTable()
    {
        return table;
    }
}