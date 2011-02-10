package org.tos.ttc;

import java.io.*;
import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.net.URL;

import javax.swing.ImageIcon;
import javax.xml.transform.*;
import javax.xml.transform.stream.StreamResult;
import javax.xml.transform.stream.StreamSource;



public class Resource
{

    static public ImageIcon getImage(String name)
    {
        URL url = Resource.class.getClassLoader().getResource("images/" + name + ".jpg");
        if (url == null) {
            System.err.println("Could not find directory ttc/images");
            System.exit(-1);
        }
        return new ImageIcon(url);
    }



    static public String getErrorCode(String errno)
    {
        if (errno.equals("0"))
            return "";

        String xml_file = Config.get_tos_dir() + "/test/messages.xml";
        InputStream xsl_input = Resource.class.getClassLoader().getResourceAsStream("xml/messages.xsl");

        String html = "";

        try {
            Source xmlSource = new StreamSource(new FileInputStream(xml_file));
            Source xsltSource = new StreamSource(xsl_input);
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            Result result = new StreamResult(out);

            TransformerFactory transFactory = TransformerFactory.newInstance();
            Transformer trans = transFactory.newTransformer(xsltSource);

            trans.setParameter("errno", errno);
            trans.transform(xmlSource, result);
            html = out.toString();
        }
        catch (Exception ex) {
            System.err.println(ex);
            html = "";
        }

        return html;
    }
}