/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@mbari.org">Frederic Py</a>
 */
package org.trex.vitre;

import java.net.Socket;
import java.net.UnknownHostException;
import java.net.ConnectException;

import java.io.DataInputStream;
import java.io.StringReader;
import java.io.IOException;

import javax.xml.parsers.DocumentBuilder; 
import javax.xml.parsers.DocumentBuilderFactory;  
import javax.xml.parsers.FactoryConfigurationError;  
import javax.xml.parsers.ParserConfigurationException;

import org.xml.sax.InputSource;
import org.xml.sax.ErrorHandler;
import org.xml.sax.SAXException;  
import org.xml.sax.SAXParseException;

import org.w3c.dom.Document;
import org.w3c.dom.DOMException;

public class VitreSocket
{
    private Socket m_socket = null;
    private DataInputStream m_dis = null;
    private DocumentBuilder m_parser;

    public VitreSocket() 
	throws ParserConfigurationException {
	m_parser = DocumentBuilderFactory.newInstance().newDocumentBuilder();
	
	m_parser.setErrorHandler(new ErrorHandler() {
		// ignore fatal errors (an exception is guaranteed)
		public void fatalError(SAXParseException e)
		    throws SAXException {
		    System.out.println("** Fatal Error"
				       + ", line " + e.getLineNumber()
				       + ", uri " + e.getSystemId());
		    System.out.println("   " + e.getMessage());
		}
		
		// treat validation errors as fatal
		public void error(SAXParseException e)
		    throws SAXParseException {
		    System.out.println("** Error"
				       + ", line " + e.getLineNumber()
				       + ", uri " + e.getSystemId());
		    System.out.println("   " + e.getMessage());
		}
		
		// dump warnings too
		public void warning(SAXParseException err)
		    throws SAXParseException {
		    System.out.println("** Warning"
				       + ", line " + err.getLineNumber()
				       + ", uri " + err.getSystemId());
		    System.out.println("   " + err.getMessage());
		}
	    }); 
    }
    public VitreSocket(String hostName, int port)
	throws UnknownHostException, IOException, ParserConfigurationException {
	this();
	connect(hostName, port);
    }

    protected VitreSocket(Socket sock) 
	throws ParserConfigurationException, IOException {
	this();
	m_socket = sock;
	m_dis = new DataInputStream(m_socket.getInputStream());
    }

    public void finalize() 
	throws Throwable {
	close();
	super.finalize();
    }

    public void connect(String hostName, int port) 
	throws UnknownHostException, IOException {
	if( isConnected() )
	    throw new ConnectException("Already connected.");
	Socket tmp = new Socket(hostName, port);
	m_dis = new DataInputStream(tmp.getInputStream());
	m_socket = tmp;
    }
    
    public void close() 
	throws IOException {
	if( isConnected() ) {
	    Socket tmp = m_socket;
	    m_dis = null;
	    m_socket = null;
	    tmp.close();
	    System.out.println("Socket closed");
	}//  else
// 	    System.out.println("Socket already closed");
    }

    public boolean isConnected() {
	return null!=m_socket;
    }
    
    private final int getInt() 
	throws IOException {
	return m_dis.readInt();
    }

    private final String getString() 
	throws IOException {
	int size = getInt();
	
	if( size>0 ) {
	    byte[] buff = new byte[size];
	    m_dis.read(buff);
	    return new String(buff);
	} else 
	    return new String();
    }

    public boolean hasMessage()
	throws IOException {
	if( !isConnected() )
	    return false;
	return m_dis.available()>0;
    }

    public Document getMessage() 
	throws IOException, SAXException {
	String xmlMessage = getString();

        //System.err.println("Message : \""+xmlMessage+"\"");
	
	if( xmlMessage.length()==0 )
	    throw new IOException("Disconnection");
	
	return m_parser.parse(new InputSource(new StringReader(xmlMessage)));
    }
    
} // VitreSocket
