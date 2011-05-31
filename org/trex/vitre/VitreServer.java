/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@mbari1224.shore.mbari.org">Frederic Py</a>
 */

import java.net.SocketException;
import java.net.ServerSocket;
import java.net.SocketAddress;
import java.net.InetSocketAddress;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

public class VitreServer {
    private ServerSocket m_server = null;
    
    public VitreServer() throws IOException {
	m_server = new ServerSocket();
    }

    public VitreServer(SocketAddress addr) 
	throws IOException {
	this();
	bind(addr);
    }

    public VitreServer(int port) 
	throws IOException {
	this();
	bind(port);
    }

    public void finalize() 
	throws Throwable {
	close();
	super.finalize();
    }

    public void bind(int port) throws IOException {
	bind(new InetSocketAddress(port));
    }

    public void bind(SocketAddress addr)
	throws IOException {
	m_server.bind(addr);
    }

    public int getPort() {
	return m_server.getLocalPort();
    }

    public VitreSocket accept() 
	throws IOException, ParserConfigurationException {
	return new VitreSocket(m_server.accept());
    }

    public void close() 
	throws IOException {
	m_server.close();
    }

    public boolean isBound() {
	return m_server.isBound();
    }

    public boolean isClosed() {
	return m_server.isClosed();
    }

    public void setSoTimeout(int timeout) throws SocketException {
	m_server.setSoTimeout(timeout);
    }
  
}
