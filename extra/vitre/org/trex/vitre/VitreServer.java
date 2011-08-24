/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@mbari1224.shore.mbari.org">Frederic Py</a>
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
package org.trex.vitre;

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
