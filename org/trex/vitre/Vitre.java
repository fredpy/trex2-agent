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

import java.awt.Color;
import java.awt.GradientPaint;
import java.awt.Dimension;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import java.io.IOException;

import java.util.concurrent.Semaphore;

import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.Timer;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class Vitre extends JFrame 
    implements Runnable, ActionListener {

    // Socket management
    private Semaphore m_sem;
    private VitreServer m_server;
    private VitreSocket m_client = null;
    private Thread m_listener;
    private boolean m_running;

    private Timer m_update;

    // GUI
    private TimeLinePanel m_panel;
    private VitreAlert m_alert;

    private TimeLineSet m_timelines;

    private static final int DEFAULT_PORT = 31415; 

    public void run() {
	try {
            System.out.println("Wait for new client.");
	    while( m_running ) {
		Thread.yield();
		m_sem.acquire();
		try {
		    m_client = m_server.accept();
		    setTitle("Vitre ["+m_server.getPort()+"] - connected");
                    System.out.println("New client accepted "+m_client);
		    m_timelines.clear();
		    m_update.start();
		} catch(IOException e) {
		    m_sem.release();
		} finally {
		    Thread.sleep(1000);
		}
	    }
	} catch(InterruptedException ie) {
// 	    System.err.println("interrupted");
	} catch(Exception ex) {
	    m_alert.setExcept(ex);
	    m_alert.setVisible(true);
	} finally {
// 	    System.err.println("finally");
	    try {
		m_server.close();
		m_server = null;
		if( m_client!=null ) {
		    m_client.close();
		    m_client = null;
		}
	    } catch(IOException ioe) {
		m_alert.setExcept(ioe);
		m_alert.setVisible(true);
	    }
	}
    }

    String getDomain(Element e) {
	if( e!=null ) {
	    String kind = e.getTagName();

	    if( kind.equals("value") )
		return e.getAttribute("name");
	    if( kind.equals("symbol") )
		return e.getAttribute("value");
	    if( kind.equals("interval") )
		return "["+e.getAttribute("min")+", "+e.getAttribute("max")+"[";
	    if( kind.equals("set") ) {
		String result = new String("{");
		Element val = (Element)e.getFirstChild();
		
		while( val!=null ) { 
		    result += getDomain(val);
		    val = (Element)val.getNextSibling();
		    if( val!=null )
			result += ", "; 
		}
		return result+"}";
	    }
	    return "#"+kind;
	}
	return null;
    }

    private String buildToolTip(Element msg) {
	String result = new String("<html>");

	result += "<b>"+msg.getAttribute("predicate")+"</b> {";
	Element cstr = (Element)msg.getFirstChild();
	
	if( cstr!=null ) {
	    while( cstr!=null ) {
		Element domain = (Element)cstr.getFirstChild();
		result += "<br>&nbsp;"+cstr.getAttribute("name");
		if( domain !=null ) 
		    result += " = <i>"+getDomain(domain)+"</i>";
		cstr = (Element)cstr.getNextSibling();
	    }
	    result += "<br>";
	}

	return result+"}</html>";
    }
   
    private void treatMessage(Element msg) 
	throws IOException, BadTickInterval, BadToken {
        if( msg.getTagName()=="Tick" ) {
            Tick date = new Tick(Integer.parseInt(msg.getAttribute("value")));
            m_timelines.newTick(date);            
        } else if( msg.getTagName()=="Token" ) {
            Tick date = new Tick(Integer.parseInt(msg.getAttribute("tick"))),
                end;
            String tlName = msg.getAttribute("on");
            String pred =  msg.getAttribute("pred"); 
            
            end = date.next();
            
            m_timelines.newTick(date);
            
            //pred = pred.substring(pred.indexOf('.')+1);
            Token tok = new Token(new TickInterval(date, date), 
                                  pred, 
                                  new TickInterval(end, Tick.PLUS_INF));
            //tok.setToolTip(buildToolTip(msg)); // < hope it works
            m_timelines.add(tlName, tok);
        }
    }

    public void actionPerformed(ActionEvent ev) {
// 	System.out.println("Tick");
	if( null!=m_client ) {
	    try {
		Document message;
		while( m_client.hasMessage() ) {
		    message = m_client.getMessage();
		    try {
			treatMessage(message.getDocumentElement());
		    } catch(Exception e) {
			m_alert.setExcept(e);
			m_alert.setVisible(true);
		    }
		}
	    } catch(IOException e) {
		m_client = null;
		m_update.stop();
		setTitle("Vitre ["+m_server.getPort()+"]");
		m_sem.release();
	    } catch(Exception e) {
		m_alert.setExcept(e);
		m_alert.setVisible(true);
	    }
	}
    }

    public Vitre() throws IOException {
	int port = DEFAULT_PORT;
	m_server = new VitreServer();

	do {
	    try {
		m_server.bind(port);
	    } catch(IOException e) {
		System.err.println("Cannot bind the port "+port+", trying "+(port+1));
		++port;
	    }	    
	} while( !m_server.isBound() );

	m_server.setSoTimeout(500); // timeout of 500ms for accept

	m_sem = new Semaphore(1);
	m_running = true;

	m_timelines = new TimeLineSet();

	initComponents();
	m_listener = new Thread(this);
	m_listener.start();
    }

    private void initComponents() {
	m_panel = new TimeLinePanel(m_timelines);
	m_panel.setBgPaint(new GradientPaint(0, 0, Color.blue, 
					     0, 800, new Color(0.3f, 0.3f, 0.7f)));

	JScrollPane scroll = new JScrollPane();
	scroll.setViewportView(m_panel);
	add(scroll);

	setTitle("Vitre ["+m_server.getPort()+"]");
	addWindowListener(new WindowAdapter() {
		public void windowClosing(WindowEvent ev) {
		    quit(0);
		}
	    });
	setSize(new Dimension(320, 240));
	m_alert = new VitreAlert(this, true);
	// update each 500 milliseconds
	m_update = new Timer(250, this);
    }

    private void quit(int ret) {
	m_running = false;
	while( true ) {
	    try {
		m_listener.interrupt();
		m_listener.join();
		System.exit(0);
	    } catch(InterruptedException e) {}
	}			
    }

    public static void main(String[] argv) throws IOException {
	Vitre window = new Vitre();
	window.setVisible(true);
    }
  
}
