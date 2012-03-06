/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@onaga.shore.mbari.org">Frederic Py</a>
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

import java.awt.Frame;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JDialog;
import javax.swing.JTextField;
import javax.swing.JLabel;
import javax.swing.JButton;

import java.io.IOException;

public class VitreConnect extends JDialog{

    private boolean m_isOK;
    private JTextField m_hostName, m_port;

    public VitreConnect(Frame parent, boolean modal) {
	super(parent, modal);
	initComponents();
    }

    private void initComponents() {
	GridBagConstraints gbc;
	JLabel label;
	JButton button;

	getContentPane().setLayout(new GridBagLayout());

	label = new JLabel();
	label.setText("Host : ");
	gbc = new GridBagConstraints();
	gbc.gridx = 0;
	gbc.gridy = 0;
	gbc.fill = GridBagConstraints.BOTH;
	getContentPane().add(label, gbc);

	m_hostName = new JTextField();
	m_hostName.setText("localhost");
	gbc = new GridBagConstraints();
	gbc.gridx = 1;
	gbc.gridy = 0;
	gbc.gridwidth = 2;
	gbc.weightx = 1.0;
	gbc.fill = GridBagConstraints.BOTH;
	getContentPane().add(m_hostName, gbc);

	label = new JLabel();
	label.setText("Port : ");
	gbc = new GridBagConstraints();
	gbc.gridx = 0;
	gbc.gridy = 1;
	gbc.fill = GridBagConstraints.BOTH;
	getContentPane().add(label, gbc);

	m_port = new JTextField();
	m_port.setText("31415");
	gbc = new GridBagConstraints();
	gbc.gridx = 1;
	gbc.gridy = 1;
	gbc.gridwidth = 2;
	gbc.weightx = 1.0;
	gbc.fill = GridBagConstraints.BOTH;
	getContentPane().add(m_port, gbc);

	button = new JButton();
	button.setText("Cancel");
	button.addActionListener(new ActionListener() {
		public void actionPerformed(ActionEvent ev) {
		    closeDialog(false);
		}
	    });
	gbc = new GridBagConstraints();
	gbc.gridx = 1;
	gbc.gridy = 2;
	gbc.fill = GridBagConstraints.BOTH;
	getContentPane().add(button, gbc);

	button = new JButton();
	button.setText("Connect");
	button.addActionListener(new ActionListener() {
		public void actionPerformed(ActionEvent ev) {
		    closeDialog(true);
		}
	    });
	gbc = new GridBagConstraints();
	gbc.gridx = 2;
	gbc.gridy = 2;
	gbc.fill = GridBagConstraints.BOTH;
	getContentPane().add(button, gbc);
	
	setTitle("Connect to TREX agent");
	addWindowListener(new WindowAdapter() {
		public void windowClosing(WindowEvent ev) {
		    closeDialog(false);
		}
	    });
	setResizable(false);
	pack();
    }

    private void closeDialog(boolean okPressed) {
	m_isOK = okPressed;
	setVisible(false);
    }

    public boolean isReplyOK() {
	return m_isOK;
    }

    public String getHostName() 
	throws IOException {
	if( !m_isOK )
	    throw new IOException("Dialog canceled");
	return m_hostName.getText();
    }

    public int getPort()
	throws IOException {
	if( !m_isOK )
	    throw new IOException("Dialog canceled");
	return Integer.parseInt(m_port.getText());
    }
	
}