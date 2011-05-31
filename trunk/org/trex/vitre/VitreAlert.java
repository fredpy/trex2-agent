/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@mbari.org">Frederic Py</a>
 */
import java.awt.Frame;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import java.io.StringWriter;
import java.io.PrintWriter;

import javax.swing.JDialog;
import javax.swing.JTextArea;
import javax.swing.JScrollPane;
import javax.swing.JButton;


class VitreAlert extends JDialog {
    private JTextArea m_longMsg;
    
    public VitreAlert(Frame parent, boolean modal) {
	super(parent, modal);
	initComponents();
    }

    private void closeDialog() {
	setVisible(false);
    }

    public void setExcept(Exception e) {
	StringWriter sw = new StringWriter();
	PrintWriter pw = new PrintWriter(sw);
	e.printStackTrace(pw);
	m_longMsg.setText(sw.toString());
	pack();
    }

    private void initComponents() {
	JButton okButton;
	GridBagConstraints gbc;
	JScrollPane scroll;
	
	setTitle("Exception");
	addWindowListener(new WindowAdapter() {
		public void windowClosing(WindowEvent ev) {
		    closeDialog();
		}
	    });
	getContentPane().setLayout(new GridBagLayout());
	
	m_longMsg = new JTextArea();
	m_longMsg.setEditable(false);
	m_longMsg.setText("");

	scroll = new JScrollPane();
	scroll.setViewportView(m_longMsg);
	gbc = new GridBagConstraints();
	gbc.gridx = 0;
	gbc.gridy = 0;
	gbc.gridwidth = 2;
	gbc.weightx = 1.0;
	gbc.weighty = 1.0;
	gbc.fill = GridBagConstraints.BOTH;
	getContentPane().add(scroll, gbc);

	okButton = new JButton();
	okButton.setText("OK");
	okButton.addActionListener(new ActionListener() {
		public void actionPerformed(ActionEvent ev) {
		    closeDialog();
		}
	    });
	gbc = new GridBagConstraints();
	gbc.gridx = 1;
	gbc.gridy = 1;
	gbc.weightx = 0.5;
	gbc.weighty = 0.0;
	gbc.fill = GridBagConstraints.BOTH;
	getContentPane().add(okButton, gbc);
    }
}	