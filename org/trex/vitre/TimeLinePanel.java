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

import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Insets;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Rectangle;
import java.awt.geom.Rectangle2D;
import java.awt.Paint;
import java.awt.Container;
import java.awt.Image;

import java.awt.AWTEvent;
import javax.swing.ToolTipManager;
import javax.swing.JComponent;
import javax.swing.JViewport;

import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;

public class TimeLinePanel extends JPanel 
    implements TimeLineSet.UpdateListener
{
    private TimeLineSet m_timelines;
    private Tick m_start;

    private Paint m_bgPaint;

    private Image m_buffer = null;
    private boolean m_updateBuff = true;
    private int m_bufferWidth, m_bufferHeight;
    

    public TimeLinePanel(TimeLineSet set) {
	m_timelines = set;
	m_timelines.setUpdateListener(this);
	m_start = Tick.MINUS_INF;
	m_bgPaint = null;
	ToolTipManager.sharedInstance().registerComponent(this);
    }

    public Paint getBgPaint() {
	return m_bgPaint;
    }

    public void setBgPaint(Paint p) {
	m_bgPaint = p;
	m_updateBuff = true;
    }

    public void setStart(Tick date) {
	m_start = date;
	m_updateBuff = true;
    }

    public Tick getStart() {
	return m_start;
    }
    
    public TimeLineSet getTimeLines() {
	return m_timelines;
    }

    public void setTimeLines(TimeLineSet set) {
	m_timelines = set;
	m_timelines.setUpdateListener(this);
	m_updateBuff = true;
	repaint();
    }

    public void updated(TimeLineSet set) {
	if( set==m_timelines ) {
	    m_updateBuff = true;
	    repaint();
	}
    }

    public String getToolTipText(MouseEvent ev) {
	Insets insets = getInsets();
	return m_timelines.getToolTipText(ev.getX()-insets.left,
					  ev.getY()-insets.top);
    }

    public void paintComponent(Graphics g) {
	super.paintComponent(g);

	Graphics2D g2 = (Graphics2D)g.create();
	Dimension size = getSize();
	Insets insets = getInsets();
	Rectangle2D available = new Rectangle2D.Double(insets.left, insets.top,
						       size.getWidth()-insets.left-insets.right,
						       size.getHeight()-insets.top-insets.bottom);
	
	Tick clk = getStart();

	if( null==m_timelines )
	    return;

	if( m_buffer==null 
	    || m_bufferWidth!=available.getWidth()
	    || m_bufferHeight!=available.getHeight() ) {
	    m_bufferWidth = (int)available.getWidth();
	    m_bufferHeight = (int)available.getHeight();
	    m_buffer = createImage(m_bufferWidth, m_bufferHeight);
	    m_updateBuff = true;
	}
	if( m_updateBuff ) {
	    Graphics2D bufferG2 = (Graphics2D)m_buffer.getGraphics();
	    
	    bufferG2.setPaint(m_bgPaint);
	    bufferG2.fill(available);
	    
	    Dimension dim = m_timelines.paint(bufferG2, available, m_start);
	    setPreferredSize(dim);
	    Container parent = getParent();
	
	    m_updateBuff = false;
// 	    if( parent instanceof JComponent ) {
// 		JComponent jparent = (JComponent)parent;
// 		jparent.revalidate();
// 		jparent.scrollRectToVisible(new Rectangle((int)(available.getX()+dim.getWidth()), 
// 							    (int)available.getY(), 0, 0));
// 	    }
	}
	
	g2.drawImage(m_buffer, insets.left, insets.right, this);
    }

}
