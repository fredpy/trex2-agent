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

import java.awt.geom.Rectangle2D;

public class Token {
    private TickInterval m_start, m_end;
    private String m_shortName, m_tooltip;

    private static void checkValidity(TickInterval low, TickInterval hi) throws BadToken {
	int comp = low.getLowerBound().compareTo(hi.getLowerBound());
	
	if( comp>0 || low.getUpperBound().compareTo(hi.getUpperBound())>0 )
	    throw new BadToken("Invalid token bounds : "+low+" - "+hi);
    }

    public Token(TickInterval low, String name, TickInterval high) throws BadToken {
	checkValidity(low, high);
	m_shortName = name;
	m_start = low;
	m_end = high;
	m_tooltip = toString();
    }

    public Token(String name) {
	m_start = new TickInterval();
	m_end = new TickInterval();
	m_shortName = name;
	m_tooltip = name;
    }

    public boolean isFailed() {
	return m_shortName.equals("Failed");
    }
    public boolean isUndefined() {
	return m_shortName.equals("undefined");
    }

    public void setToolTip(String message) {
	m_tooltip = message;
    }

    private Rectangle2D m_box = null;

    public boolean above(int x) {
	return m_box==null || m_box.getX()>x;
    }

    public boolean below(int x) {
	return m_box==null || m_box.getX()+m_box.getWidth()<x;
    }

    public void setBox(Rectangle2D box) {
	m_box = box;
    } 

    public String getToolTipText() {
	return m_tooltip;
    }

    public TickInterval getStart() {
	return m_start;
    }

    public String getShortName() {
	return m_shortName;
    }

    public TickInterval getEnd() {
	return m_end;
    }

    public boolean equals(Object obj) {
	if( this==obj )
	    return true;
	if( !(obj instanceof Token) )
	    return false;
	Token that = (Token)obj;
	return m_start.equals(that.m_start) && m_end.equals(that.m_end) 
	    && m_shortName.equals(that.m_shortName);
    }

    public String toString() {
	return "[ "+m_start+" "+m_shortName+" "+m_end+" [";
    }


} // Token
