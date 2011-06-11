/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@mbari.org">Frederic Py</a>
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

import java.util.LinkedList;
import java.util.ListIterator;

import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.GradientPaint;
import java.awt.Shape;
import java.awt.geom.Rectangle2D;


public class TimeLine
{
    private Rectangle2D m_lastArea = null;
    private LinkedList<Token> m_tokens;
    private String m_name;
    private double minY, maxY;
    private int m_type;

    public static final int unknown = 0;
    public static final int internal = 1;
    public static final int external = 2;

    public TimeLine(String name, int type) {
	m_name = name;
	m_tokens = new LinkedList<Token>();
	m_type = type; 
    }

    public TimeLine(String name) {
	m_name = name;
	m_tokens = new LinkedList<Token>();
	m_type = unknown; 
    }

    public int getType() {
	return m_type;
    }

    public String getName() {
	return m_name;
    }

    public boolean below(int y) {
	return m_lastArea==null || m_lastArea.getY()+m_lastArea.getHeight()<y;
    }

    public boolean above(int y) {
	return m_lastArea==null || m_lastArea.getY()>y;
    }
    
    public ListIterator<Token> iterator() {
	return m_tokens.listIterator(0);
    }

    public String getToolTipText(int x) {
	ListIterator<Token> iter = iterator();

	while( iter.hasNext() ) {
	    Token cur = iter.next();

	    if( cur.below(x) )
		continue;
	    else if( cur.above(x) )
		break;
	    else 
		return cur.getToolTipText();
	}
	return null;
    }

    public void add(Token tok) {
	ListIterator<Token> iter = iterator();

	while( iter.hasNext() ) {
	    Token cur = iter.next();
	    int cmp = cur.getStart().getUpperBound().compareTo(tok.getEnd().getUpperBound());
	    
	    if( cmp>=0 ) {
		iter.previous();
		break;
	    } else {
		cmp = cur.getEnd().getUpperBound().compareTo(tok.getStart().getUpperBound());
		if( cmp>0 ) {
		    try {
			cur.getEnd().setUpperBound(tok.getStart().getUpperBound());
		    } catch(BadTickInterval e) {}
		}
	    }
	}
	iter.add(tok);
    }

    public void newTick(Tick val) {
	ListIterator<Token> iter = iterator();
	
	while( iter.hasNext() ) {
	    Token cur = iter.next();
	    cur.getStart().clockSet(val);
	    cur.getEnd().clockSet(val);
	}
    }

    public static class PaintResult {
	public Rectangle2D area;
	public ListIterator<Token> drawIter, sizeIter;
	public Tick nextTick;

	public double predictedWidth;
	public int    numberToken;

	protected PaintResult(Rectangle2D rect, ListIterator<Token> di, 
			      ListIterator<Token> si,  Tick d) {
	    area = rect;
	    drawIter = di;
	    sizeIter = si;
	    while( drawIter.hasNext() ) {
		Token cur = drawIter.next();
		sizeIter.next();
		int cmp = cur.getEnd().getLowerBound().compareTo(d);
		if( cmp>=0 ) {
		    drawIter.previous();
		    sizeIter.previous();
		    break;
		}
	    }
	    nextTick = d;
	    predictedWidth = 0;
	    numberToken = 0;
	}

    } // TimeLine.PaintResult


    private static double defaultMargin = 4;

    public PaintResult initPaint(Rectangle2D area, Tick position) {
	m_lastArea = area;
	return new PaintResult(area, iterator(), iterator(), position);
    }

    private static Rectangle2D minBlock(Graphics2D g2, String content) {
	return g2.getFontMetrics().getStringBounds(content, g2);
    }

    public static double getTickLength(Graphics2D g2, PaintResult state, 
				       Tick clk, double tickX) {
	TickInterval previous = null;
	double posX = state.area.getX();
	boolean inTick = false;
	
	state.predictedWidth = 0;
	state.numberToken = 0;
		
	while( state.sizeIter.hasNext() ) {
	    Token cur = state.sizeIter.next();
	    TickInterval tokStart = cur.getStart();
	    int cmp = tokStart.getLowerBound().compareTo(clk);
	    double size;

	    // Look at the header
	    if( cmp>0 ) {
		state.sizeIter.previous();
		break;
	    } 
	    if( cmp==0 ) { 
		if( !tokStart.isSingleton() && 
		    (null==previous || !previous.equals(tokStart)) ) {
		    size = minBlock(g2, 
				    tokStart.toString()).getWidth()+2*defaultMargin;
		    state.predictedWidth += size;
		}
		inTick = true;
		posX = tickX+state.predictedWidth;
	    } 

	    previous = cur.getEnd();
	    cmp = previous.getLowerBound().compareTo(clk.next());
	    
	    if( cmp>0 ) {
		state.sizeIter.previous();
		break;
	    }
	    if( cmp==0 || inTick ) {
		size = minBlock(g2, cur.getShortName()).getWidth()+2*defaultMargin;
		if( posX<tickX )
		    size -= tickX-posX;
		if( size>0 )
		    state.predictedWidth += size;
		state.numberToken += 1;
	    }
	    if( previous.getLowerBound().equals(clk) ) {
		if( !previous.isSingleton() ) {
		    size = minBlock(g2, previous.toString()).getWidth()+2*defaultMargin;
		    state.predictedWidth += size;
		}
	    } else {
		state.sizeIter.previous();
		break;
	    }
	    posX = tickX+state.predictedWidth;
	}
	return state.predictedWidth;
    }

    static final int TOK_OK =0;
    static final int TOK_UNDEFINED =1;
    static final int TOK_FAILED =2;
    
    public static int kindOf(Token tok) {
	if( tok.isUndefined() ) {
	    return TOK_UNDEFINED;
	} else if( tok.isFailed() ) {
	    return TOK_FAILED;
	} else {
	    return TOK_OK;
	}
    }

    private static Rectangle2D drawBlock(Graphics2D g2, PaintResult state, 
					 String content, double width,
					 Rectangle2D textInfo, int kind) {
	Rectangle2D area = new Rectangle2D.Double(state.area.getX(), 
						  state.area.getY(),
						  width, state.area.getHeight());
	Color col_from, col_to;
	if( TOK_FAILED==kind ) {
	    col_from = new Color(1f, 0.8f, 0.8f);
	    col_to = new Color(0.8f, 0.5f, 0.5f);
	} else if( TOK_UNDEFINED==kind ) {
	    col_from = new Color(1f, 1f, 0.8f);
	    col_to = new Color(0.8f, 0.8f, 0.5f);
	} else {
	    col_from = new Color(0.8f, 1f, 0.8f);
	    col_to = new Color(0.5f, 0.8f, 0.5f);
	}
	    	
	g2.setPaint(new GradientPaint((int)area.getX(), (int)area.getY(),
				      col_from,
				      (int)area.getX(), (int)(area.getY()+area.getHeight()),
				      col_to));
	g2.fill(area);
	g2.setPaint(new Color(0.25f, 0.25f, 0.25f));
	g2.draw(area);
	
	Shape savedClip = g2.getClip();
	g2.clip(area);
	
	g2.setPaint(Color.black);
	g2.drawString(content, (int)(area.getX()+(area.getWidth()-textInfo.getWidth())/2),
		      (int)(area.getY()+(area.getHeight()+textInfo.getHeight()-2)/2));

	g2.setClip(savedClip);
	state.area.setRect(state.area.getX()+width, state.area.getY(),
			   state.area.getWidth()-width, state.area.getHeight());
	return area;
    }

    private static Rectangle2D drawBlock(Graphics2D g2, PaintResult state, String content, double width,
					 int kind) {
	return drawBlock(g2, state, content, width, minBlock(g2, content), kind);
    }

    private static Rectangle2D drawBlock(Graphics2D g2, PaintResult state, String content, double tickX, 
					 double margin, int kind) {
	Rectangle2D txt = minBlock(g2, content);

	return drawBlock(g2, state, content, txt.getWidth()+2*margin, txt, kind);
    }

    public static PaintResult drawTick(Graphics2D g2, PaintResult state, Tick clk, 
				       double tickX, double tickWidth) {
	double extraMargin=0;
	TickInterval previous = null;
	boolean inTick =false;

	if( state.numberToken>0 )
	    extraMargin = (tickWidth-state.predictedWidth)/(2*state.numberToken);
	
	//	System.out.println("extraMargin["+clk+"] = "+extraMargin);

	while( state.drawIter.hasNext() ) {
	    Token cur = state.drawIter.next();
	    TickInterval tokStart = cur.getStart();
	    int cmp = tokStart.getLowerBound().compareTo(clk);
	    
	    if( cmp>0 ) {
		state.drawIter.previous();
		state.nextTick = tokStart.getLowerBound();
		return state;
	    }
	    if( cmp==0 ) {
		if( !tokStart.isSingleton() &&( null==previous || !previous.equals(tokStart) ) ) {
		    if( tickX>state.area.getX() )
			state.area.setRect(tickX, state.area.getY(),
					   state.area.getWidth()-tickX,
					   state.area.getHeight());
		    drawBlock(g2, state, tokStart.toString(), tickX, defaultMargin, kindOf(cur));
		}
		inTick = true;
	    }
	    previous = cur.getEnd();
	    cmp = previous.getLowerBound().compareTo(clk.next());
	    
	    if( cmp>0 ) {
		state.drawIter.previous();
		state.nextTick = previous.getLowerBound();
		return state;
	    }
	    if( cmp==0 ) {
		cur.setBox(drawBlock(g2, state, cur.getShortName(), tickX-state.area.getX()+tickWidth, 
				     kindOf(cur)));
	    } else if( inTick ) 
		cur.setBox(drawBlock(g2, state, cur.getShortName(), tickX, defaultMargin+extraMargin,
				     kindOf(cur)));
	    
	    if( previous.getLowerBound().equals(clk) ) {
		if( !previous.isSingleton() ) {
		    if( tickX>state.area.getX() )
			state.area.setRect(tickX, state.area.getY(),
					   state.area.getWidth()-tickX,
					   state.area.getHeight());
		    drawBlock(g2, state, previous.toString(), tickX, defaultMargin, kindOf(cur));
		}
	    } else {
		state.drawIter.previous();
		state.nextTick = previous.getLowerBound();
		return state;
	    }
	}

	return null;
    }

} // TimeLine
