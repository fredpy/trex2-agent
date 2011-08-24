/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@porbeagle.shore.mbari.org">Frederic Py</a>
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

public class TickInterval {
    private Tick m_lowerBound, m_upperBound;

    public static final int before       = -6; // [a a]  [b b]  
    public static final int meet         = -5; // [a a][b b]   
    public static final int overlap      = -4; // [a [b a] b]  
    public static final int endedBy      = -3; // [a [b ab]
    public static final int contain      = -2; // [a [b b] a]
    public static final int start        = -1; // [ab a] b]     
    public static final int equal        = 0;  // [ab ab]
    public static final int startedBy    = 1;  // [ab b] a]
    public static final int during       = 2;  // [b [a a] b]
    public static final int end          = 3;  // [b [a ab]
    public static final int overlappedBy = 4;  // [b [a b] a]
    public static final int metBy        = 5;  // [b b][a a]
    public static final int after        = 6;  // [b b] [a a]
    
    
    private static void checkValidity(Tick low, Tick up) throws BadTickInterval {
	if( low.compareTo(up)>0 )
	    throw new BadTickInterval("Invalid interval : ["+low+" "+up+"]"); 
    }
    
    public TickInterval(int tick) {
	m_lowerBound = m_upperBound = new Tick(tick);
    }

    public TickInterval(Tick min, Tick max) throws BadTickInterval {
	checkValidity(min, max);
	m_lowerBound = min;
	m_upperBound = max;
    }

    public TickInterval() {
	m_lowerBound = Tick.MINUS_INF;
	m_upperBound = Tick.PLUS_INF;
    }

    public TickInterval(TickInterval other) {
	m_lowerBound = other.m_lowerBound;
	m_upperBound = other.m_upperBound;
    }

    public Tick getLowerBound() {
	return m_lowerBound;
    }

    public Tick getUpperBound() {
	return m_upperBound;
    }

    public void setLowerBound(Tick value) throws BadTickInterval {
	checkValidity(value, m_upperBound);
	m_lowerBound = value;
    }

    public void setUpperBound(Tick value) throws BadTickInterval {
	checkValidity(m_lowerBound, value);
	m_upperBound = value;
    }
    
    public boolean equals(Object obj) {
	if( this==obj )
	    return true;
	if( !(obj instanceof TickInterval) )
	    return false;
	TickInterval other = (TickInterval)obj;
	return m_lowerBound.equals(other.m_lowerBound) && m_upperBound.equals(other.m_upperBound);
    }

    public void clockSet(Tick now) {
	int cmp = now.compareTo(m_lowerBound);
	
	if( cmp>0 ) {
	    cmp = now.compareTo(m_upperBound);
	    if( cmp<=0 )
		m_lowerBound = now;
	}
    }

    private int subCompareTo(TickInterval other) {
	// Pre : low(this)<low(other)
	// before | meet | overlap | contain | endedBy
	
	int comp = m_upperBound.compareTo(other.m_upperBound);
	
	if( comp<0 ) {
	    // before | meet | overlap
	    Tick upUpper = m_upperBound.next();
	    
	    comp = upUpper.compareTo(other.m_lowerBound);
	    
	    if( comp<0 )
		return before;
	    else
		return comp==0?meet:overlap;
	} else 
	    return comp==0?endedBy:contain;
    } 

    public int compareTo(TickInterval other) {
	int comp = m_lowerBound.compareTo(other.m_lowerBound);
	
	if( comp<0 ) 
	    return subCompareTo(other);
	else if( comp>0 )
	    return -other.subCompareTo(this);
	else {
	    comp = m_upperBound.compareTo(other.m_upperBound);
	    if( comp<0 )
		return start;
	    else 
		return comp==0?equal:startedBy;
	}
    }

    public boolean isSingleton() {
	return m_lowerBound.equals(m_upperBound);
    }

    public String toString() {
	if( isSingleton() )
	    return m_lowerBound.toString();
	else 
	    return "["+m_lowerBound+" "+m_upperBound+"]";
    }
		
}
