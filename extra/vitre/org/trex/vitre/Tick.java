/* -*- java -*-
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

public class Tick
{
    private int     m_value;
    private boolean m_isInfinity, m_isPositive;

    private Tick(boolean pos) {
	m_isInfinity = true;
	m_isPositive = pos;
    }

    public static final Tick PLUS_INF = new Tick(true);
    public static final Tick MINUS_INF = new Tick(false);

    public Tick(int value) {
	m_isInfinity = false;
	m_value = value;
    }

    public boolean equals(Object obj) {
	if( this==obj )
	    return true;
	if( !( obj instanceof Tick ) )
	    return false;
	Tick other = (Tick)obj;
	if( m_isInfinity!=other.m_isInfinity )
	    return false;
	if( m_isInfinity )
	    return m_isPositive==other.m_isPositive;
	else
	    return m_value==other.m_value; 
    }

    Tick next() {
	if( m_isInfinity )
	    return this;
	else 
	    return new Tick(m_value+1);
    }

    Tick previous() {
	if( m_isInfinity )
	    return this;
	else 
	    return new Tick(m_value-1);
    }

    public int compareTo(Tick other) {
	if( m_isInfinity ) {
	    if( other.m_isInfinity ) {
		if( m_isPositive )
		    return other.m_isPositive?0:1;
		else 
		    return other.m_isPositive?-1:0;
	    } else 
		return m_isPositive?1:-1;
	} else if( other.m_isInfinity )
	    return other.m_isPositive?-1:1;
	else 
	    return m_value-other.m_value;
    }

    public String toString() {
	if( m_isInfinity )
	    if( m_isPositive )
		return "+inf";
	    else
		return "-inf";
	else 
	    return Integer.toString(m_value);
    }

    public static Tick max(Tick a, Tick b) {
	if( a.compareTo(b)>0 )
	    return a;
	else 
	    return b;
    }

    public static Tick min(Tick a, Tick b) {
	if( a.compareTo(b)<0 )
	    return a;
	else 
	    return b;
    }

} // class Tick
