/* -*- java -*-
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@porbeagle.shore.mbari.org">Frederic Py</a>
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
