/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@mbari1224.shore.mbari.org">Frederic Py</a>
 */

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
