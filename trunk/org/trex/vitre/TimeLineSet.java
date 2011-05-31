/* @(#)TEMPLATE.java.tpl
 */
/**
 * 
 *
 * @author <a href="mailto:fpy@mbari1224.shore.mbari.org">Frederic Py</a>
 */
import java.util.TreeSet;
import java.util.SortedSet;
import java.util.LinkedList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.ListIterator;

import javax.swing.JComponent;
import javax.swing.JViewport;

import java.awt.Container;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.Shape;
import java.awt.Font;
import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

public class TimeLineSet
{
    private static class NameCompare implements Comparator<TimeLine> {

	public int compare(TimeLine a, TimeLine b) {
	    return a.getName().compareTo(b.getName());
	}

    } // TimeLineSet.NameCompare

    public static interface UpdateListener {
	public void updated(TimeLineSet who);
    } // TimeLineSet.UpdateListener

    private UpdateListener m_listener = null;

    private double m_tlHeigh = 20;
    private double m_minTickSpace = 5;

    private Font m_font = new Font("fixed", Font.PLAIN, 10);
    private Color m_lineColor = new Color(0.5f, 0.5f, 0.5f);   

    private TreeSet<TimeLine> m_timelines;

    private Tick m_curTick = null;

    public TimeLineSet() {
	m_timelines = new TreeSet<TimeLine>(new NameCompare());
    }

    public void setUpdateListener(UpdateListener listen) {
	m_listener = listen;
    }

    public Iterator<TimeLine> iterator() {
	return m_timelines.iterator();
    }

    public TimeLine get(String name) {
	TimeLine result = new TimeLine(name);

	if( !m_timelines.add(result) ) 
	    result = m_timelines.tailSet(result).first();
	    
	return result; 
    }

    public TimeLine add(String name, Token tok) {
	TimeLine tmp = get(name);
	tmp.add(tok);
	if( m_listener!=null )
	    m_listener.updated(this);
	return tmp;
    }

    public void clear() {
	m_timelines.clear();
	m_curTick = null;
	if( m_listener!=null )
	    m_listener.updated(this);
    }

    public String getToolTipText(int x, int y) {
	Iterator<TimeLine> iter = iterator();
	
	while( iter.hasNext() ) {
	    TimeLine cur = iter.next();
	    if( cur.below(y) )
		continue;
	    else if( cur.above(y) )
		break;
	    else
		return cur.getToolTipText(x);
	}
	return null;
    }

    public boolean newTick(Tick val) {
	if( m_curTick==null || val.compareTo(m_curTick)>0 ) {
	    Iterator<TimeLine> iter = iterator();

	    m_curTick = val;
	    while( iter.hasNext() )
		iter.next().newTick(m_curTick);
	    if( m_listener!=null )
		m_listener.updated(this);
	    return true;
	}
	return false;
    }

    public Dimension paint(Graphics2D g2, Rectangle2D avail, Tick start) {
	Iterator<TimeLine> iter = iterator();
	double textMargin = 0;
	double rowHeight = 18;
	int count = 0;
	
	LinkedList<TimeLine.PaintResult> hints = new LinkedList<TimeLine.PaintResult>();

	g2.setPaint(Color.white);
	while( iter.hasNext() ) {
	    String id = iter.next().getName();
	    Rectangle2D txt =  g2.getFontMetrics().getStringBounds(id, g2);

	    textMargin = Math.max(textMargin, txt.getWidth());
	    g2.drawString(id, 5, (int)(avail.getY()+5+count*(rowHeight+4)+(rowHeight+txt.getHeight())/2));
	    ++count;
	}
	Rectangle2D chart = new Rectangle2D.Double(avail.getX()+10+textMargin, avail.getY()+5,
						   avail.getWidth()-15-textMargin, 
						   avail.getHeight()-10);
	g2.setPaint(new Color(1f, 1f, 1f, 0.8f));
	g2.fill(chart);

	double tickX = chart.getX()+1;
	Shape savedClip = g2.getClip();
	g2.clip(chart);
	
	count = 0;
	iter = iterator();
	
	while( iter.hasNext() ) {
	    TimeLine cur = iter.next();
	    Rectangle2D tlArea = new Rectangle2D.Double(chart.getX(), 
							chart.getY()+2+count*(rowHeight+4),
							chart.getWidth(), rowHeight);

	    hints.offer(cur.initPaint(tlArea, start));
	    ++count;
	}
	
	while( !hints.isEmpty() ) {
	    Tick nextTick = Tick.PLUS_INF;
	    double tickWidth = 0;

	    boolean writeTick = false;

	    ListIterator<TimeLine.PaintResult> hintsi = hints.listIterator();

	    while( hintsi.hasNext() ) {
		TimeLine.PaintResult cur = hintsi.next();
		
		if( start.equals(cur.nextTick) )
		    writeTick = true;

		// 		if( start.compareTo(cur.nextTick)>=-1 )
		tickWidth = Math.max(tickWidth, TimeLine.getTickLength(g2, cur, start, tickX));
	    }	    
	    
	    hintsi = hints.listIterator();

	    if( writeTick ) {
		g2.setPaint(m_lineColor);
		g2.draw(new Line2D.Double(tickX, chart.getY(), tickX, 
					  chart.getY()+chart.getHeight()));
		g2.drawString(start.toString(), (int)(tickX+2), 
			      (int)(chart.getY()+chart.getHeight()));
	    }
	    while( hintsi.hasNext() ) {
		TimeLine.PaintResult cur = hintsi.next();
		
		if( null==TimeLine.drawTick(g2, cur, start, tickX, tickWidth) ) 
		    hintsi.remove();
		else if( cur.nextTick.compareTo(nextTick)<=0 )
		    nextTick = cur.nextTick;
	    }
	    if( !start.next().equals(nextTick) )
		start = nextTick.previous();
	    else
		start = nextTick;
	    tickX += tickWidth;
	}

	g2.setClip(savedClip);

	return new Dimension((int)(tickX+10-avail.getX()), (int)(25+count*(rowHeight+4)));
    }

} // TimeLineSet
