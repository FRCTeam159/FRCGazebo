package utils;

import java.awt.BorderLayout;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.Arrays;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Stroke;
import javax.swing.JFrame;
import javax.swing.JPanel;


//import org.usfirst.frc.team159.robot.TestPlot.DrawSine;

public class PlotRenderer extends JFrame {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	protected ArrayList<PathData> list = new ArrayList<>();
	private static final int PREF_W = 800;
	private static final int PREF_H = 650;
	private static final int MIN_H = 400;
	private static final int MAX_H = 800;
	protected double tmax=0;
	protected double ymin=1000;
	protected double ymax=-1000;
	protected double xmin=1000;
	protected double xmax=-1000;
	protected double aspect=1;
	protected int plot_width=PREF_W;
	protected int plot_height=PREF_H;
	protected int traces;
	public static final int TIME_MODE=0;
	public static final int XY_MODE=1;

	private int plotMode=TIME_MODE;
	private String labels[]=null;

	private String plot_title = "Generic Plot";
	private String x_axis_label = "X";
	private String y_axis_label = "Y";


	static public Color time_colors[]= {
			Color.BLUE,Color.RED,Color.GREEN,Color.ORANGE,Color.DARK_GRAY,Color.GRAY};
	static public Color xy_colors[]= {
			Color.BLUE,Color.BLUE,Color.DARK_GRAY,Color.DARK_GRAY,Color.RED,Color.RED};

	public PlotRenderer(ArrayList<PathData> d, int n, int type) {
		//String label_list[] ={"Generic Plot","X"};
		//this(d,n,TIME_MODE,new String[] {"Generic Plot","X","Y"});
	}
	public static void showPlot(ArrayList<PathData> d, int traces, int type){
		if(type==PlotUtils.PLOT_DISTANCE)
			distancePlot(d,traces);
		else if(type==PlotUtils.PLOT_DYNAMICS)
			dynamicsPlot(d,traces);
		else if(type==PlotUtils.PLOT_POSITION)
			positionPlot(d,traces);
		else if(type==PlotUtils.PLOT_CALIBRATE)
			calibratePlot(d,traces);
		else
			genericPLot(d,traces);
	}
	
	// Plot Path motion (values vs time)
	public static void calibratePlot(ArrayList<PathData> d, int traces) {
		String list[] ={"Calibrate Plot","Distance","Time","Power","Velocity","Acceleration"};
		JFrame frame = new PlotRenderer(d, traces, PlotRenderer.TIME_MODE, list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
	
    }
	// Plot Path motion (values vs time)
	public static void genericPLot(ArrayList<PathData> d, int traces) {
		String list[] ={"Generic Plot","X",""};
		JFrame frame = new PlotRenderer(d, traces, PlotRenderer.TIME_MODE, list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
	
    }
	// Plot Path motion (values vs time)
	public static void distancePlot(ArrayList<PathData> d, int traces) {
		String list[] = { "Distance Plot","Time (s)","","Left Travel", "Target", "Right Travel", "Target","Heading","Target"};
        JFrame frame = new PlotRenderer(d, traces, PlotRenderer.TIME_MODE, list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
	
	// Plot Path dynamics (values vs time)
	public static void dynamicsPlot(ArrayList<PathData> d, int traces) {
		String list[] = { "Dynamics Plot","Time (s)","","Distance", "Target", "Velocity", "Target","Acceleration","Target"};
        JFrame frame = new PlotRenderer(d, traces, PlotRenderer.TIME_MODE, list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }

    // Plot Path position (wheel traces y vs x)
    public static void positionPlot(ArrayList<PathData> d, int traces) {
		String list[] = { "Position Plot","X","Y","Left Wheels", "Target", "Center", "Target","Right Wheels","Target"};
        JFrame frame = new PlotRenderer(d, traces, PlotRenderer.XY_MODE, list);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }

	public PlotRenderer(ArrayList<PathData> d, int n, int m, String[] l) {
		if(l.length>3)
			labels=Arrays.copyOfRange(l, 3, l.length);
		else
		    labels=null;
		plot_title=l[0];
		x_axis_label=l[1];
		y_axis_label=l[2];
		//labels=l;
		plotMode=m;
		list.addAll(d);
		traces=n;
		getLimits();

		setSize(plot_width, plot_height);
		//setTitle("Path Plot");
		setTitle(plot_title);

		setLocationRelativeTo(null);
		add(new DrawPlot(), BorderLayout.CENTER);
		pack();
		setVisible(true);
	}
	void getLimits() {
		for(PathData  data : list) {
			tmax=data.tm>tmax?data.tm:tmax;	
			for (int i=0;i< PathData.DATA_SIZE;i++) {		
				if(plotMode==XY_MODE){
					xmax=data.d[i]>xmax?data.d[i]:xmax;
					xmin=data.d[i]<xmin?data.d[i]:xmin;
					i++;
				}
				ymax=data.d[i]>ymax?data.d[i]:ymax;
				ymin=data.d[i]<ymin?data.d[i]:ymin;			
			}
		}
		if(plotMode==TIME_MODE){
			xmax=tmax;
			xmin=0;
		}
		else{
			double dx=xmax-xmin;
			double dy=ymax-ymin;
			aspect=dx/dy;
			plot_width=(int)(PREF_W);
			plot_height=(int)(PREF_H/aspect);
			if(aspect>1 && plot_height<MIN_H){
				plot_height=MIN_H;
				plot_width=(int)(plot_height*aspect);
			}
			else if (aspect<1 && plot_height>MAX_H){
				plot_height=MAX_H;
				plot_width=(int)(plot_height*aspect);
			}
			if(aspect>1 && plot_width>MAX_H){
				plot_width=MAX_H;
			}
		}	
		System.out.println("PlotPath: xmax="+xmax+" xmin="+xmin+" ymax="+ymax+" ymin="+ymin+" aspect:"+aspect);
	}
	class DrawPlot extends JPanel {
		
		private static final long serialVersionUID = 1L;
		private int padding = 25;
		private int labelPadding = 35;
		private Color gridColor = new Color(200, 200, 200, 200);
		private int pointWidth = 2;
		private int numberYDivisions = 10;
		private int numberXDivisions = 10;

		private Stroke LINE_STROKE = new BasicStroke(1f);
		private Stroke GRAPH_STROKE = new BasicStroke(2f);
		private Stroke DASHED = new BasicStroke(3f, BasicStroke.CAP_ROUND,
		        BasicStroke.JOIN_ROUND, 3.0f, new float[]{5,5}, 0.0f);

		
		public DrawPlot() {
			setPreferredSize(new Dimension(plot_width, plot_height));
		}

	    protected void paintComponent(Graphics g) {
	        super.paintComponent(g);
	        Graphics2D g2 = (Graphics2D) g.create();
	        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

			double xScale = ((double) getWidth() - (2 * padding) - labelPadding) / (xmax-xmin);
			double yScale = ((double) getHeight() - (2 * padding) - labelPadding) /  (ymax-ymin);

			//System.out.println("w="+ getWidth()+" h="+getHeight()+" xScale="+xScale+" yscale="+yScale);
			
			// draw white background
			g2.setColor(Color.WHITE);
			g2.fillRect(padding + labelPadding, padding, getWidth() - (2 * padding) - labelPadding, getHeight() - 2 * padding - labelPadding);
			g2.setColor(Color.BLACK);

			//g2.setStroke(GRAPH_STROKE);

			// create hatch marks and grid lines for y axis.
			for (int i = 0; i < numberYDivisions + 1; i++) {
				int x0 = padding + labelPadding;
				int x1 = pointWidth + padding + labelPadding;
				int y0 = getHeight() - ((i * (getHeight() - padding * 2 - labelPadding)) / numberYDivisions + padding + labelPadding);
				int y1 = y0;
				if (list.size() > 0) {
					g2.setColor(gridColor);
					g2.drawLine(padding + labelPadding + 1 + pointWidth, y0, getWidth() - padding, y1);
					g2.setColor(Color.BLACK);
					String yLabel = ((int) ((ymin + (ymax - ymin) * ((i * 1.0) / numberYDivisions)) * 100)) / 100.0 + "";
					FontMetrics metrics = g2.getFontMetrics();
					int labelWidth = metrics.stringWidth(yLabel);
					g2.drawString(yLabel, x0 - labelWidth - 5, y0 + (metrics.getHeight() / 2) - 3);
				}
				g2.drawLine(x0, y0, x1, y1);
			}

			// and for x axis
			for (int i = 0; i < numberXDivisions + 1; i++) {
				int x0 = i * (getWidth() - padding * 2 - labelPadding) / numberXDivisions + padding + labelPadding;
				int x1 = x0;
			    int y0 = getHeight() - padding - labelPadding;
			    int y1 = y0 - pointWidth;
				if (list.size() > 0) {
					g2.setColor(gridColor);
					g2.drawLine(x0, getHeight() - padding - labelPadding - 1 - pointWidth, x1, padding);
					g2.setColor(Color.BLACK);
					String xLabel = ((int) ((xmin + (xmax - xmin) * ((i * 1.0) / numberXDivisions)) * 100)) / 100.0 + "";
					FontMetrics metrics = g2.getFontMetrics();
					int labelWidth = metrics.stringWidth(xLabel);
					g2.drawString(xLabel, x0 - labelWidth / 2, y0 + metrics.getHeight() + 3);
				}
				g2.drawLine(x0, y0, x1, y1);
			}


			// create x and y axes 
			g2.drawLine(padding + labelPadding, getHeight() - padding - labelPadding, padding + labelPadding, padding);
			g2.drawLine(padding + labelPadding, getHeight() - padding - labelPadding, getWidth() - padding, getHeight() - padding - labelPadding);
			
			//String xAxisLabel="Time Secs";
			//if(plotMode==XY_MODE)
			//	 xAxisLabel="X Meters";
			String xAxisLabel=x_axis_label;
			int x0 =  (getWidth() - padding * 2 - labelPadding) / 2 + padding + labelPadding;
			int y0 = getHeight() - labelPadding;
			FontMetrics metrics = g2.getFontMetrics();
			int labelWidth = metrics.stringWidth(xAxisLabel);

		    g2.drawString(xAxisLabel, x0 - labelWidth / 2, y0 + metrics.getHeight() + 3);
		    int[] xs = new int[list.size()];
			int[] ys = new int[list.size()];
			if (plotMode == TIME_MODE) {
				for (int i = 0; i < list.size(); i++) {
					xs[i] = (int) (list.get(i).tm * xScale + padding + labelPadding);
				}
				for (int j = 0; j < traces; j++) {
					g2.setColor(time_colors[j]);
					for (int i = 0; i < list.size(); i++) {
						ys[i] = (int) ((ymax - list.get(i).d[j]) * yScale + padding);
					}
					if ((j % 2) == 0)
						g2.setStroke(GRAPH_STROKE);
					else
						g2.setStroke(DASHED);
					g2.drawPolyline(xs, ys, list.size());
				}
			}
			else{
				for (int j = 0; j < traces; j++) {
					g2.setColor(xy_colors[j]);
					for (int i = 0; i < list.size(); i++) {
						xs[i] = (int) ((list.get(i).d[2*j]) * xScale + padding+labelPadding);
						ys[i] = (int) ((ymax - list.get(i).d[2*j+1]) * yScale + padding);
					}
					if ((j % 2) == 0)
						g2.setStroke(GRAPH_STROKE);
					else
						g2.setStroke(DASHED);
					g2.drawPolyline(xs, ys, list.size());
				}
			}
			// draw optional legend
			if(labels != null){
				int legend_line_length = 50;
				int legend_width =0;
				int spacing=metrics.getHeight()+5;
				int legend_height=labels.length*(spacing);
				for (int i=0;i<labels.length;i++){
					labelWidth = metrics.stringWidth(labels[i]);
					legend_width=labelWidth>legend_width?labelWidth:legend_width;
				}
				legend_width+=legend_line_length+30;
				int top = padding+20;
				int left = padding + labelPadding+10;
				g2.setStroke(LINE_STROKE);
				g2.setColor(Color.WHITE);
				g2.fillRect(left, top, legend_width, legend_height);
				g2.setColor(Color.BLACK);

				g2.drawRect(left, top, legend_width, legend_height);
				left+=10;
				top+=10;
				for (int j=0;j<labels.length;j++){
					if(plotMode==XY_MODE)
						g2.setColor(xy_colors[j]);
					else
						g2.setColor(time_colors[j]);
					if ((j % 2) == 0)
						g2.setStroke(GRAPH_STROKE);
					else
						g2.setStroke(DASHED);
					g2.drawLine(left,top,left+legend_line_length,top);
					g2.setColor(Color.BLACK);
					g2.setStroke(LINE_STROKE);
					g2.drawString(labels[j], left+legend_line_length+10, top+5);
					top+=spacing;
				}
			}
		    g2.dispose();
	    }    
	 }
}
