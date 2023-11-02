
package frc.robot.subsystems;

import java.awt.BorderLayout;
import java.awt.Graphics;
import java.util.ArrayList;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Stroke;
import javax.swing.JFrame;
import javax.swing.JPanel;

import frc.robot.PathData;

public class PlotPath extends JFrame {
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	protected ArrayList<PathData> list = new ArrayList<>();
	private int PREF_W = 800;
	private int PREF_H = 650;
	private int PIXELS_PER_FOOT = 80;
	private int numberYDivisions = 10;
	private int numberXDivisions = 10;
	protected double tmax = 0;
	protected double ymin = 1000;
	protected double ymax = -1000;
	protected double xmin = 1000;
	protected double xmax = -1000;
	protected int traces = 0;
	protected int id = 0;
	public static final int DFLT_MODE = 0;
	public static final int PATH_MODE = 1;
	public static final int TRAJ_MODE = 2;
	public static final int DIST_MODE = 4;
	public static final int TEST_MODE = 3;

	private int plotMode = DFLT_MODE;
	private static String pathLabels[] = { "LEFT", "CTR", "RIGHT" };
	private static String trajLabels[] = { "X", "Y", "VEL", "ACC", "HDG" };
	private static String distLabels[] = { "L-OBS", "L-TGT", "R-OBS", "R-TGT", "H-OBS", "H-TGT" };
	private static String testLabels[] = { "X-TGT", "X-OBS", "Y-TGT", "Y-OBS", "H-TGT", "H-OBS" };

	private String traceLabels[] = null;
	private String xAxisLabel = null;
	private String yAxisLabel = null;

	private String plotTitle = "Plot";

	static public Color colors[] = { Color.BLUE, Color.RED, Color.GREEN, Color.ORANGE, Color.DARK_GRAY, Color.GRAY };

	public PlotPath(ArrayList<PathData> d, int n) {
		this(d, 0, n, DFLT_MODE);
	}

	public PlotPath(ArrayList<PathData> d, int i, int n, int m) {
		plotMode = m;
		list.addAll(d);
		traces = n;
		id = i;
		getLimits();
		switch (plotMode) {
		default:
		case DFLT_MODE:
			plotTitle = "Plot";
			break;
		case PATH_MODE:
			traceLabels = pathLabels;
			xAxisLabel = "X";
			plotTitle = "Path Plot";
			yAxisLabel = "Y";
			setXYSize();
			break;
		case TRAJ_MODE:
			traceLabels = trajLabels;
			xAxisLabel = "Time";
			plotTitle = "Trajectory Plot";
			break;
		case DIST_MODE:
			traceLabels = distLabels;
			xAxisLabel = "Time";
			plotTitle = "L/R Distance Plot";
			break;
		case TEST_MODE:
			traceLabels = testLabels;
			xAxisLabel = "Time";
			plotTitle = "Test Plot";
			break;
		}
		setSize(PREF_W, PREF_H);
		setLocationRelativeTo(null);
		plotTitle += " " + id;
		setTitle(plotTitle);
		System.out.println("PlotPath " + plotTitle + " traces=" + n + " points=" + list.size());

		add(new DrawPlot(), BorderLayout.CENTER);
		pack();
		setVisible(true);
	}

	// Set window size of XY plot so that tick spacing is the same in both
	// dimentions
	void setXYSize() {
		int xrange = (int) (xmax - xmin + 0.5);
		int yrange = (int) (ymax - ymin + 0.5);
		PREF_W = xrange * PIXELS_PER_FOOT;
		PREF_H = yrange * PIXELS_PER_FOOT;
		System.out.println("W=" + PREF_W + " H=" + PREF_H);
		numberXDivisions = xrange;
		numberYDivisions = yrange;

	}

	void getLimits() {
		for (PathData data : list) {
			tmax = data.tm > tmax ? data.tm : tmax;
			for (int i = 0; i < PathData.DATA_SIZE; i++) {
				if (plotMode == PATH_MODE) {
					xmax = data.d[i] > xmax ? data.d[i] : xmax;
					xmin = data.d[i] < xmin ? data.d[i] : xmin;
					i++;
				}
				ymax = data.d[i] > ymax ? data.d[i] : ymax;
				ymin = data.d[i] < ymin ? data.d[i] : ymin;
			}
		}
		if (plotMode != PATH_MODE) {
			xmax = tmax;
			xmin = 0;
		}
		System.out.println("PlotPath Limits: xmax=" + xmax + " xmin=" + xmin + " ymax=" + ymax + " ymin=" + ymin);
	}

	class DrawPlot extends JPanel {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;
		private int padding = 25;
		private int labelPadding = 35;
		private Color gridColor = new Color(200, 200, 200, 200);
		private int pointWidth = 2;

		private Stroke BOX_STROKE = new BasicStroke(1f);
		private Stroke GRAPH_STROKE = new BasicStroke(2f);
		private Stroke DASHED = new BasicStroke(3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 3.0f,
				new float[] { 5, 5 }, 0.0f);

		public DrawPlot() {
			setPreferredSize(new Dimension(PREF_W, PREF_H));
		}

		private void showAxisLabels(Graphics2D g) {
			// draw x axis label
			if (xAxisLabel != null) {
				int x0 = (getWidth() - padding * 2 - labelPadding) / 2 + padding + labelPadding;
				int y0 = getHeight() - labelPadding;
				FontMetrics metrics = g.getFontMetrics();
				int xLabelWidth = metrics.stringWidth(xAxisLabel);
				g.drawString(xAxisLabel, x0 - xLabelWidth / 2, y0 + metrics.getHeight() + 3);
			}
			if (yAxisLabel != null) {
				int x0 = labelPadding / 2;
				int y0 = (getHeight() - labelPadding) / 2;
				FontMetrics metrics = g.getFontMetrics();
				int labelWidth = metrics.stringWidth(yAxisLabel);
				g.drawString(yAxisLabel, x0 - labelWidth / 2, y0 - metrics.getHeight() / 2);
			}
		}

		private void showTraceLabels(Graphics2D g) {
			if (traceLabels == null)
				return;
			FontMetrics metrics = g.getFontMetrics();
			int labelHeight = metrics.getHeight();
			// System.out.println("label ht=" + labelHeight + " label width=" +
			// xLabelWidth);
			// draw trace legends
			// - calculate box size (traces)
			int textWidth = 50;
			int lineLength = 25;
			int labelSpacing = 10;
			int boxBorder = 6;

			int boxHt = traces * (labelHeight + labelSpacing) + boxBorder;
			int boxWidth = textWidth + lineLength + boxBorder;
			int X0 = getWidth() - boxWidth - padding - labelPadding;
			int Y0 = boxBorder + padding;
			g.setStroke(BOX_STROKE);

			// System.out.println("traces="+traces+" box ht="+boxHt+" box width="+boxWidth);
			g.setColor(Color.WHITE);
			g.fillRect(X0, Y0, boxWidth, boxHt);
			g.setColor(Color.BLACK);
			g.drawRect(X0, Y0, boxWidth, boxHt);

			// - position box (plot width, plot weight, box size)
			// - draw labels in box (mode specific)
			// o draw a small line segment (same color and stipling as trace)
			// o draw trace label text (same color as trace)
			int X = X0 + boxBorder;
			int Y = Y0 + boxBorder + labelSpacing;
			for (int j = 0; j < traces; j++) {
				g.setColor(colors[j]);
				if ((j % 2) == 0)
					g.setStroke(GRAPH_STROKE);
				else
					g.setStroke(DASHED);
				g.drawLine(X, Y, X + lineLength, Y);
				g.setColor(Color.BLACK);
				g.drawString(traceLabels[j], X + lineLength + 6, Y + 5);
				Y += labelHeight + labelSpacing;
			}
		}

		protected void paintComponent(Graphics g) {
			super.paintComponent(g);
			Graphics2D g2 = (Graphics2D) g.create();
			g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

			double xScale = ((double) getWidth() - (2 * padding) - labelPadding) / (xmax - xmin);
			double yScale = ((double) getHeight() - (2 * padding) - labelPadding) / (ymax - ymin);

			// System.out.println("w="+ getWidth()+" h="+getHeight()+" xScale="+xScale+"
			// yscale="+yScale);

			// draw white background
			int px0=padding + labelPadding;
			int py0=padding;
			int pw=getWidth() - (2 * padding) - labelPadding;
			int ph=getHeight() - (2 * padding) - labelPadding;

			g2.setColor(Color.WHITE);
			g2.fillRect(px0, py0, pw,ph);
			g2.setColor(Color.BLACK);
			double factor= (plotMode != PATH_MODE)?100:10;
			int num=numberYDivisions+1;
			if (plotMode != PATH_MODE) {
				// create hatch marks and grid lines for y axis.
				for (int i = 0; i < num; i++) {
					int x0 = padding + labelPadding;
					int x1 = pointWidth + padding + labelPadding;
					int y0 = (int)(getHeight() - ((i * ph) / num + padding + labelPadding));
					int y1 = y0;
					if (list.size() > 0) {
						g2.setColor(gridColor);
						g2.drawLine(padding + labelPadding + 1 + pointWidth, y0, getWidth() - padding, y1);
						g2.setColor(Color.BLACK);
						double yLabel = ((int) ((ymin + (ymax - ymin) * i / num) * factor)) /factor;
						FontMetrics metrics = g2.getFontMetrics();
						int labelWidth = metrics.stringWidth(""+yLabel);
						g2.drawString(""+yLabel, x0 - labelWidth - 5, y0 + (metrics.getHeight() / 2) - 3);
					}
					g2.drawLine(x0, y0, x1, y1);
				}
			}
			else{
				for (int i = 0; i < num + 1; i++) {
					int x0 = padding + labelPadding;
					int x1 = pointWidth + padding + labelPadding;
					int y0 = (int)((i * ph) / num+ py0);
					int y1 = y0;
					if (list.size() > 0) {
						g2.setColor(gridColor);
						g2.drawLine(padding + labelPadding + 1 + pointWidth, y0, getWidth() - padding, y1);
						g2.setColor(Color.BLACK);
						double yLabel = ((int) ((ymin + (ymax - ymin) * i  / num) * factor)) /factor;
						FontMetrics metrics = g2.getFontMetrics();
						int labelWidth = metrics.stringWidth(""+yLabel);
						g2.drawString(""+yLabel, x0 - labelWidth - 5, y0 + (metrics.getHeight() / 2) - 3);
					}
					g2.drawLine(x0, y0, x1, y1);
				}
			}

			// and for x axis
			num=numberXDivisions+1;
			for (int i = 0; i < num; i++) {
				int x0 = (int)(i * (getWidth() - padding * 2 - labelPadding) / num + padding + labelPadding);
				int x1 = x0;
				int y0 = getHeight() - padding - labelPadding;
				int y1 = y0 - pointWidth;
				if (list.size() > 0) {
					g2.setColor(gridColor);
					g2.drawLine(x0, getHeight() - padding - labelPadding - 1 - pointWidth, x1, padding);
					g2.setColor(Color.BLACK);
					double xLabel = ((int) ((xmin + (xmax - xmin) * i / num) * factor)) / factor;
					FontMetrics metrics = g2.getFontMetrics();
					int labelWidth = metrics.stringWidth(""+xLabel);
					g2.drawString(""+xLabel, x0 - labelWidth / 2, y0 + metrics.getHeight() + 3);
				}
				g2.drawLine(x0, y0, x1, y1);
			}
			showAxisLabels(g2);

			// create x and y axes
			g2.drawLine(padding + labelPadding, getHeight() - padding - labelPadding, padding + labelPadding, padding);
			g2.drawLine(padding + labelPadding, getHeight() - padding - labelPadding, getWidth() - padding,
					getHeight() - padding - labelPadding);

			// draw data traces
			int[] xs = new int[list.size()];
			int[] ys = new int[list.size()];
			if (plotMode != PATH_MODE) { // each vector drawn as a separate trace
				for (int i = 0; i < list.size(); i++) {
					xs[i] = (int) (list.get(i).tm * xScale + padding + labelPadding);
				}
				for (int j = 0; j < traces; j++) {
					g2.setColor(colors[j]);
					for (int i = 0; i < list.size(); i++) {
						ys[i] = (int) ((ymax - list.get(i).d[j]) * yScale + padding+j);
					}
					if ((j % 2) == 0)
						g2.setStroke(GRAPH_STROKE);
					else
						g2.setStroke(DASHED);
					g2.drawPolyline(xs, ys, list.size());
				}
			} else { // PATH_MODE - XY Plots data arranged as traces of of x[],y[] vectors
				for (int j = 0; j < traces; j++) {
					g2.setColor(colors[j]);
					for (int i = 0; i < list.size(); i++) {
						xs[i] = (int) ((list.get(i).d[2 * j]) * xScale + padding + labelPadding);
						// ys[i] = (int) ((ymax - list.get(i).d[2 * j + 1]) * yScale + padding);
						ys[i] = (int) ((list.get(i).d[2 * j + 1] - ymin) * yScale + padding);

					}
					if ((j % 2) == 0)
						g2.setStroke(GRAPH_STROKE);
					else
						g2.setStroke(DASHED);
					g2.drawPolyline(xs, ys, list.size());
				}
			}
			showTraceLabels(g2);
			g2.dispose();
		}
	}
}
