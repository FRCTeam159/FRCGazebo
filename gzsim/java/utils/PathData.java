package utils;


public class PathData {
	public static final int DATA_SIZE=6;
	public double tm=0;
	public double d[] = new double[DATA_SIZE];
	
	public PathData() {}
	
	public PathData(PathData copyData) {
		tm = copyData.tm;
		d = copyData.d.clone();
	}
}
