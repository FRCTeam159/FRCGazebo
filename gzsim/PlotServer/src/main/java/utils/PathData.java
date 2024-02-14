package utils;


public class PathData {
	public static final int DATA_SIZE=12;
	public double tm=0;
	public double d[] = new double[DATA_SIZE];
	
	public PathData() {
		for(int i=0;i<DATA_SIZE;i++){
            d[i]=0;
        }
	}
	
	public PathData(PathData copyData) {
		tm = copyData.tm;
		d = copyData.d.clone();
	}
	public PathData minus(PathData p1){
		PathData pd=new PathData();
        for(int i=0;i<DATA_SIZE;i++){
            pd.d[i]=d[i]-p1.d[i];
        }
        return pd;
	}
	public PathData plus(PathData p1){
		PathData pd=new PathData();
        for(int i=0;i<DATA_SIZE;i++){
            pd.d[i]=d[i]+p1.d[i];
        }
        return pd;
	}
	public void print(){
        System.out.format("%f %f %f %f %f %f\n", 
        tm, d[0], d[1], d[2], d[3],d[4],d[5]);
    }
}
