
package utils;

import org.opencv.core.Point;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N3;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.simple.SimpleMatrix;

public class TagResult {
    int id;
    int tag_id;
    double margin;
    double centerX, centerY;
    double[][] corners;
    double[][] homog;
    double[][] rotation;
    double pose_err;

    Transform3d poseResult;

    public TagResult(
            int id,
            int tag_id,
            double margin,
            double centerX,
            double centerY,
            double[] cdata,
            double[] hdata,
            double[] pdata,
            double[] rdata,
            double perr) {
        this.id = id;
        this.tag_id = tag_id;
        this.margin = margin;
        this.centerX = centerX;
        this.centerY = centerY;
        corners = new double[4][2];
        homog = new double[3][3];
        rotation = new double[3][3];
        int dcnt = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                corners[i][j] = cdata[dcnt++];
            }
        }
        dcnt = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                homog[i][j] = hdata[dcnt++];
            }
        }
        Matrix<N3, N3> rmat = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
        dcnt = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rmat.set(i, j, rdata[dcnt++]);
            }
        }
      
        pose_err = perr;
        poseResult=new Transform3d(
            new Translation3d(pdata[0], pdata[1], pdata[2]),
            new Rotation3d(orthogonalizeRotationMatrix(new MatBuilder<>(Nat.N3(), Nat.N3()).fill(rdata))));
        //System.out.println(poseResult);   
    }
    
    /**
     * from: org.photonvision.common.util.math
     * Orthogonalize an input matrix using a QR decomposition. QR decompositions
     * decompose a
     * rectangular matrix 'A' such that 'A=QR', where Q is the closest orthogonal
     * matrix to the input,
     * and R is an upper triangular matrix.
     */
    public static Matrix<N3, N3> orthogonalizeRotationMatrix(Matrix<N3, N3> input) {
        var a = DecompositionFactory_DDRM.qr(3, 3);
        if (!a.decompose(input.getStorage().getDDRM())) {
            // best we can do is return the input
            return input;
        }

        // Grab results (thanks for this _great_ api, EJML)
        var Q = new DMatrixRMaj(3, 3);
        var R = new DMatrixRMaj(3, 3);
        a.getQ(Q, false);
        a.getR(R, false);

        // Fix signs in R if they're < 0 so it's close to an identity matrix
        // (our QR decomposition implementation sometimes flips the signs of columns)
        for (int colR = 0; colR < 3; ++colR) {
            if (R.get(colR, colR) < 0) {
                for (int rowQ = 0; rowQ < 3; ++rowQ) {
                    Q.set(rowQ, colR, -Q.get(rowQ, colR));
                }
            }
        }
        if (!MatrixFeatures_DDRM.isOrthogonal(Q, 1e-9)) // added this test
            return null;
        return new Matrix<>(new SimpleMatrix(Q));
    }

    /**
     * Photonvision: All our solvepnp code returns a tag with X left, Y up, and Z out of the tag To better match
     * wpilib, we want to apply another rotation so that we get Z up, X out of the tag, and Y to the
     * right. We apply the following change of basis: X -> Y Y -> Z Z -> X
     */
    private static final Rotation3d WPILIB_BASE_ROTATION = new Rotation3d(
        new MatBuilder<>(Nat.N3(), Nat.N3()).fill(0, 1, 0, 0, 0, 1, 1, 0, 0));

    public static Pose3d convertOpenCVtoWPIlib(Transform3d cameraToTarget3d) {
        Pose3d p=new Pose3d(cameraToTarget3d.getTranslation(),cameraToTarget3d.getRotation());
        var nwu = CoordinateSystem.convert(p, CoordinateSystem.EDN(), CoordinateSystem.NWU());
        return new Pose3d(nwu.getTranslation(), WPILIB_BASE_ROTATION.rotateBy(nwu.getRotation()));
    }

    public int getId() {
        return id;
    }

    public int getTagId() {
        return tag_id;
    }

    public void setCenterPoint(double x, double y) {
        centerX = x;
        centerY = y;
    }

    public double getDecisionMargin() {
        return margin;
    }

    public double width() {
        return corners[1][0] - corners[0][0];
    }

    public double height() {
        return corners[0][1] - corners[3][1];
    }

    public double getCenterX() {
        return centerX;
    }

    public Point center() {
        return new Point(centerX, centerY);
    }

    public Point tl() {
        return new Point(corners[0][0], corners[0][1]);
    }

    public Point br() {
        return new Point(corners[2][0], corners[2][1]);
    }

    public Point tr() {
        return new Point(corners[1][0], corners[1][1]);
    }

    public Point bl() {
        return new Point(corners[3][0], corners[3][1]);
    }

    public double getCenterY() {
        return centerY;
    }

    public double[][] getCorners() {
        return corners;
    }

    public double[][] getHomog() {
        return homog;
    }

    public Transform3d getPoseTransform() {
        return poseResult;
    }
    public Pose3d getPose() {
        return convertOpenCVtoWPIlib(poseResult);
    }
    public double getPoseError() {
        return pose_err;
    }

    public double getDistance() {
		// camera to target distance along the ground
		Translation2d trans=getPose().getTranslation().toTranslation2d();
		double distance = trans.getNorm(); 
        return distance;
    }
    public double getYaw() {
        Pose3d pose=getPose();
        double angle=Math.atan2(pose.getTranslation().getY(), pose.getTranslation().getX());
		return Math.toDegrees(angle);	
    }
    public double getPitch() {
        Pose3d pose=getPose();
        double angle=Math.atan2(pose.getTranslation().getZ(), pose.getTranslation().getX());
		return Math.toDegrees(angle);	
    }
    
    public double getX(){
        return getPose().getTranslation().getX();
    }
    public double getY(){
        return getPose().getTranslation().getY();
    }
    public double getZ(){
        return getPose().getTranslation().getZ();
    }
    public String toString() {
        String str=null;
        Pose3d pose=getPose();
        if(pose !=null)
          str = String.format("id:%d err:%-2.1f X:%-2.1f Y:%-2.1f Z:%-2.1f H:%-2.1f P:%-2.1f",
                tag_id, pose_err*1e5, getX(), getY(), getZ(), getYaw(), getPitch());
        return str;
    }

    public void print() {
        String str = toString();
        System.out.println(str);
        //System.out.println(getPoseTransform());
        //if(poseResult !=null){
        //Pose3d pose=getPose();
        //System.out.println(getPose()); 
    }
}