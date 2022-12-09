
package utils;

import org.opencv.core.Point;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Quaternion;
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
                // rotation[i][j]=rdata[dcnt++];
            }
        }
        // normalize(rmat);

        pose_err = perr;
        Matrix<N3, N3> R = orthogonalizeRotationMatrix(rmat);
        Translation3d trans = new Translation3d(pdata[0], pdata[1], pdata[2]);
        // Matrix<N3,N3> rmat=orthogonalizeRotationMatrix(new MatBuilder<>(Nat.N3(),
        // Nat.N3()).fill(rdata));

        if (R==null) {
            System.out.println("rotation matrix is not orthoganal");
        } else {     
            Quaternion q = getQuaternion(R);
            Rotation3d rot = new Rotation3d(q);
            poseResult = new Transform3d(trans, rot);
        }
        //System.out.println(this);
    }
    // work around for bug in Rotation3d(Matrix) constructor
    // Turn rotation matrix into a quaternion
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    public static Quaternion getQuaternion(Matrix<N3,N3>  R){
        Quaternion m_q = new Quaternion();
        double trace = R.get(0, 0) + R.get(1, 1) + R.get(2, 2);
        double w;
        double x;
        double y;
        double z;

        if (trace > 0.0) {
            double s = 0.5 / Math.sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (R.get(2, 1) - R.get(1, 2)) * s;
            y = (R.get(0, 2) - R.get(2, 0)) * s;
            z = (R.get(1, 0) - R.get(0, 1)) * s;
            } 
        else {
            if (R.get(0, 0) > R.get(1, 1) && R.get(0, 0) > R.get(2, 2)) {
                double s = 2.0 * Math.sqrt(1.0 + R.get(0, 0) - R.get(1, 1) - R.get(2, 2));
                w = (R.get(2, 1) - R.get(1, 2)) / s;
                x = 0.25 * s;
                y = (R.get(0, 1) + R.get(1, 0)) / s;
                z = (R.get(0, 2) + R.get(2, 0)) / s;
            } else if (R.get(1, 1) > R.get(2, 2)) {
                double s = 2.0 * Math.sqrt(1.0 + R.get(1, 1) - R.get(0, 0) - R.get(2, 2));
                w = (R.get(0, 2) - R.get(2, 0)) / s;
                x = (R.get(0, 1) + R.get(1, 0)) / s;
                y = 0.25 * s;
                z = (R.get(1, 2) + R.get(2, 1)) / s;
            } else {
                double s = 2.0 * Math.sqrt(1.0 + R.get(2, 2) - R.get(0, 0) - R.get(1, 1));
                w = (R.get(1, 0) - R.get(0, 1)) / s;
                x = (R.get(0, 2) + R.get(2, 0)) / s;
                y = (R.get(1, 2) + R.get(2, 1)) / s;
                z = 0.25 * s;
            }
        }
        return new Quaternion(w, x, y, z);
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

    public Transform3d getPoseResult() {
        return poseResult;
    }
    public double getPoseError() {
        return pose_err;
    }

    public String toString() {
        String str = String.format("id:%d tag_id:%d conf:%f cX:%-2.1f cY:%-2.1f w:%d h:%d",
                id, tag_id, margin, centerX, centerY, (int) width(), (int) height());
        return str;
    }

    public void print() {
        String str = toString();
        System.out.println(str);
        if(poseResult !=null)
           System.out.println(poseResult);
        
        // if (homog != null) {
        //     for (int k = 0; k < 3; k++) {
        //         for (int l = 0; l < 3; l++) {
        //             System.out.format("%-8.2f ", homog[k][l]);
        //         }
        //         System.out.println();
        //     }
        // }
    }
}