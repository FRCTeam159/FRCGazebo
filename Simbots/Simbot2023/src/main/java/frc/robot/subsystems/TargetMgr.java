package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class TargetMgr {
    static ArrayList<TagTarget> targets = new ArrayList<>();

    static final public int NO_TAGS = 0;
    static final public int SINGLE_TAG = 1;
    static final public int FRONT_TAGS = 2;
    static final public int PERIMETER_TAGS = 3;
    static final public int FIELD_TAGS = 4;

    static final public int UNKNOWN = 0;
    static final public int OUTSIDE = 1;
    static final public int CENTER = 2;
    static final public int INSIDE = 3;
    static final public int STATION = 4;

    static final public int RED = 1;
    static final public int BLUE = 2;

    static final String[] pStrings = { "unknown", "outside", "center", "inside", "station" };
    static final String[] aStrings = { "unknown", "red", "blue" };

    public static double targetSize = 0.1524; // 0.371; side of inner rectangle of apriltag (meters)

    public static Translation3d field_center_offset = new Translation3d(325.4, 157, 0);

    public static Translation3d robot_offset = new Translation3d(0, 0, 0);
    public static Rotation3d robot_rotation = new Rotation3d(0, 0, 0);
    public static Translation3d field_center = field_center_offset.times(Units.inchesToMeters(1));

    static int type = FIELD_TAGS; // changed by init function

    static int alliance = RED;
    static int start_position = CENTER;

    static Pose2d start_pose = new Pose2d();
    static boolean start_pose_set = false;

    static boolean field_relative = true;

    static public void init() {
        switch (type) {
            case NO_TAGS:
                break;
            case SINGLE_TAG:
                setSingleTarget();
                break;
            case FRONT_TAGS:
                setFrontTargets();
                break;
            case PERIMETER_TAGS:
                setPerimTargets();
                break;
            case FIELD_TAGS:
                setFieldTargets();
                break;
        }
    }

    public static boolean FRCfield() {
        return type == FIELD_TAGS ? true : false;
    }

    static public void setFieldRelative(boolean b) {
        field_relative = b;
    }

    public static void clearStartPose() {
        start_pose = new Pose2d();
        start_pose_set = false;
    }

    public static Pose2d startPose() {
        return start_pose;
    }

    public static Pose2d fieldToRobot(Pose2d r) {
        return r.relativeTo(start_pose);
    }

    static boolean fieldRelative() {
        return field_relative;
    }

    Pose2d getFieldPose(Pose2d robotpose) {
        return robotpose;
    }

    public static int getAlliance() {
        return alliance;
    }

    public static int getStartPosition() {
        return start_position;
    }

    public static boolean startPoseSet() {
        return start_pose_set;
    }

    public static Translation2d getTagTranslation(int id) {
        TagTarget target = getTarget(id);
        Translation2d tag_trans = target.getPose().getTranslation().toTranslation2d();
        if (start_pose_set)
            tag_trans = tag_trans.minus(start_pose.getTranslation());
        return tag_trans;
    }

    public static void setStartPose(int i, Pose2d p) {
        start_pose_set = true;
        double h = i > 4 ? Math.toRadians(180) : 0;
        start_pose = new Pose2d(p.getTranslation(), new Rotation2d(h));
        switch (i) {
            default:
                System.out.println("could not determine starting position from tag !");
                start_position = UNKNOWN;
                alliance = UNKNOWN;
                break;
            case 1:
                alliance = RED;
                start_position = OUTSIDE;
                break;
            case 2:
                alliance = RED;
                start_position = CENTER;
                break;
            case 3:
                alliance = RED;
                start_position = INSIDE;
                break;
            case 4:
                alliance = RED;
                start_position = STATION;
                break;
            case 5:
                alliance = BLUE;
                start_position = STATION;
                break;
            case 6:
                alliance = BLUE;
                start_position = INSIDE;
                break;
            case 7:
                alliance = BLUE;
                start_position = CENTER;
                break;
            case 8:
                alliance = BLUE;
                start_position = OUTSIDE;
                break;
        }
        System.out.format("Initial robot position %s-%s X:%-3.1f Y:%-3.1f H:%-3.1f\n",
                aStrings[alliance], pStrings[start_position], start_pose.getX(), start_pose.getY(),
                start_pose.getRotation().getDegrees());
    }

    static void setFieldTargets() {
        // data taken from inset in field drawing 5 of 7

        Translation3d tags[] = {
                new Translation3d(610.77, 42.19, 18.22), // 1 180
                new Translation3d(610.77, 108.19, 18.22), // 2 180
                new Translation3d(610.77, 174.19, 18.22), // 3 180
                new Translation3d(636.96, 265.74, 27.38), // 4 180
                new Translation3d(14.45, 265.74, 27.38), // 5 0
                new Translation3d(40.25, 174.19, 18.22), // 6 0
                new Translation3d(40.25, 108.19, 18.22), // 7 0
                new Translation3d(40.25, 42.19, 18.22) // 8 0
        };

        for (int i = 0; i < tags.length; i++)
            tags[i] = tags[i].times(Units.inchesToMeters(1));

        // frc has 0,0 at lower OUTSIDE corner but for Gazebo 0,0 is center of field
        // for display and simulation translate tags to field center

        System.out.println("Robot:" + robot_offset);

        for (int i = 0; i < tags.length; i++) {
            Translation3d tag = tags[i];// robot_offset.minus(tags[i]);
            double x = tag.getX();
            double y = tag.getY();
            double z = tag.getZ();
            double tag_angle = i > 3 ? Math.toRadians(180) : 0; // looking forward
            Rotation3d tag_rotation = new Rotation3d(0, 0, tag_angle);
            Rotation3d robot_angle = robot_rotation.rotateBy(tag_rotation);
            TagTarget tt = new TagTarget(i + 1, x, y, z, robot_angle.getZ());
            targets.add(tt);
        }
        System.out.println("tag offsets from FRC origin");
        for (int i = 0; i < targets.size(); i++) {
            // Translation3d tag=robotToFRC(targets.get(i).getTranslation());
            Translation3d tag = targets.get(i).getTranslation();
            System.out.println("id:" + (i + 1) + " " + getString(tag));
        }

        System.out.println("\noffsets from field center");
        System.out.println("robot " + getString(robot_offset.minus(field_center)));
        for (int i = 0; i < targets.size(); i++) {
            // Translation3d tag_frc=robotToFRC(targets.get(i).getTranslation());
            Translation3d tag_frc = targets.get(i).getTranslation();
            Translation3d tag_center = tag_frc.minus(field_center); // offsets to gazebo origin
            System.out.println("id:" + (i + 1) + " " + getString(tag_center));
        }
    }

    static String getString(Translation3d t) {
        return String.format("x:%-2.2f y:%-2.2f z:%-2.2f", t.getX(), t.getY(), t.getZ());
    }

    static Translation3d fromField(Translation3d tt) {
        // Translation3d atag = tt.times(Units.inchesToMeters(1));
        return tt.minus(field_center);
    }

    static Translation2d fromField(Translation2d tt) {
        // Translation3d atag = tt.times(Units.inchesToMeters(1));
        return tt.minus(field_center.toTranslation2d());
    }

    static void setSingleTarget() {
        double dx = 8;
        double dz = 0.25;
        targets.add(new TagTarget(0, dx, 0, dz, 0)); // OUTSIDE
    }

    static void setFrontTargets() {
        double dx = 8;
        double dy = 2;
        double dz = 0.25;
        targets.add(new TagTarget(0, dx, -dy, dz, 0)); // OUTSIDE
        targets.add(new TagTarget(0, dx, dy, dz, 0)); // INSIDE
    }

    static void setPerimTargets() {
        // set one target on all corners and sides of typical FRC field
        double dx = 8.5;
        double dy = 4.26;
        double dz = 0.25;

        double h45 = Math.toRadians(45);
        double h90 = Math.toRadians(90);

        targets.add(new TagTarget(0, dx, -dy, dz, h45)); // OUTSIDE-front
        targets.add(new TagTarget(1, dx, 0, dz, 0.0)); // middle-front
        targets.add(new TagTarget(2, dx, dy, dz, -h45)); // INSIDE-front
        targets.add(new TagTarget(3, 0, dy, dz, h90)); // INSIDE-middle
        targets.add(new TagTarget(4, -dx, dy, dz, h45)); // INSIDE-back
        targets.add(new TagTarget(5, -dx, 0, dz, 0)); // middle-back
        targets.add(new TagTarget(6, -dx, -dy, dz, -h45)); // OUTSIDE-back
        targets.add(new TagTarget(7, 0, -dy, dz, h90)); // OUTSIDE-middle
    }

    static public int maxTargetId() {
        int i = numTargets();
        return (type == FIELD_TAGS) ? i : i - 1;
    }

    static public int minTargetId() {
        return (type == FIELD_TAGS) ? 1 : 0;
    }

    static public int numTargets() {
        switch (type) {
            case NO_TAGS:
                return 0;
            case SINGLE_TAG:
                return 1;
            case FRONT_TAGS:
                return 2;
            case PERIMETER_TAGS:
                return 8;
            case FIELD_TAGS:
                return 8;
        }
        return type;
    }

    static public TagTarget getTarget(int i) {
        int id = (type == FIELD_TAGS) ? i - 1 : i;
        if (id < 0 || id > targets.size())
            return null;
        return targets.get(id);
    }

    static boolean tagsPresent() {
        return type == NO_TAGS ? false : true;
    }

    static public class TagTarget {
        Pose3d targetPose;
        int targetID = 0;

        public TagTarget(int id, Pose3d pose) {
            targetPose = pose;
            targetID = id;
        }

        public TagTarget(int id, double x, double y, double z, double a) {
            targetPose = new Pose3d(x, y, z, new Rotation3d(0, 0, a));
            targetID = id;
        }

        public int getID() {
            return targetID;
        }

        public Pose3d getPose() {
            return targetPose;
        }

        public Translation3d getTranslation() {
            return targetPose.getTranslation();
        }

        public double getTargetSize() {
            return targetSize;
        }

        public TagTarget moveTo(Translation3d loc) {
            Translation3d trans = loc.minus(targetPose.getTranslation());
            Pose3d pose = new Pose3d(trans, targetPose.getRotation());
            return new TagTarget(targetID, pose);
        }

        public String toString() {
            double angle = targetPose.getRotation().toRotation2d().getDegrees();
            return String.format("id:%d x:%-2.1f y:%2.1f z:%1.2f a:%3.1f",
                    targetID, targetPose.getX(), targetPose.getY(), targetPose.getZ(), angle);
        }

    }

}