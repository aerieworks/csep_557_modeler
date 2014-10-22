// The sample box model.  You should build a file
// very similar to this for when you make your model in
// order to plug in to the animator project.

#pragma warning (disable : 4305)
#pragma warning (disable : 4244)
#pragma warning(disable : 4786)
#pragma warning (disable : 4312)

#include "modelerview.h"
#include "modelerapp.h"
#include "modelerdraw.h"
#include "mat.h"
#include "vec.h"
#include <FL/gl.h>
#include <math.h>
#include <iostream>

#define PI 3.14159265359

using namespace std;

// This is a list of the controls for the RobotArm
// We'll use these constants to access the values 
// of the controls from the user interface.
enum BoxModelControls
{ 
    TIME, XPOS, YPOS, ZPOS, HEIGHT, DIRECTION,
    ABDOMEN_LENGTH, ABDOMEN_WIDTH, ABDOMEN_HEIGHT, ABDOMEN_OFFSET,
    HEAD_LENGTH, HEAD_WIDTH, HEAD_HEIGHT,
    LEG_RADIUS, TOE_RADIUS, LEG_UPPER_LENGTH, LEG_MIDDLE_LENGTH, LEG_LOWER_LENGTH, FOOT_LENGTH,
    LOWER_KNEE_ANGLE_FROM_GROUND,
    HEEL_TO_HIP_DISTANCE,
    _1_LEG_POS, _2_LEG_POS, _3_LEG_POS, _4_LEG_POS,
    L1_HIP_ANGLE, L1_X_OFFSET, L1_Y_OFFSET,
    L2_HIP_ANGLE, L2_X_OFFSET, L2_Y_OFFSET,
    L3_HIP_ANGLE, L3_X_OFFSET, L3_Y_OFFSET,
    L4_HIP_ANGLE, L4_X_OFFSET, L4_Y_OFFSET,
    R1_HIP_ANGLE, R1_X_OFFSET, R1_Y_OFFSET,
    R2_HIP_ANGLE, R2_X_OFFSET, R2_Y_OFFSET,
    R3_HIP_ANGLE, R3_X_OFFSET, R3_Y_OFFSET,
    R4_HIP_ANGLE, R4_X_OFFSET, R4_Y_OFFSET,
    NUMCONTROLS,
};

enum LegSide
{
    LEFT = -1,
    RIGHT = 1
};

// We'll be getting the instance of the application a lot;
// might as well have it as a macro.

#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))

// Create a sinusoidal transition from v0 to v0+vd
#define sinize(v0, vd, t0, t1, t) ((v0) + (vd) * ((cos(((double)(t) / (t1)) * PI + PI) + 1) / 2))

#define DEG(x) (180/PI * (x))
#define RAD(x) ((x) * PI/180)
#define SQ(x) (pow((x), 2))
#define VEC3D Vec3<double>
#define VEC4D Vec4<double>
Mat4<double> copyMVMatrix()
{
    Mat4<double> mat;
    glGetDoublev(GL_MODELVIEW_MATRIX, mat.n);
    // OpenGL stores matrices in column-major order, but the Mat class uses row-major order.
    // Transpose the matrix to swap the rows and columns into their correct arrangement.
    return mat.transpose();
}

void drawFixedSphere(const Vec4<double>& vec, const double r = 0.8, double g = 0, double b = 0.8)
{
    // Calculate the transformation necessary to get back to the desired point given the current matrix.
    Mat4<double> mat = copyMVMatrix();
    Vec4<double> adjusted = mat.inverse() * vec;

    glPushMatrix();
    setDiffuseColor(r, g, b);
    cerr << "Fixed sphere: adjusting by " << adjusted << endl;
    glTranslated(adjusted[0], adjusted[1], adjusted[2]);
    drawSphere(.25);
    glPopMatrix();
    setDiffuseColor(0.8, 0.8, 0.8);
}

void drawOriginSphere(const char* name, double r = 0.8, double g = 0, double b = 0.8)
{
    drawFixedSphere(Vec4<double>(0, 0, 0, 1), r, g, b);
}

static int iteration = 0;
bool doLog()
{
    return iteration % 500 == 0;
}

class Bone
{
public:
    double angle;
    
    Bone(const double length, const Vec3d axis) : length(length), axis(axis), angle(0) {}
    
    Mat4d apply(const Mat4d& mat)
    {
        const Vec4d start4d = mat * Vec4d(0, 0, 0, 1);
        start = Vec3d(start4d[0], start4d[1], start4d[2]);
        
        jacobianAxis = mat * axis;
        
        Mat4d newMat = mat * mat.createRotation(angle, axis[0], axis[1], axis[2]) * mat.createTranslation(length, 0, 0);
        const Vec4d end4d = newMat * Vec4d(0, 0, 0, 1);
        end = Vec3d(end4d[0], end4d[1], end4d[2]);
        return newMat;
    }
    
    Vec3d getJacobianAxis()
    {
        return jacobianAxis;
    }
    
    Vec3d getStart()
    {
        return start;
    }
    
    Vec3d getEnd()
    {
        return end;
    }
private:
    Vec3d jacobianAxis;
    Vec3d start;
    Vec3d end;
    const Vec3d axis;
    const double length;
};

Vec3d crossProduct(const Vec3d& u, const Vec3d& v)
{
    return Vec3d(u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0]);
}

class Leg
{
public:
    Leg(LegSide side, double position) : side(side), position(position) {}
    
    void draw(double xOffset, double yOffset, double lateralAngle)
    {
        cerr << "Drawing leg" << endl;
        const double x = VAL(HEEL_TO_HIP_DISTANCE) + xOffset;
        const double y = yOffset;
        const double legPositionRadians = RAD(position);
        double hipAngle, knee1Angle, knee2Angle, ankleAngle;
        Vec4<double> hipVec(side * VAL(HEAD_WIDTH) * sin(legPositionRadians), 0, VAL(HEAD_LENGTH) * cos(legPositionRadians), 1);
        cerr << "Hipvec: " << hipVec << endl;
        //findAngles(x, y - VAL(HEIGHT), hipAngle, knee1Angle, knee2Angle, ankleAngle);
        // Upper
        glPushMatrix();
        glTranslated(hipVec[0], hipVec[1], hipVec[2]);
        Mat4<double> mat = copyMVMatrix().inverse();
        Vec4<double> target = mat * Vec4<double>(x * cos(legPositionRadians), 0, x * sin(legPositionRadians), 1);
        Vec4d angles = findAngles2(target);
        
        /*glRotated(angles[0], 0, side, 0);
        glRotated(angles[1], 1, 0, 0);
        drawCylinder(VAL(LEG_UPPER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // Middle
        glTranslated(0, 0, VAL(LEG_UPPER_LENGTH));
        glRotated(angles[2], 1, 0, 0);
        drawCylinder(VAL(LEG_MIDDLE_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // Lower
        glTranslated(0, 0, VAL(LEG_MIDDLE_LENGTH));
        glRotated(angles[3], 1, 0, 0);
        drawCylinder(VAL(LEG_LOWER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // Foot
        glTranslated(0, 0, VAL(LEG_LOWER_LENGTH));
        glRotated(0, 1, 0, 0);
        drawCylinder(VAL(FOOT_LENGTH), VAL(LEG_RADIUS), VAL(TOE_RADIUS));*/

        glPopMatrix();
    }
    
private:
    const double JACOBIAN_EPSILON = 1;
    const double JACOBIAN_THRESHOLD = 0.1;
    const double JACOBIAN_ITERATION_LIMIT = 600000;
    const int JOINT_COUNT = 4;
    
    LegSide side;
    double position;
    
    void drawWithAngles(const Vec4d angles, double r, double g, double b)
    {
        glPushMatrix();
        setDiffuseColor(r, g, b);
        glRotated(angles[0], 0, side, 0);
        glRotated(angles[1], 1, 0, 0);
        drawCylinder(VAL(LEG_UPPER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // Middle
        glTranslated(0, 0, VAL(LEG_UPPER_LENGTH));
        glRotated(angles[2], 1, 0, 0);
        drawCylinder(VAL(LEG_MIDDLE_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // Lower
        glTranslated(0, 0, VAL(LEG_MIDDLE_LENGTH));
        glRotated(angles[3], 1, 0, 0);
        drawCylinder(VAL(LEG_LOWER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // Foot
        glTranslated(0, 0, VAL(LEG_LOWER_LENGTH));
        glRotated(0, 1, 0, 0);
        drawCylinder(VAL(FOOT_LENGTH), VAL(LEG_RADIUS), VAL(TOE_RADIUS));
        setDiffuseColor(0.8, 0.8, 0.8);
        glPopMatrix();
    }
    
    Vec4d findAngles2(const Vec3d target)
    {
        cerr << "InvKin to " << target << endl;
        glPushMatrix();
        glTranslated(target[0], target[1], target[2]);
        drawSphere(0.25);
        glPopMatrix();
        
        Bone* bones[JOINT_COUNT];
        bones[0] = new Bone(0, Vec3d(0, 1, 0));
        bones[1] = new Bone(VAL(LEG_UPPER_LENGTH), Vec3d(1, 0, 0));
        bones[2] = new Bone(VAL(LEG_MIDDLE_LENGTH), Vec3d(1, 0, 0));
        bones[3] = new Bone(VAL(LEG_LOWER_LENGTH), Vec3d(1, 0, 0));

        int jointIndex = 0;
        bool done = false;
        Vec4d deltas;
        iteration = 0;
        while (!done && iteration < JACOBIAN_ITERATION_LIMIT)
        {
            // Adjust bone positions.
            Mat4d mat;
            for (int i = 0; i < JOINT_COUNT; i++)
            {
                mat = bones[i]->apply(mat);
                if (doLog())
                {
                    //cerr << "[" << iteration << "] Bone " << i << " result matrix: " << endl << mat << endl;
                }
            }

            // We use a 4x4 for the Jacobian, but leave the last row 0 and ignore it (so we can still use the Mat4 class).
            Mat4d J;
            Vec3d effector(bones[JOINT_COUNT - 1]->getEnd());
            for (int i = 0; i < JOINT_COUNT; i++)
            {
                // Fill the Jacobian.
                Vec3d column(crossProduct(bones[i]->getJacobianAxis(), effector - bones[i]->getStart()));
                for (int j = 0; j < 3; j++)
                {
                    J.n[j * 4 + i] = column[j];
                }
                
                // Build a vector of deltas.
                deltas[i] = bones[i]->angle;
            }
            //cerr << "Jacobian: " << endl << J << endl;
            
            // Apply the adjustment value.
            bones[jointIndex]->angle += JACOBIAN_EPSILON;
            //cerr << "Adjusting " << jointIndex << " to " << bones[jointIndex]->angle << endl;
            double curDifference = getJacobianDifference(J, deltas, target);
            if (doLog())
            {
                cerr << "Effector location: " << effector << endl;
                //drawWithAngles(deltas, (double)iteration / (double)JACOBIAN_ITERATION_LIMIT, 0, 0);
            }
            if (curDifference <= JACOBIAN_THRESHOLD)
            {
                cerr << "Got close enough." << endl;
                // Close enough, call it good.
                done = true;
            }

            jointIndex = (jointIndex + 1) % JOINT_COUNT;
            iteration += 1;
        }
        drawWithAngles(deltas, 0, 0, 0.8);
 
        for (int i = 0; i < JOINT_COUNT; i++)
        {
            delete bones[i];
        }
        
        cerr << "Final angles: " << deltas << endl;
        return deltas;
    }
    
    double** multiplyMatrix(const double** m1, const double** m2, const int rows, const int cols)
    {
        double** result[rows][cols];
        for (int tgtR = 0; tgtR < rows; tgtR++)
        {
            for (int tgtC = 0; tgtC < cols; tgtC++)
            {
                double result = 0;
                for (int srcR = 0;
                m2Col[0] = m2[0][c];
                m2[1][c], m2[2][c]);
            }
        }
    }
    
    double getJacobianDifference(const Vecd* J, const int jointCount, const Vecd& deltas, const Vec3d& target)
    {
        Vec3d result;
        for (int i = 0; i < 3; i++)
        {
            Vecd jointResult = J[i] * deltas;
            for (int j = 0; j < jointCount; j++)
            {
                result[i] += jointResult[i];
            }
        }
        return (target - result).length();
    }
    
    void findAngles(double x, double y, double& hipAngle, double& knee1Angle, double& knee2Angle, double& ankleAngle)
    {
        cerr << "Finding angles for: (" << x << ", " << y << ")" << endl;
        const double upperLength = VAL(LEG_UPPER_LENGTH);
        const double middleLength = VAL(LEG_MIDDLE_LENGTH);
        const double lowerLength = VAL(LEG_LOWER_LENGTH);
        double distance = sqrt(pow(x, 2) + pow(y, 2));
        double hipAngleRad;
        // Start with the hip angle set to something that will "work" (but not necessarily be comfortable).
        const double initialAngle = atan(y / x);
        bool isUpperParallel = false;
        if (distance > upperLength)
        {
            // Start with the upper leg angled towards the target heel point.
            hipAngleRad = initialAngle;
            isUpperParallel = true;
            if (distance >= upperLength + middleLength + lowerLength)
            {
                hipAngle = DEG(hipAngleRad);
                knee1Angle = 0;
                knee2Angle = 0;
                ankleAngle = hipAngle;
                return;
            }
        }
        else
        {
            // Start with the upper leg angled perpendicular to the target heel point.
            hipAngle = initialAngle - PI/2;
        }

        double knee1Rad = NAN;
        double knee2Rad = NAN;
        double ankleRad = NAN;
        
        double prevDelta = NAN;
        double prevHipAngleRad = NAN;
        int factor = 0;
        do
        {
            double curKnee1Rad, curKnee2Rad, curAnkleRad;
            findKneeAndAnkleAngles(x, y, hipAngleRad, curKnee1Rad, curKnee2Rad, curAnkleRad);
            const double curDelta = fabs(curKnee1Rad - curKnee2Rad);
            if (isnan(prevDelta) || curDelta < prevDelta)
            {
                prevDelta = curDelta;
                knee1Rad = curKnee1Rad;
                knee2Rad = curKnee2Rad;
                ankleRad = curAnkleRad;
            }
            else if (factor < 2)
            {
                hipAngleRad = prevHipAngleRad;
                factor += 1;
            }
            else
            {
                break;
            }
            prevHipAngleRad = hipAngleRad;
            const double inc = PI/(180 * pow(10, factor));
            hipAngleRad += inc;

        } while (true);
        hipAngle = DEG(hipAngleRad);
        knee1Angle = DEG(knee1Rad);
        knee2Angle = DEG(knee2Rad);
        ankleAngle = -DEG(ankleRad);
        
        
        
    }
        
    void findKneeAndAnkleAngles(double x, double y, double hipRad, double& knee1Rad, double& knee2Rad, double& ankleRad)
    {
        const double upperLength = VAL(LEG_UPPER_LENGTH);
        const double middleLength = VAL(LEG_MIDDLE_LENGTH);
        const double lowerLength = VAL(LEG_LOWER_LENGTH);

        // Calculate the distance between the end of the upper leg and the target heel point.
        const double upperY = upperLength * sin(hipRad);
        const double remainingX = x - upperLength * cos(hipRad);
        const double remainingY = y - upperY;
        const double distance = sqrt(SQ(remainingX) + SQ(remainingY));

        // Calculate angles for a triangle formed by the middle leg, lower leg, and a line from the end of the upper leg to the target heel point.
        // (Law of Cosines)
        const double lowRemainingAngle = acos((SQ(lowerLength) + SQ(distance) - SQ(middleLength)) / (2 * lowerLength * distance));
        const double midRemainingAngle = acos((SQ(middleLength) + SQ(distance) - SQ(lowerLength)) / (2 * middleLength * distance));
        knee2Rad = midRemainingAngle + lowRemainingAngle;
        
        double angleFromUpperToVertical;
        if (upperY < 0)
        {
            angleFromUpperToVertical = fabs(hipRad) + PI/2;
        }
        else
        {
            angleFromUpperToVertical = PI/2 - fabs(hipRad);
        }
        knee1Rad = PI - (midRemainingAngle + angleFromUpperToVertical + asin(remainingX / distance));
        ankleRad = lowRemainingAngle + acos(remainingX / distance);
    }
};

// To make a BoxModel, we inherit off of ModelerView
class BoxModel : public ModelerView
{
public:
    // Constructor for the model.  In your model, 
    // make sure you call the ModelerView constructor,
    // as done below.
    BoxModel(int x, int y, int w, int h, char *label) : ModelerView(x,y,w,h,label), frame(0) {}
    virtual void draw();
private:
    int frame;
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createBoxModel(int x, int y, int w, int h, char *label)
{ 
    return new BoxModel(x,y,w,h,label); 
}

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out RobotArm
void BoxModel::draw()
{
    const double duration = 30;
    
    // This call takes care of a lot of the nasty projection 
    // matrix stuff.  Unless you want to fudge directly with the 
	// projection matrix, don't bother with this ...
    ModelerView::draw();
    
    // draw the floor
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(.5f,.5,0);
	glPushMatrix();
	glTranslated(-10,0,-10);
	drawBox(20,0.01f,20);
	glPopMatrix();

    glPushMatrix();
    glTranslated(VAL(HEEL_TO_HIP_DISTANCE) * cos(RAD(VAL(_1_LEG_POS))), 0, VAL(HEEL_TO_HIP_DISTANCE) * sin(RAD(VAL(_1_LEG_POS))));
    glScaled(0.5, 0.125, 0.5);
    drawSphere(1);
    glPopMatrix();
    
    setAmbientColor(.1f,.1f,.1f);
    setDiffuseColor(0.8, 0.8, 0.8);
    glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
    glTranslated(0, VAL(HEIGHT) , 0);
    glRotated(VAL(DIRECTION), 0, 1, 0);

    // Abdomen
    glPushMatrix();
    glRotated(180, 0, 1, 0);
    glTranslated(0, 0, VAL(ABDOMEN_OFFSET));
    glScaled(VAL(ABDOMEN_WIDTH), VAL(ABDOMEN_HEIGHT), VAL(ABDOMEN_LENGTH));
    drawSphere(1);
    glPopMatrix();
    
    // Cephalothorax (head)
    glTranslated(0, 0, -VAL(HEAD_LENGTH)/4);
    const double angle = sinize(0, 30, 0, 30, VAL(TIME));
    glRotated(angle, -1, 0, 0);
    glTranslated(0, 0, VAL(HEAD_LENGTH)/4);
    glPushMatrix();
    glScaled(VAL(HEAD_WIDTH), VAL(HEAD_HEIGHT), VAL(HEAD_LENGTH));
    drawSphere(1);
    glPopMatrix();
    
    Leg(LEFT, VAL(_1_LEG_POS)).draw(VAL(L1_X_OFFSET)/* + sinize(0, -1, 0, duration, frame)*/,
                                    VAL(L1_Y_OFFSET)/* + sinize(0, -2, 0, duration, frame)*/,
                                    VAL(L1_HIP_ANGLE)/* + sinize(0, -35, 0, duration, frame)*/);
    /*Leg(LEFT, VAL(_2_LEG_POS)).draw(VAL(L2_X_OFFSET) + sinize(0, -2, 0, 100, VAL(TIME)), VAL(L2_Y_OFFSET), VAL(L2_HIP_ANGLE));
    Leg(LEFT, VAL(_3_LEG_POS)).draw(VAL(L3_X_OFFSET) + sinize(0, -2, 0, 100, VAL(TIME)), VAL(L3_Y_OFFSET), VAL(L3_HIP_ANGLE));
    Leg(LEFT, VAL(_4_LEG_POS)).draw(VAL(L4_X_OFFSET) + sinize(0, -2, 0, 100, VAL(TIME)), VAL(L4_Y_OFFSET), VAL(L4_HIP_ANGLE));
    
    Leg(RIGHT, VAL(_1_LEG_POS)).draw(VAL(R1_X_OFFSET) + sinize(0, -1, 0, duration, frame) + sinize(0, -2, 0, 100, VAL(TIME)),
                                     VAL(R1_Y_OFFSET) + sinize(0, -2, 0, duration, frame),
                                     VAL(R1_HIP_ANGLE) + sinize(0, -35, 0, duration, frame));
    Leg(RIGHT, VAL(_2_LEG_POS)).draw(VAL(R2_X_OFFSET) + sinize(0, -2, 0, 100, VAL(TIME)), VAL(R2_Y_OFFSET), VAL(R2_HIP_ANGLE));
    Leg(RIGHT, VAL(_3_LEG_POS)).draw(VAL(R3_X_OFFSET) + sinize(0, -2, 0, 100, VAL(TIME)), VAL(R3_Y_OFFSET), VAL(R3_HIP_ANGLE));
    Leg(RIGHT, VAL(_4_LEG_POS)).draw(VAL(R4_X_OFFSET) + sinize(0, -2, 0, 100, VAL(TIME)), VAL(R4_Y_OFFSET), VAL(R4_HIP_ANGLE));
*/
    frame = (frame + 1) % 60;
}

int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	//								 stepsize, defaultvalue)
    // You will want to modify this to accommodate your model.
    ModelerControl controls[NUMCONTROLS];
    controls[TIME]   = ModelerControl("Time", 0, 100, 0.1f, 0);
	controls[XPOS]   = ModelerControl("X Position", -5, 5, 0.1f, 1);
	controls[YPOS]   = ModelerControl("Y Position",  0, 5, 0.1f, 0);
	controls[ZPOS]   = ModelerControl("Z Position", -5, 5, 0.1f, 0);
	controls[HEIGHT] = ModelerControl("Height",      1, 5, 0.1f, 2);
    controls[DIRECTION] = ModelerControl("Direction", 0, 360, 0.1f, 90);
    controls[ABDOMEN_LENGTH] = ModelerControl("Abdomen Length", 0, 5, 0.1f, 2.03);
    controls[ABDOMEN_WIDTH] = ModelerControl("Abdomen Width", 0, 5, 0.1f, 1.44);
    controls[ABDOMEN_HEIGHT] = ModelerControl("Abdomen Height", 1, 5, 0.1f, 1.23);
    controls[ABDOMEN_OFFSET] = ModelerControl("Abdomen Offset", 0, 5, 0.1f, 2.01);
    controls[HEAD_LENGTH] = ModelerControl("Head Length", 0, 5, 0.1f, 1.53);
    controls[HEAD_WIDTH] = ModelerControl("Head Width", 0, 5, 0.1f, 1.36);
    controls[HEAD_HEIGHT] = ModelerControl("Head Height", 1, 5, 0.1f, 1);
    controls[LEG_RADIUS] = ModelerControl("Leg Radius", 0, 1, 0.1f, 0.15);
    controls[TOE_RADIUS] = ModelerControl("Toe Radius", 0, 1, 0.1f, 0.1);
    controls[LEG_UPPER_LENGTH] = ModelerControl("Leg Length (Top)", 1, 5, 0.1f, 3.5);
    controls[LEG_MIDDLE_LENGTH] = ModelerControl("Leg Length (Middle)", 1, 5, 0.1f, 2.5);
    controls[LEG_LOWER_LENGTH] = ModelerControl("Leg Length (Bottom)", 1, 5, 0.1f, 2.75);
    controls[FOOT_LENGTH] = ModelerControl("Foot Length", 1, 5, 0.1f, 2);
    controls[LOWER_KNEE_ANGLE_FROM_GROUND] = ModelerControl("Lower Knee Angle", 0, 180, 0.1f, 65);
    controls[HEEL_TO_HIP_DISTANCE] = ModelerControl("Heel-to-Hip Distance", 0, 15, 0.1f, 5);

    controls[_1_LEG_POS] = ModelerControl("Leg Position (Front)", 0, 1, 0.1f, 30.6);
    controls[_2_LEG_POS] = ModelerControl("Leg Position (Front Mid.)", 0, 1, 0.1f, 61.2);
    controls[_3_LEG_POS] = ModelerControl("Leg Position (Back Mid.)", 0, 1, 0.1f, 81);
    controls[_4_LEG_POS] = ModelerControl("Leg Position (Back)", 0, 1, 0.1f, 95.4);

    controls[L1_HIP_ANGLE] = ModelerControl("L1: Hip Angle", 0, 180, 0.1f, 40);
    controls[L1_X_OFFSET] = ModelerControl("L1: Foot X Offset", -5, 5, 0.1f, 0);
    controls[L1_Y_OFFSET] = ModelerControl("L1: Foot Y Offset", -5, 5, 0.1f, 0);
    controls[L2_HIP_ANGLE] = ModelerControl("L2: Hip Angle", 0, 180, 0.1f, 70);
    controls[L2_X_OFFSET] = ModelerControl("L2: Foot X Offset", -5, 5, 0.1f, 0);
    controls[L2_Y_OFFSET] = ModelerControl("L2: Foot Y Offset", -5, 5, 0.1f, 0);
    controls[L3_HIP_ANGLE] = ModelerControl("L3: Hip Angle", 0, 180, 0.1f, 110);
    controls[L3_X_OFFSET] = ModelerControl("L3: Foot X Offset", -5, 5, 0.1f, 0);
    controls[L3_Y_OFFSET] = ModelerControl("L3: Foot Y Offset", -5, 5, 0.1f, 0);
    controls[L4_HIP_ANGLE] = ModelerControl("L4: Hip Angle", 0, 180, 0.1f, 140);
    controls[L4_X_OFFSET] = ModelerControl("L4: Foot X Offset", -5, 5, 0.1f, 0);
    controls[L4_Y_OFFSET] = ModelerControl("L4: Foot Y Offset", -5, 5, 0.1f, 0);

    controls[R1_HIP_ANGLE] = ModelerControl("R1: Hip Angle", 0, 180, 0.1f, 40);
    controls[R1_X_OFFSET] = ModelerControl("R1: Foot X Offset", -5, 5, 0.1f, 0);
    controls[R1_Y_OFFSET] = ModelerControl("R1: Foot Y Offset", -5, 5, 0.1f, 0);
    controls[R2_HIP_ANGLE] = ModelerControl("R2: Hip Angle", 0, 180, 0.1f, 70);
    controls[R2_X_OFFSET] = ModelerControl("R2: Foot X Offset", -5, 5, 0.1f, 0);
    controls[R2_Y_OFFSET] = ModelerControl("R2: Foot Y Offset", -5, 5, 0.1f, 0);
    controls[R3_HIP_ANGLE] = ModelerControl("R3: Hip Angle", 0, 180, 0.1f, 110);
    controls[R3_X_OFFSET] = ModelerControl("R3: Foot X Offset", -5, 5, 0.1f, 0);
    controls[R3_Y_OFFSET] = ModelerControl("R3: Foot Y Offset", -5, 5, 0.1f, 0);
    controls[R4_HIP_ANGLE] = ModelerControl("R4: Hip Angle", 0, 180, 0.1f, 140);
    controls[R4_X_OFFSET] = ModelerControl("R4: Foot X Offset", -5, 5, 0.1f, 0);
    controls[R4_Y_OFFSET] = ModelerControl("R4: Foot Y Offset", -5, 5, 0.1f, 0);

    
    // Initialize the modeler application with your model and the
    // appropriate array of controls.
    ModelerApplication::Instance()->Init(&createBoxModel, controls, NUMCONTROLS);


	// make sure we give back the memory to older OSs that don't 
	// clear your memory pool after shutdown.
    int Result = ModelerApplication::Instance()->Run();
	delete ModelerApplication::Instance();
	return Result;
}

