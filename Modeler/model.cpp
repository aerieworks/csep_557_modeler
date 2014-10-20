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

#define DEG(x) (180 / PI * (x))

class Leg
{
public:
    Leg(LegSide side, double position) : side(side), position(position) {}
    
    void draw(double xOffset, double yOffset, double lateralAngle)
    {
        const double x = VAL(HEEL_TO_HIP_DISTANCE) + xOffset;
        const double y = VAL(HEIGHT) + yOffset;
        const double legPositionRadians = position * PI;
        double hipAngle;
        double knee1Angle;
        double knee2Angle;
        findHipAngle(x, y, hipAngle, knee1Angle, knee2Angle);
        glPushMatrix();
        // Upper
        glTranslated(side * sin(legPositionRadians), 0, cos(legPositionRadians));
        //glRotated(sinize(0, xRotation, 0, 30, frame), 1, 0, 0);
        glRotated(lateralAngle, 0, side, 0);
        glRotated(-hipAngle, 1, 0, 0);
        drawCylinder(VAL(LEG_UPPER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // Middle
        glTranslated(0, 0, VAL(LEG_UPPER_LENGTH));
        glRotated(knee1Angle, 1, 0, 0);
        drawCylinder(VAL(LEG_MIDDLE_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // Lower
        glTranslated(0, 0, VAL(LEG_MIDDLE_LENGTH));
        glRotated(knee2Angle, 1, 0, 0);
        drawCylinder(VAL(LEG_LOWER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
        // AnklelowerLength
        glTranslated(0, 0, VAL(LEG_LOWER_LENGTH));
        glRotated(-VAL(LOWER_KNEE_ANGLE_FROM_GROUND), 1, 0, 0);
        drawCylinder(VAL(FOOT_LENGTH), VAL(LEG_RADIUS), VAL(TOE_RADIUS));
        glPopMatrix();
    }
    
private:
    const double FIND_ANGLE_THRESHOLD = 0.01;
    LegSide side;
    double position;
    
    //double getKnee1Angle(double hipAngle) { return hipAngle * VAL(HIP_KNEE1_RATIO); }
    //double getKnee2Angle(double hipAngle) { return hipAngle * VAL(HIP_KNEE2_RATIO); }

    void findHipAngle(double x, double y, double& hipAngle, double& knee1Angle, double& knee2Angle)
    {
        cerr << "Finding hip angle for (" << x << ", " << y << "), " << VAL(LEG_UPPER_LENGTH) << ", " << VAL(LEG_MIDDLE_LENGTH) << ", " << VAL(LEG_LOWER_LENGTH) << endl;
        double min = 0;
        double max = PI / 2;
        // Fix the lower knee angle.  This value isn't actually what we will rotate by; it is the inside angle of the lower leg from the horizontal.
        double knee2AngleFromXRad = VAL(LOWER_KNEE_ANGLE_FROM_GROUND) * PI / 180;
        // Calculate the X and Y distances covered by the lower leg.
        double lowerX = VAL(LEG_LOWER_LENGTH) * cos(knee2AngleFromXRad);
        double lowerY = VAL(LEG_LOWER_LENGTH) * sin(knee2AngleFromXRad);
        cerr << "\tLower: <" << lowerX << ", " << lowerY << ">" << endl;

        double hipAngleRad = NAN;
        double knee1AngleRad = NAN;
        double upperY = NAN;
        double middleY = NAN;
        bool keepSearching = true;
        do {
            // Let the hip angle be the current test value.
            hipAngleRad = min + (max - min) / 2;
            // Calculate the x distance covered by the upper leg.
            const double upperX = VAL(LEG_UPPER_LENGTH) * cos(hipAngleRad);
            // Calculate the x distance that must be covered by the middle leg.
            const double middleX = x - upperX - lowerX;
            cerr << "\tWith hip angle " << DEG(hipAngleRad) << ": " << "Upper X: " << upperX << ", Middle X: " << middleX << endl;
            if (middleX >= VAL(LEG_MIDDLE_LENGTH))
            {
                // The hip angle is too large.  Shrink the window and try again.
                max = hipAngleRad;
                cerr << "\tHip angle " << hipAngleRad << " too large, searching within bottom window: [" << min << ", " << max << "] (NAN upper knee angle)" << endl;

            }
            else
            {
                // Calculate the upper knee angle required by the X distance.
                knee1AngleRad = acos(middleX / VAL(LEG_MIDDLE_LENGTH));
                middleY = VAL(LEG_MIDDLE_LENGTH) * sin(knee1AngleRad);
                upperY = VAL(LEG_UPPER_LENGTH) * sin(hipAngleRad);
                const double difference = y + upperY - middleY - lowerY;
                cerr << "\t Upper knee: " << DEG(knee1AngleRad) << endl;
                cerr << "\tMiddle: <" << middleX << ", " << middleY << ">" << endl;
                cerr << "\tUpper: <" << upperX << ", " << upperY << ">" << endl;
                if (fabs(difference) < FIND_ANGLE_THRESHOLD)
                {
                    cerr << "\tClose enough." << endl;
                    keepSearching = false;
                }
                else if (difference < 0)
                {
                    min = hipAngleRad;
                    cerr << "\tHip angle " << hipAngleRad << " too small, searching within top window: [" << min << ", " << max << "]" << endl;
                }
                else
                {
                    max = hipAngleRad;
                    cerr << "\tHip angle " << hipAngleRad << " too large, searching within bottom window: [" << min << ", " << max << "]" << endl;
                }
            }
        } while (fabs(max - min) >= FIND_ANGLE_THRESHOLD && keepSearching);
        hipAngle = DEG(hipAngleRad);
        knee1Angle = DEG(hipAngleRad + knee1AngleRad);
        knee2Angle = DEG(knee2AngleFromXRad - knee1AngleRad);
        cerr << "Going with: " << hipAngle << ", " << knee1Angle << ", " << knee2Angle << endl;
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
    
    void drawLeg(const LegSide side, const double position, const double hipLatAngle, const double hipVertAngle, const double upperKneeAngle,
                 const double lowerKneeAngle, const double ankleAngle, const double xRotation);
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createBoxModel(int x, int y, int w, int h, char *label)
{ 
    return new BoxModel(x,y,w,h,label); 
}

// Create a sinusoidal transition from v0 to v0+vd
#define sinize(v0, vd, t0, t1, t) (v0 + vd * (cos((double)(t - t0) / (t1 - t0) * PI + PI) + 1) / 2)

// We are going to override (is that the right word?) the draw()
// method of ModelerView to draw out RobotArm
void BoxModel::draw()
{
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

    setAmbientColor(.1f,.1f,.1f);
    setDiffuseColor(0.8, 0.8, 0.8);
    glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
    glTranslated(0, VAL(HEIGHT), 0);
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
    //glRotated(sinize(0, 30, 0, 30, frame), -1, 0, 0);
    glTranslated(0, 0, VAL(HEAD_LENGTH)/4);
    glPushMatrix();
    glScaled(VAL(HEAD_WIDTH), VAL(HEAD_HEIGHT), VAL(HEAD_LENGTH));
    drawSphere(1);
    glPopMatrix();
    
    Leg(LEFT, VAL(_1_LEG_POS)).draw(VAL(L1_X_OFFSET), VAL(L1_Y_OFFSET), VAL(L1_HIP_ANGLE));
    Leg(LEFT, VAL(_2_LEG_POS)).draw(VAL(L2_X_OFFSET), VAL(L2_Y_OFFSET), VAL(L2_HIP_ANGLE));
    Leg(LEFT, VAL(_3_LEG_POS)).draw(VAL(L3_X_OFFSET), VAL(L3_Y_OFFSET), VAL(L3_HIP_ANGLE));
    Leg(LEFT, VAL(_4_LEG_POS)).draw(VAL(L4_X_OFFSET), VAL(L4_Y_OFFSET), VAL(L4_HIP_ANGLE));
    Leg(RIGHT, VAL(_1_LEG_POS)).draw(VAL(R1_X_OFFSET), VAL(R1_Y_OFFSET), VAL(R1_HIP_ANGLE));
    Leg(RIGHT, VAL(_2_LEG_POS)).draw(VAL(R2_X_OFFSET), VAL(R2_Y_OFFSET), VAL(R2_HIP_ANGLE));
    Leg(RIGHT, VAL(_3_LEG_POS)).draw(VAL(R3_X_OFFSET), VAL(R3_Y_OFFSET), VAL(R3_HIP_ANGLE));
    Leg(RIGHT, VAL(_4_LEG_POS)).draw(VAL(R4_X_OFFSET), VAL(R4_Y_OFFSET), VAL(R4_HIP_ANGLE));

    frame = (frame + 1) % 60;
}

void BoxModel::drawLeg(const LegSide side, const double position, const double hipLatAngle, const double hipVertAngle, const double upperKneeAngle,
                       const double lowerKneeAngle, const double ankleAngle, const double xRotation)
{
    const double legPositionRadians = position * PI;
    glPushMatrix();
    // Upper
    glTranslated(side * sin(legPositionRadians), 0, cos(legPositionRadians));
    //glRotated(sinize(0, xRotation, 0, 30, frame), 1, 0, 0);
    glRotated(hipLatAngle, 0, side, 0);
    glRotated(-hipVertAngle, 1, 0, 0);
    drawCylinder(VAL(LEG_UPPER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
    // Middle
    glTranslated(0, 0, VAL(LEG_UPPER_LENGTH));
    glRotated(upperKneeAngle, 1, 0, 0);
    drawCylinder(VAL(LEG_MIDDLE_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
    // Lower
    glTranslated(0, 0, VAL(LEG_MIDDLE_LENGTH));
    glRotated(lowerKneeAngle, 1, 0, 0);
    drawCylinder(VAL(LEG_LOWER_LENGTH), VAL(LEG_RADIUS), VAL(LEG_RADIUS));
    // Ankle
    glTranslated(0, 0, VAL(LEG_LOWER_LENGTH));
    glRotated(ankleAngle - 90, 1, 0, 0);
    drawCylinder(VAL(FOOT_LENGTH), VAL(LEG_RADIUS), VAL(TOE_RADIUS));
    glPopMatrix();
}

int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	//								 stepsize, defaultvalue)
    // You will want to modify this to accommodate your model.
    ModelerControl controls[NUMCONTROLS];
    controls[TIME]   = ModelerControl("Time", 0, 100, 0.1f, 0);
	controls[XPOS]   = ModelerControl("X Position", -5, 5, 0.1f, 0);
	controls[YPOS]   = ModelerControl("Y Position",  0, 5, 0.1f, 0);
	controls[ZPOS]   = ModelerControl("Z Position", -5, 5, 0.1f, 0);
	controls[HEIGHT] = ModelerControl("Height",      1, 5, 0.1f, 2);
    controls[DIRECTION] = ModelerControl("Direction", 0, 360, 0.1f, 0);
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
    controls[HEEL_TO_HIP_DISTANCE] = ModelerControl("Heel-to-Hip Distance", 0, 7, 0.1f, 5);

    controls[_1_LEG_POS] = ModelerControl("Leg Position (Front)", 0, 1, 0.1f, 0.17);
    controls[_2_LEG_POS] = ModelerControl("Leg Position (Front Mid.)", 0, 1, 0.1f, 0.34);
    controls[_3_LEG_POS] = ModelerControl("Leg Position (Back Mid.)", 0, 1, 0.1f, 0.45);
    controls[_4_LEG_POS] = ModelerControl("Leg Position (Back)", 0, 1, 0.1f, 0.53);

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

