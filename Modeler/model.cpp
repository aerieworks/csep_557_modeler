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

#define PI 3.14159265359
#define degToRad(x) (x * PI / 180)

// This is a list of the controls for the RobotArm
// We'll use these constants to access the values 
// of the controls from the user interface.
enum BoxModelControls
{ 
    TIME, XPOS, YPOS, ZPOS, HEIGHT, DIRECTION,
    ABDOMEN_LENGTH, ABDOMEN_WIDTH, ABDOMEN_HEIGHT, ABDOMEN_OFFSET,
    HEAD_LENGTH, HEAD_WIDTH, HEAD_HEIGHT,
    LEG_RADIUS, TOE_RADIUS, LEG_UPPER_LENGTH, LEG_MIDDLE_LENGTH, LEG_LOWER_LENGTH, FOOT_LENGTH,
    LEFT_FRONT_LEG_POS, LEFT_FRONT_HIP_LAT_ANGLE, LEFT_FRONT_HIP_VERTICAL_ANGLE, LEFT_FRONT_KNEE_1_ANGLE, LEFT_FRONT_KNEE_2_ANGLE, LEFT_FRONT_ANKLE_ANGLE,
    LEFT_FWD_MID_LEG_POS, LEFT_FWD_MID_HIP_LAT_ANGLE, LEFT_FWD_MID_HIP_VERTICAL_ANGLE, LEFT_FWD_MID_KNEE_1_ANGLE, LEFT_FWD_MID_KNEE_2_ANGLE, LEFT_FWD_MID_ANKLE_ANGLE,
    LEFT_BACK_MID_LEG_POS, LEFT_BACK_MID_HIP_LAT_ANGLE, LEFT_BACK_MID_HIP_VERTICAL_ANGLE, LEFT_BACK_MID_KNEE_1_ANGLE, LEFT_BACK_MID_KNEE_2_ANGLE, LEFT_BACK_MID_ANKLE_ANGLE,
    LEFT_BACK_LEG_POS, LEFT_BACK_HIP_LAT_ANGLE, LEFT_BACK_HIP_VERTICAL_ANGLE, LEFT_BACK_KNEE_1_ANGLE, LEFT_BACK_KNEE_2_ANGLE, LEFT_BACK_ANKLE_ANGLE,
    RIGHT_FRONT_LEG_POS, RIGHT_FRONT_HIP_LAT_ANGLE, RIGHT_FRONT_HIP_VERTICAL_ANGLE, RIGHT_FRONT_KNEE_1_ANGLE, RIGHT_FRONT_KNEE_2_ANGLE, RIGHT_FRONT_ANKLE_ANGLE,
    RIGHT_FWD_MID_LEG_POS, RIGHT_FWD_MID_HIP_LAT_ANGLE, RIGHT_FWD_MID_HIP_VERTICAL_ANGLE, RIGHT_FWD_MID_KNEE_1_ANGLE, RIGHT_FWD_MID_KNEE_2_ANGLE, RIGHT_FWD_MID_ANKLE_ANGLE,
    RIGHT_BACK_MID_LEG_POS, RIGHT_BACK_MID_HIP_LAT_ANGLE, RIGHT_BACK_MID_HIP_VERTICAL_ANGLE, RIGHT_BACK_MID_KNEE_1_ANGLE, RIGHT_BACK_MID_KNEE_2_ANGLE, RIGHT_BACK_MID_ANKLE_ANGLE,
    RIGHT_BACK_LEG_POS, RIGHT_BACK_HIP_LAT_ANGLE, RIGHT_BACK_HIP_VERTICAL_ANGLE, RIGHT_BACK_KNEE_1_ANGLE, RIGHT_BACK_KNEE_2_ANGLE, RIGHT_BACK_ANKLE_ANGLE,
    NUMCONTROLS,
};

enum LegSide
{
    LEFT = -1,
    RIGHT = 1
};

// To make a BoxModel, we inherit off of ModelerView
class BoxModel : public ModelerView
{
public:
    // Constructor for the model.  In your model, 
    // make sure you call the ModelerView constructor,
    // as done below.
    BoxModel(int x, int y, int w, int h, char *label)
        : ModelerView(x,y,w,h,label)
    {
    }
    virtual void draw();
private:
    void drawLeg(const LegSide side, const double position, const double hipLatAngle, const double hipVertAngle, const double upperKneeAngle,
                 const double lowerKneeAngle, const double ankleAngle);
};

// We need to make a creator function, mostly because of
// nasty API stuff that we'd rather stay away from.
ModelerView* createBoxModel(int x, int y, int w, int h, char *label)
{ 
    return new BoxModel(x,y,w,h,label); 
}

// We'll be getting the instance of the application a lot;
// might as well have it as a macro.

#define VAL(x) (ModelerApplication::Instance()->GetControlValue(x))

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

	// Cephalothorax (head)
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(0.8, 0.8, 0.8);
    glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
    glTranslated(0, VAL(HEIGHT), 0);
    glRotated(VAL(DIRECTION), 0, 1, 0);
    glPushMatrix();
    glScaled(VAL(HEAD_WIDTH), VAL(HEAD_HEIGHT), VAL(HEAD_LENGTH));
    drawSphere(1);
    glPopMatrix();
    
    // Abdomen
    glPushMatrix();
    glRotated(180, 0, 1, 0);
    glTranslated(0, 0, VAL(ABDOMEN_OFFSET));
    glScaled(VAL(ABDOMEN_WIDTH), VAL(ABDOMEN_HEIGHT), VAL(ABDOMEN_LENGTH));
    drawSphere(1);
    glPopMatrix();
    
    drawLeg(LEFT, VAL(LEFT_FRONT_LEG_POS), VAL(LEFT_FRONT_HIP_LAT_ANGLE), VAL(LEFT_FRONT_HIP_VERTICAL_ANGLE) + 24*fabs(sin(VAL(TIME) / 4)), VAL(LEFT_FRONT_KNEE_1_ANGLE),
            VAL(LEFT_FRONT_KNEE_2_ANGLE), VAL(LEFT_FRONT_ANKLE_ANGLE));
    drawLeg(LEFT, VAL(LEFT_FWD_MID_LEG_POS), VAL(LEFT_FWD_MID_HIP_LAT_ANGLE), VAL(LEFT_FWD_MID_HIP_VERTICAL_ANGLE), VAL(LEFT_FWD_MID_KNEE_1_ANGLE),
            VAL(LEFT_FWD_MID_KNEE_2_ANGLE), VAL(LEFT_FWD_MID_ANKLE_ANGLE));
    drawLeg(LEFT, VAL(LEFT_BACK_MID_LEG_POS), VAL(LEFT_BACK_MID_HIP_LAT_ANGLE), VAL(LEFT_BACK_MID_HIP_VERTICAL_ANGLE), VAL(LEFT_BACK_MID_KNEE_1_ANGLE),
            VAL(LEFT_BACK_MID_KNEE_2_ANGLE), VAL(LEFT_BACK_MID_ANKLE_ANGLE));
    drawLeg(LEFT, VAL(LEFT_BACK_LEG_POS), VAL(LEFT_BACK_HIP_LAT_ANGLE), VAL(LEFT_BACK_HIP_VERTICAL_ANGLE), VAL(LEFT_BACK_KNEE_1_ANGLE),
            VAL(LEFT_BACK_KNEE_2_ANGLE), VAL(LEFT_BACK_ANKLE_ANGLE));

    drawLeg(RIGHT, VAL(RIGHT_FRONT_LEG_POS), VAL(RIGHT_FRONT_HIP_LAT_ANGLE), VAL(RIGHT_FRONT_HIP_VERTICAL_ANGLE), VAL(RIGHT_FRONT_KNEE_1_ANGLE),
            VAL(RIGHT_FRONT_KNEE_2_ANGLE), VAL(RIGHT_FRONT_ANKLE_ANGLE));
    drawLeg(RIGHT, VAL(RIGHT_FWD_MID_LEG_POS), VAL(RIGHT_FWD_MID_HIP_LAT_ANGLE), VAL(RIGHT_FWD_MID_HIP_VERTICAL_ANGLE),
            VAL(RIGHT_FWD_MID_KNEE_1_ANGLE), VAL(RIGHT_FWD_MID_KNEE_2_ANGLE), VAL(RIGHT_FWD_MID_ANKLE_ANGLE));
    drawLeg(RIGHT, VAL(RIGHT_BACK_MID_LEG_POS), VAL(RIGHT_BACK_MID_HIP_LAT_ANGLE), VAL(RIGHT_BACK_MID_HIP_VERTICAL_ANGLE),
            VAL(RIGHT_BACK_MID_KNEE_1_ANGLE), VAL(RIGHT_BACK_MID_KNEE_2_ANGLE), VAL(RIGHT_BACK_MID_ANKLE_ANGLE));
    drawLeg(RIGHT, VAL(RIGHT_BACK_LEG_POS), VAL(RIGHT_BACK_HIP_LAT_ANGLE), VAL(RIGHT_BACK_HIP_VERTICAL_ANGLE), VAL(RIGHT_BACK_KNEE_1_ANGLE),
            VAL(RIGHT_BACK_KNEE_2_ANGLE), VAL(RIGHT_BACK_ANKLE_ANGLE));
}

void BoxModel::drawLeg(const LegSide side, const double position, const double hipLatAngle, const double hipVertAngle, const double upperKneeAngle,
                       const double lowerKneeAngle, const double ankleAngle)
{
    const double legPositionRadians = position * PI;
    glPushMatrix();
    // Upper
    glTranslated(side * sin(legPositionRadians), 0, cos(legPositionRadians));
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
    controls[LEFT_FRONT_LEG_POS] = ModelerControl("L1: Leg Position", 0, 1, 0.1f, 0.17);
    controls[LEFT_FRONT_HIP_LAT_ANGLE] = ModelerControl("L1: Hip Lateral Angle", 0, 180, 0.1f, 40);
    controls[LEFT_FRONT_HIP_VERTICAL_ANGLE] = ModelerControl("L1: Hip Angle", 0, 180, 0.1f, 42);
    controls[LEFT_FRONT_KNEE_1_ANGLE] = ModelerControl("L1: First Knee Angle", 0, 180, 0.1f, 64);
    controls[LEFT_FRONT_KNEE_2_ANGLE] = ModelerControl("L1: Second Knee Angle", 0, 180, 0.1f, 55);
    controls[LEFT_FRONT_ANKLE_ANGLE] = ModelerControl("L1: Ankle Angle", 0, 180, 0.1f, 30.5);
    controls[LEFT_FWD_MID_LEG_POS] = ModelerControl("L2: Leg Position", 0, 1, 0.1f, 0.34);
    controls[LEFT_FWD_MID_HIP_LAT_ANGLE] = ModelerControl("L2: Hip Lateral Angle", 0, 180, 0.1f, 70);
    controls[LEFT_FWD_MID_HIP_VERTICAL_ANGLE] = ModelerControl("L2: Hip Angle", 0, 180, 0.1f, 42);
    controls[LEFT_FWD_MID_KNEE_1_ANGLE] = ModelerControl("L2: First Knee Angle", 0, 180, 0.1f, 64);
    controls[LEFT_FWD_MID_KNEE_2_ANGLE] = ModelerControl("L2: Second Knee Angle", 0, 180, 0.1f, 55);
    controls[LEFT_FWD_MID_ANKLE_ANGLE] = ModelerControl("L2: Ankle Angle", 0, 180, 0.1f, 30.5);
    controls[LEFT_BACK_MID_LEG_POS] = ModelerControl("L3: Leg Position", 0, 1, 0.1f, 0.45);
    controls[LEFT_BACK_MID_HIP_LAT_ANGLE] = ModelerControl("L3: Hip Lateral Angle", 0, 180, 0.1f, 110);
    controls[LEFT_BACK_MID_HIP_VERTICAL_ANGLE] = ModelerControl("L3: Hip Angle", 0, 180, 0.1f, 42);
    controls[LEFT_BACK_MID_KNEE_1_ANGLE] = ModelerControl("L3: First Knee Angle", 0, 180, 0.1f, 64);
    controls[LEFT_BACK_MID_KNEE_2_ANGLE] = ModelerControl("L3: Second Knee Angle", 0, 180, 0.1f, 55);
    controls[LEFT_BACK_MID_ANKLE_ANGLE] = ModelerControl("L3: Ankle Angle", 0, 180, 0.1f, 30.5);
    controls[LEFT_BACK_LEG_POS] = ModelerControl("L4: Leg Position", 0, 1, 0.1f, 0.53);
    controls[LEFT_BACK_HIP_LAT_ANGLE] = ModelerControl("L4: Hip Lateral Angle", 0, 180, 0.1f, 140);
    controls[LEFT_BACK_HIP_VERTICAL_ANGLE] = ModelerControl("L4: Hip Angle", 0, 180, 0.1f, 42);
    controls[LEFT_BACK_KNEE_1_ANGLE] = ModelerControl("L4: First Knee Angle", 0, 180, 0.1f, 64);
    controls[LEFT_BACK_KNEE_2_ANGLE] = ModelerControl("L4: Second Knee Angle", 0, 180, 0.1f, 55);
    controls[LEFT_BACK_ANKLE_ANGLE] = ModelerControl("L4: Ankle Angle", 0, 180, 0.1f, 30.5);
    
    controls[RIGHT_FRONT_LEG_POS] = ModelerControl("R1: Leg Position (R 1)", 0, 1, 0.1f, 0.17);
    controls[RIGHT_FRONT_HIP_LAT_ANGLE] = ModelerControl("R1: Hip Lateral Angle", 0, 180, 0.1f, 40);
    controls[RIGHT_FRONT_HIP_VERTICAL_ANGLE] = ModelerControl("R1: Hip Angle", 0, 180, 0.1f, 42);
    controls[RIGHT_FRONT_KNEE_1_ANGLE] = ModelerControl("R1: First Knee Angle", 0, 180, 0.1f, 64);
    controls[RIGHT_FRONT_KNEE_2_ANGLE] = ModelerControl("R1: Second Knee Angle", 0, 180, 0.1f, 55);
    controls[RIGHT_FRONT_ANKLE_ANGLE] = ModelerControl("R1: Ankle Angle", 0, 180, 0.1f, 30.5);
    controls[RIGHT_FWD_MID_LEG_POS] = ModelerControl("R2: Leg Position", 0, 1, 0.1f, 0.34);
    controls[RIGHT_FWD_MID_HIP_LAT_ANGLE] = ModelerControl("R2: Hip Lateral Angle", 0, 180, 0.1f, 70);
    controls[RIGHT_FWD_MID_HIP_VERTICAL_ANGLE] = ModelerControl("R2: Hip Angle", 0, 180, 0.1f, 42);
    controls[RIGHT_FWD_MID_KNEE_1_ANGLE] = ModelerControl("R2: First Knee Angle", 0, 180, 0.1f, 64);
    controls[RIGHT_FWD_MID_KNEE_2_ANGLE] = ModelerControl("R2: Second Knee Angle", 0, 180, 0.1f, 55);
    controls[RIGHT_FWD_MID_ANKLE_ANGLE] = ModelerControl("R2: Ankle Angle", 0, 180, 0.1f, 30.5);
    controls[RIGHT_BACK_MID_LEG_POS] = ModelerControl("R3: Leg Position", 0, 1, 0.1f, 0.45);
    controls[RIGHT_BACK_MID_HIP_LAT_ANGLE] = ModelerControl("R3: Hip Lateral Angle", 0, 180, 0.1f, 110);
    controls[RIGHT_BACK_MID_HIP_VERTICAL_ANGLE] = ModelerControl("R3: Hip Angle", 0, 180, 0.1f, 42);
    controls[RIGHT_BACK_MID_KNEE_1_ANGLE] = ModelerControl("R3: First Knee Angle", 0, 180, 0.1f, 64);
    controls[RIGHT_BACK_MID_KNEE_2_ANGLE] = ModelerControl("R3: Second Knee Angle", 0, 180, 0.1f, 55);
    controls[RIGHT_BACK_MID_ANKLE_ANGLE] = ModelerControl("R3: Ankle Angle", 0, 180, 0.1f, 30.5);
    controls[RIGHT_BACK_LEG_POS] = ModelerControl("R4: Leg Position", 0, 1, 0.1f, 0.53);
    controls[RIGHT_BACK_HIP_LAT_ANGLE] = ModelerControl("R4: Hip Lateral Angle", 0, 180, 0.1f, 140);
    controls[RIGHT_BACK_HIP_VERTICAL_ANGLE] = ModelerControl("R4: Hip Angle", 0, 180, 0.1f, 42);
    controls[RIGHT_BACK_KNEE_1_ANGLE] = ModelerControl("R4: First Knee Angle", 0, 180, 0.1f, 64);
    controls[RIGHT_BACK_KNEE_2_ANGLE] = ModelerControl("R4: Second Knee Angle", 0, 180, 0.1f, 55);
    controls[RIGHT_BACK_ANKLE_ANGLE] = ModelerControl("R4: Ankle Angle", 0, 180, 0.1f, 30.5);
    

    
    // Initialize the modeler application with your model and the
    // appropriate array of controls.
    ModelerApplication::Instance()->Init(&createBoxModel, controls, NUMCONTROLS);


	// make sure we give back the memory to older OSs that don't 
	// clear your memory pool after shutdown.
    int Result = ModelerApplication::Instance()->Run();
	delete ModelerApplication::Instance();
	return Result;
}

