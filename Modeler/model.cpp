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



// This is a list of the controls for the RobotArm
// We'll use these constants to access the values 
// of the controls from the user interface.
enum BoxModelControls
{ 
    XPOS, YPOS, ZPOS, HEIGHT, DIRECTION, LOWER_ARM_LENGTH, LOWER_ARM_ANGLE, UPPER_ARM_LENGTH, UPPER_ARM_ANGLE, NUMCONTROLS,
};

// To make a BoxModel, we inherit off of ModelerView
class BoxModel : public ModelerView 
{
public:
    // Constructor for the model.  In your model, 
    // make sure you call the ModelerView constructor,
    // as done below.
    BoxModel(int x, int y, int w, int h, char *label) 
        : ModelerView(x,y,w,h,label) {}
    virtual void draw();
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
	glTranslated(-2.5,0,-2.5);
	drawBox(5,0.01f,5);
	glPopMatrix();

	// draw the box
	setAmbientColor(.1f,.1f,.1f);
	setDiffuseColor(0,1,.5f);
    glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
    glRotated(VAL(DIRECTION), 0, 1, 0);
    glPushMatrix();
    glScaled(1, VAL(HEIGHT), 1);
    glRotated(-90, 1, 0, 0);
    drawCylinder(1, 1, 1);
    glPopMatrix();
    glPushMatrix();
    glTranslated(0, VAL(HEIGHT), 0);
    setDiffuseColor(0.5, 0, 0);
    drawSphere(0.5);
    glPopMatrix();
    glTranslated(0, VAL(HEIGHT), 0);
    glRotated(VAL(LOWER_ARM_ANGLE), 0, 0, 1);
    glPushMatrix();
    glTranslated(-0.25, 0, -0.25);
    glScaled(0.5, VAL(LOWER_ARM_LENGTH), 0.5);
    setDiffuseColor(0, 0, 1);
    drawBox(1, 1, 1);
	glPopMatrix();
    glPushMatrix();
    glTranslated(0, VAL(LOWER_ARM_LENGTH), 0);
    setDiffuseColor(0.5, 0, 0);
    drawSphere(0.5);
    glPopMatrix();
    glTranslated(0, VAL(LOWER_ARM_LENGTH), 0);
    glRotated(VAL(UPPER_ARM_ANGLE), 0, 0, 1);
    glTranslated(-0.25, 0, -0.25);
    glScaled(0.5, VAL(UPPER_ARM_LENGTH), 0.5);
    setDiffuseColor(0, 0, 1);
    drawBox(1, 1, 1);
}

int main()
{
	// Initialize the controls
	// Constructor is ModelerControl(name, minimumvalue, maximumvalue, 
	//								 stepsize, defaultvalue)
    // You will want to modify this to accommodate your model.
    ModelerControl controls[NUMCONTROLS];
	controls[XPOS]   = ModelerControl("X Position", -5, 5, 0.1f, 0);
	controls[YPOS]   = ModelerControl("Y Position",  0, 5, 0.1f, 0);
	controls[ZPOS]   = ModelerControl("Z Position", -5, 5, 0.1f, 0);
	controls[HEIGHT] = ModelerControl("Height",      1, 5, 0.1f, 1);
    controls[DIRECTION] = ModelerControl("Direction", 0, 360, 0.1f, 0);
    controls[LOWER_ARM_LENGTH] = ModelerControl("Lower Arm Length", 1, 5, 0.1f, 2);
    controls[LOWER_ARM_ANGLE] = ModelerControl("Lower Arm Angle", -90, 90, 0.1f, -45);
    controls[UPPER_ARM_LENGTH] = ModelerControl("Upper Arm Length", 1, 5, 0.1f, 1);
    controls[UPPER_ARM_ANGLE] = ModelerControl("Upper Arm Angle", -90, 90, 0.1f, -45);
    
    // Initialize the modeler application with your model and the
    // appropriate array of controls.
    ModelerApplication::Instance()->Init(&createBoxModel, controls, NUMCONTROLS);


	// make sure we give back the memory to older OSs that don't 
	// clear your memory pool after shutdown.
    int Result = ModelerApplication::Instance()->Run();
	delete ModelerApplication::Instance();
	return Result;
}

