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
    XPOS, YPOS, ZPOS, HEIGHT, NUMCONTROLS,
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
	glPushMatrix();
	glTranslated(VAL(XPOS), VAL(YPOS), VAL(ZPOS));
	glTranslated(-.5,0,-.5);
	glScaled(1, VAL(HEIGHT), 1);
	drawBox(1,1,1);
	glPopMatrix();



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

    // Initialize the modeler application with your model and the
    // appropriate array of controls.
    ModelerApplication::Instance()->Init(&createBoxModel, controls, NUMCONTROLS);


	// make sure we give back the memory to older OSs that don't 
	// clear your memory pool after shutdown.
    int Result = ModelerApplication::Instance()->Run();
	delete ModelerApplication::Instance();
	return Result;
}

