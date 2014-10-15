#include "modelerview.h"
#include "modelerdraw.h"
#include "camera.h"

#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.h>
#include <FL/gl.h>
#include <OpenGL/glu.h>
#include <cstdio>

static const int	kMouseRotationButton			= FL_LEFT_MOUSE;
static const int	kMouseTranslationButton			= FL_MIDDLE_MOUSE;
static const int	kMouseZoomButton				= FL_RIGHT_MOUSE;

ModelerView::ModelerView(int x, int y, int w, int h, char *label)
: Fl_Gl_Window(x,y,w,h,label)
{
    m_camera = new Camera();
}

ModelerView::~ModelerView()
{
	delete m_camera;
}
int ModelerView::handle(int event)
{
    unsigned eventCoordX = Fl::event_x();
	unsigned eventCoordY = Fl::event_y();
	unsigned eventButton = Fl::event_button();
	unsigned eventState  = Fl::event_state();

	switch(event)	 
	{
	case FL_PUSH:
		{
			switch(eventButton)
			{
			case kMouseRotationButton:
				m_camera->clickMouse(kActionRotate, eventCoordX, eventCoordY );
				break;
			case kMouseTranslationButton:
				m_camera->clickMouse(kActionTranslate, eventCoordX, eventCoordY );
				break;
			case kMouseZoomButton:
				m_camera->clickMouse(kActionZoom, eventCoordX, eventCoordY );
				break;
			}
           // printf("push %d %d\n", eventCoordX, eventCoordY);
		}
		break;
	case FL_DRAG:
		{
			m_camera->dragMouse(eventCoordX, eventCoordY);
            //printf("drag %d %d\n", eventCoordX, eventCoordY);
		}
		break;
	case FL_RELEASE:
		{
			switch(eventButton)
			{
			case kMouseRotationButton:
			case kMouseTranslationButton:
			case kMouseZoomButton:
				m_camera->releaseMouse(eventCoordX, eventCoordY );
				break;
			}
          //  printf("release %d %d\n", eventCoordX, eventCoordY);
		}
		break;
	default:
		return 0;
	}
	
	redraw();

	return 1;
}

static GLfloat lightPosition0[] = { 4, 2, -4, 0 };
static GLfloat lightDiffuse0[]  = { 1,1,1,1 };
static GLfloat lightPosition1[] = { -2, 1, 5, 0 };
static GLfloat lightDiffuse1[]  = { 1, 1, 1, 1 };

void ModelerView::draw()
{
    if (!valid())
    {
        glShadeModel( GL_SMOOTH );
        glEnable( GL_DEPTH_TEST );
        glEnable( GL_LIGHTING );
		glEnable( GL_LIGHT0 );
        glEnable( GL_LIGHT1 );
		glEnable( GL_NORMALIZE );
    }

  	glViewport( 0, 0, w(), h() );
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30.0,float(w())/float(h()),1.0,100.0);
	m_camera->applyViewingTransform();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   /* m_camera->applyViewingTransform();*/

    glLightfv( GL_LIGHT0, GL_POSITION, lightPosition0 );
    glLightfv( GL_LIGHT0, GL_DIFFUSE, lightDiffuse0 );
    glLightfv( GL_LIGHT1, GL_POSITION, lightPosition1 );
    glLightfv( GL_LIGHT1, GL_DIFFUSE, lightDiffuse1 );

	ModelerDrawState *mds = ModelerDrawState::Instance();

	if(mds->m_rayFile)
	{
		float const_atten_coeff = 0.0f, lin_atten_coeff = 0.0f, quad_atten_coeff = 0.0f;

		glGetLightfv(GL_LIGHT0, GL_CONSTANT_ATTENUATION, &const_atten_coeff);
		glGetLightfv(GL_LIGHT0, GL_LINEAR_ATTENUATION, &lin_atten_coeff);
		glGetLightfv(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, &quad_atten_coeff);


		fprintf(mds->m_rayFile, "point_light {\n\tposition = (%f, %f, %f);\n\tcolor = (%f, %f, %f);\n\t",
			lightPosition0[0], lightPosition0[1], lightPosition0[2],
			lightDiffuse0[0], lightDiffuse0[1], lightDiffuse0[1]);
		fprintf(mds->m_rayFile, "constant_attenuation_coeff = %f;\n\tlinear_attenuation_coeff = %f;\n\tquadratic_attenuation_coeff = %f; }\n",
			const_atten_coeff, lin_atten_coeff, quad_atten_coeff);
		
		glGetLightfv(GL_LIGHT1, GL_CONSTANT_ATTENUATION, &const_atten_coeff);
		glGetLightfv(GL_LIGHT1, GL_LINEAR_ATTENUATION, &lin_atten_coeff);
		glGetLightfv(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, &quad_atten_coeff);

		fprintf(mds->m_rayFile, "point_light {\n\tposition = (%f, %f, %f);\n\tcolor = (%f, %f, %f);\n\t",
			lightPosition1[0], lightPosition1[1], lightPosition1[2],
			lightDiffuse1[0], lightDiffuse1[1], lightDiffuse1[1]);
		fprintf(mds->m_rayFile, "constant_attenuation_coeff = %f;\n\tlinear_attenuation_coeff = %f;\n\tquadratic_attenuation_coeff = %f; }\n",
			const_atten_coeff, lin_atten_coeff, quad_atten_coeff);

	}

    // If you want to change lighting, perspective, etc ...
    // Simply don't call this function, and take care of 
    // this stuff yourself in your model.  This will make
    // porting your model to Animator much easier.  
}