/*
		2011 Takahiro Harada
*/
#define _CRT_SECURE_NO_WARNINGS 
#define GLUT_DISABLE_ATEXIT_HACK

#include <Common/DeviceUtils/DeviceDraw.h>

#include <stdlib.h>
#include <stdio.h>

#include <assert.h>
#include <math.h>

#pragma warning( disable : 4996 )




#define INIT_HEIGHT 720
#define INIT_WIDTH 1024

int g_wWidth,g_wHeight;
bool g_fullScreen = false;
bool g_autoMove = false;
int g_mouseX, g_mouseY;
int g_mouseStatus = 0;
float g_rotationX = 0.0, g_rotationY = 0.0;
float g_translationX = 0.0f;
float g_translationY = 0.0f;
float g_translationZ = 0.0f;

float g_fov = 35.f;

#include <common/Utils/Stopwatch.h>

#include <Demos/Dem2Demo.h>


typedef Demo* CreateFunc(const DeviceDataBase* deviceData);

CreateFunc* createFuncs[] = {
	Dem2Demo::createFunc,
};


Demo* m_pDemo;

void drawDemo()
{
	m_pDemo->render();
}

void drawDemo2D()
{
	float spacing = 0.04f*INIT_HEIGHT/g_wHeight;
	float4 pos;
	pos.x = pos.y = pos.z = 0.f;

	pos.x = -1.0f+spacing;
	pos.y = 1.0f-spacing;

	for(int i=0; i<m_pDemo->m_nTxtLines; i++)
	{
		pxDrawText( m_pDemo->m_txtBuffer[i], pos );
		pos.y -= spacing;
	}
}

void step()
{
	if( m_pDemo->m_stepCount == -153 )
	{
		g_autoMove = false;
	}

	m_pDemo->stepDemo();
}

void initDemo( DeviceDataBase* deviceData = NULL )
{
	m_pDemo = NULL;
	m_pDemo = createFuncs[0]( deviceData );

	CLASSERT( m_pDemo );
	m_pDemo->init();
	m_pDemo->reset();
}

void resetDemo()
{
	m_pDemo->resetDemo();
}

void finishDemo()
{
	if( m_pDemo ) delete m_pDemo;
	m_pDemo = 0;
}

void keyListenerDemo(unsigned char key)
{
	m_pDemo->keyListener(key);
}

void keySpecialListenerDemo(int key, int x, int y)
{
	m_pDemo->keySpecialListener(key);
}

//---



bool drawText = true;

void setModelViewMatrix()
{
	glLoadIdentity();
	glTranslatef(g_translationX, g_translationY, g_translationZ);
	glRotatef(g_rotationX, 1.0, 0.0, 0.0);
	glRotatef(g_rotationY, 0.0, 1.0, 0.0);
}
void setPerspectiveProjMatrix()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(g_fov, (double)g_wWidth / (double)g_wHeight, 0.001, 100.0);
	gluLookAt(.0, .0, 4.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	glMatrixMode(GL_MODELVIEW);
}
void setOrthoProjMatrix()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1,1,-1,1,-100,100);
	glMatrixMode(GL_MODELVIEW);
}
void display(void)
{
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	if( m_pDemo->m_enableLighting )
	{
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	}
	else
	{
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	}

	glLoadIdentity();
	{
		setPerspectiveProjMatrix();
		setModelViewMatrix();
	}
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	glColor3f(1, 1, 1);

	if( m_pDemo->m_enableLighting )
	{
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_NORMALIZE);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		float lightPos[] = {0,2,2,1};
		glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
		float s = 0.3f;
		float ambColor[] = {s,s,s,1.f};
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambColor);
	}
	else
	{
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	}

	drawDemo();

	if( m_pDemo->m_enableLighting )
	{
		glDisable(GL_LIGHT0);
		glDisable(GL_LIGHTING);
	}

	if( drawText )
	{
		setOrthoProjMatrix();
		glLoadIdentity();
		drawDemo2D();
	}

	glutSwapBuffers();
}
void idle(void)
{
	if( g_autoMove )
		step();

	glutPostRedisplay();
}
void resize(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1,1,-1,1,-100,100);
	gluLookAt(.0, .0, 4.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	glMatrixMode(GL_MODELVIEW);
	g_wWidth=w;g_wHeight=h;
}
void init()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_DEPTH_TEST);

	initDemo();
	

	{
//		char txt[128];
//		sprintf(txt,"Dem: %dK(%d)", m_pSim->m_numParticles/1024, m_pSim->m_numParticles);
//		glutSetWindowTitle(txt);
	}
}

void mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN) {
		g_mouseStatus |= 1<<button;
	} else if (state == GLUT_UP) {
		g_mouseStatus = 0;
	}
	if (glutGetModifiers() & GLUT_ACTIVE_SHIFT
		 && state == GLUT_DOWN){
		g_mouseStatus |= 2 << 2;
	}
	
	g_mouseX = x;
	g_mouseY = y;
	glutPostRedisplay();
}
void motion(int x, int y)
{
	float dx, dy;
	dx = x - g_mouseX;
	dy = y - g_mouseY;
	
	if(g_mouseStatus & (2 << 2) && g_mouseStatus & 1){
		
	}else if (g_mouseStatus & 1) {
		g_rotationX += dy * 0.2;
		g_rotationY += dx * 0.2;
	} else if (g_mouseStatus & (2<<2)) {
		g_translationZ += dy * 0.01;
	} else if (g_mouseStatus & 4){
		g_translationX += dx * 0.005;
		g_translationY -= dy * 0.005;
	}
	
	
	g_mouseX = x;
	g_mouseY = y;
}
void keyboard(unsigned char key, int x, int y)
{
	float dz = 0.1f;
	const char str = key;

	keyListenerDemo(key);
	
	switch (key) {
		case 'a':
			g_autoMove = !g_autoMove;
			break;
		case 'r':
			resetDemo();
			break;
		case' ':
			step();
			break;
		case 't':
			drawText = !drawText;
			break;
		case 'l':
			m_pDemo->m_enableLighting = !m_pDemo->m_enableLighting;
			break;
		case 'q':
		case 'Q':
		case '\033':
			finishDemo();
			exit(0);
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			break;
		default:
			break;
	}
	glutPostRedisplay();
}
int main(int argc, char *argv[])
{
	glutInitWindowSize(INIT_WIDTH,INIT_HEIGHT);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA| GLUT_DEPTH | GLUT_DOUBLE | GLUT_ALPHA);

	if( g_fullScreen )
	{
		glutEnterGameMode();
	}
	else
	{
		glutCreateWindow(argv[0]);
	}

	glutDisplayFunc(display);
	glutReshapeFunc(resize);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(keySpecialListenerDemo);
	glutIdleFunc(idle);
	init();
	glutMainLoop();
	
	return 0;
}
