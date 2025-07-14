//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.1.1 $Rev: 1928 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"
//---------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//---------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cGenericTool* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;


//---------------------------------------------------------------------------
// ODE MODULE VARIABLES
//---------------------------------------------------------------------------

// ODE world
cODEWorld* ODEWorld;

// ODE objects
cODEGenericBody* ODEBody0;
cODEGenericBody* ODEBody1;
cODEGenericBody* ODEBody2;

// ODE objects
cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;


//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

string resourceRoot;
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);


//===========================================================================
/*
    DEMO:    ODE_cubic.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. In this scene we create 3
    cubic meshes that we individually attach to ODE bodies. Haptic interactions
    are computer by using the finger-proxy haptic model and forces are
    propagated to the ODE representation.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 40-ODE-cubic" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[g] - Enable/Disable gravity" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = 0.8 * screenH;
    windowH = 0.5 * screenH;
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY; 

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //-----------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d (2.5, 0.0, 0.3),    // camera position (eye)
                cVector3d (0.0, 0.0,-0.5),    // lookat position (target)
                cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(2.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    world->addChild(light);    

    // enable light source
    light->setEnabled(true);                   

    // position the light source
    light->setLocalPos(0.0, 0.0, 1.2);

    // define the direction of the light beam
    light->setDir(0.0, 0.0,-1.0);

    // set uniform concentration level of light 
    light->setSpotExponent(0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(45);


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (gripper or pointer)
    if (hapticDeviceInfo.m_actuatedGripper)
    {
        tool = new cToolGripper(world);
    }
    else
    {
        tool = new cToolCursor(world);
    }

    // insert tool into world
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.3);

    // define a radius for the virtual tool contact points (sphere)
    double toolRadius = 0.06;
    tool->setRadius(toolRadius, toolRadius);

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelHapticRate);


    //-----------------------------------------------------------------------
    // CREATE ODE WORLD AND OBJECTS
    //-----------------------------------------------------------------------

    //////////////////////////////////////////////////////////////////////////
    // ODE WORLD
    //////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.00, 0.00,-9.81));

    // define damping properties
    ODEWorld->setAngularDamping(0.00002);
    ODEWorld->setLinearDamping(0.00002);


    //////////////////////////////////////////////////////////////////////////
    // 3 ODE BLOCKS
    //////////////////////////////////////////////////////////////////////////

    // create a new ODE object that is automatically added to the ODE world
    ODEBody0 = new cODEGenericBody(ODEWorld);
    ODEBody1 = new cODEGenericBody(ODEWorld);
    ODEBody2 = new cODEGenericBody(ODEWorld);

    // create a virtual mesh  that will be used for the geometry representation of the dynamic body
    cMesh* object0 = new cMesh();
    cMesh* object1 = new cMesh();
    cMesh* object2 = new cMesh();

    // create a cube mesh
    double size = 0.40;
    cCreateBox(object0, size, size, size);
    object0->createAABBCollisionDetector(toolRadius);

    cCreateBox(object1, size, size, size);
    object1->createAABBCollisionDetector(toolRadius);

    cCreateBox(object2, size, size, size);
    object2->createAABBCollisionDetector(toolRadius);
      
    // define some material properties for each cube
    cMaterial mat0, mat1, mat2;
    mat0.setRedIndian();
    mat0.setStiffness(0.3 * maxStiffness);
    mat0.setDynamicFriction(0.6);
    mat0.setStaticFriction(0.6);
    object0->setMaterial(mat0);

    mat1.setBlueRoyal();
    mat1.setStiffness(0.3 * maxStiffness);
    mat1.setDynamicFriction(0.6);
    mat1.setStaticFriction(0.6);
    object1->setMaterial(mat1);

    mat2.setGreenDarkSea();
    mat2.setStiffness(0.3 * maxStiffness);
    mat2.setDynamicFriction(0.6);
    mat2.setStaticFriction(0.6);
    object2->setMaterial(mat2);

    // add mesh to ODE object
    ODEBody0->setImageModel(object0);
    ODEBody1->setImageModel(object1);
    ODEBody2->setImageModel(object2);

    // create a dynamic model of the ODE object. Here we decide to use a box just like
    // the object mesh we just defined
    ODEBody0->createDynamicBox(size, size, size);
    ODEBody1->createDynamicBox(size, size, size);
    ODEBody2->createDynamicBox(size, size, size);

    // define some mass properties for each cube
    ODEBody0->setMass(0.05);
    ODEBody1->setMass(0.15);
    ODEBody2->setMass(0.25);

    // set position of each cube
    ODEBody0->setLocalPos(0.0,-0.6,-0.5);
    ODEBody1->setLocalPos(0.0, 0.6,-0.5);
    ODEBody2->setLocalPos(0.0, 0.0,-0.5);

    // rotate central cube 45 degrees around z-axis
    ODEBody0->rotateAboutGlobalAxisDeg(0,0,1, 45);


    //////////////////////////////////////////////////////////////////////////
    // 6 ODE INVISIBLE WALLS
    //////////////////////////////////////////////////////////////////////////

    // we create 6 static walls to contains the 3 cubes within a limited workspace
    ODEGPlane0 = new cODEGenericBody(ODEWorld);
    ODEGPlane1 = new cODEGenericBody(ODEWorld);
    ODEGPlane2 = new cODEGenericBody(ODEWorld);
    ODEGPlane3 = new cODEGenericBody(ODEWorld);
    ODEGPlane4 = new cODEGenericBody(ODEWorld);
    ODEGPlane5 = new cODEGenericBody(ODEWorld);

    double width = 1.0;
    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0, width/2.5), cVector3d(0.0, 0.0 ,-1.0));
    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -width), cVector3d(0.0, 0.0 , 1.0));
    ODEGPlane2->createStaticPlane(cVector3d(0.0,  width, 0.0), cVector3d(0.0,-1.0, 0.0));
    ODEGPlane3->createStaticPlane(cVector3d(0.0, -width, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d( width, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(-0.8 * width, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));


    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////

    // create a mesh that represents the ground
    cMesh* ground = new cMesh();
    ODEWorld->addChild(ground);

    // create a plane
    double groundSize = 3.0;
    cCreatePlane(ground, groundSize, groundSize);

    // position ground in world where the invisible ODE plane is located (ODEGPlane1)
    ground->setLocalPos(0.0, 0.0, -1.0);

    // define some material properties and apply to mesh
    cMaterial matGround;
    matGround.setStiffness(0.3 * maxStiffness);
    matGround.setDynamicFriction(0.5);
    matGround.setStaticFriction(2.0);
	matGround.m_ambient.set(0.2f, 0.2f, 0.2f);
	matGround.m_diffuse.set(0.7f, 0.7f, 0.7f);
	matGround.m_specular.set(0.1f, 0.1f, 0.1f);
    matGround.m_emission.setGrayLevel(0.3);
    ground->setMaterial(matGround);

    // setup collision detector
    ground->createAABBCollisionDetector(toolRadius);
	//////////////////////////////////////////////////////////////////////////
	// Ceiling
	//////////////////////////////////////////////////////////////////////////
	// create a mesh that represents the Ceiling
	cMesh* Ceiling = new cMesh();
	ODEWorld->addChild(Ceiling);

	// create a plane
	double CeilingSize = 3.0;
	cMatrix3d rot_ceiling;
	rot_ceiling.setAxisAngleRotationRad(1, 0, 0, cDegToRad(180));  // 180° rotation about X-axis
	cCreatePlane(Ceiling, CeilingSize, CeilingSize, cVector3d(0, 0, 0.4), rot_ceiling);

	// define some material properties and apply to mesh
	cMaterial matCeiling;
	matCeiling.setStiffness(0.3 * maxStiffness);
	matCeiling.setDynamicFriction(0.5);
	matCeiling.setStaticFriction(2.0);
	matCeiling.m_ambient.set(0.2f, 0.2f, 0.2f);
	matCeiling.m_diffuse.set(0.7f, 0.7f, 0.7f);
	matCeiling.m_specular.set(0.1f, 0.1f, 0.1f);
	matCeiling.m_emission.setGrayLevel(0.3);
	Ceiling->setMaterial(matCeiling);

	// setup collision detector
	Ceiling->createAABBCollisionDetector(toolRadius);

	//////////////////////////////////////////////////////////////////////////
	// BackWall
	//////////////////////////////////////////////////////////////////////////
	// create a mesh that represents the Ceiling
	cMesh* BackWall = new cMesh();
	ODEWorld->addChild(BackWall);


	cMatrix3d rot_backwall;
	rot_backwall.setAxisAngleRotationRad(0, 1, 0, cDegToRad(90));  // 180° rotation about X-axis
	cCreatePlane(BackWall, 1.8, 3.4, cVector3d(-2, 0, -0.3), rot_backwall);

	// define some material properties and apply to mesh
	cMaterial matBackWall;
	matBackWall.setStiffness(0.3 * maxStiffness);
	matBackWall.setDynamicFriction(0.5);
	matBackWall.setStaticFriction(2.0);
	matBackWall.m_ambient.set(0.2f, 0.2f, 0.2f);
	matBackWall.m_diffuse.set(0.7f, 0.7f, 0.7f);
	matBackWall.m_specular.set(0.1f, 0.1f, 0.1f);
	matBackWall.m_emission.setGrayLevel(0.5);
	BackWall->setMaterial(matBackWall);

	// setup collision detector
	BackWall->createAABBCollisionDetector(toolRadius);
	//////////////////////////////////////////////////////////////////////////
	// LeftWall
	//////////////////////////////////////////////////////////////////////////
	// create a mesh that represents the Ceiling
	cMesh* LeftWall = new cMesh();
	ODEWorld->addChild(LeftWall);


	cMatrix3d rot_LeftWall;
	rot_LeftWall.setAxisAngleRotationRad(1, 0, 0, cDegToRad(90));  // 180° rotation about X-axis
	cCreatePlane(LeftWall, 3.0, 1.8, cVector3d(0, -1.5, -0.3), rot_LeftWall);

	// define some material properties and apply to mesh
	cMaterial matLeftWall;
	matLeftWall.setStiffness(0.3 * maxStiffness);
	matLeftWall.setDynamicFriction(0.5);
	matLeftWall.setStaticFriction(2.0);
	matLeftWall.m_ambient.set(0.2f, 0.2f, 0.2f);
	matLeftWall.m_diffuse.set(0.7f, 0.7f, 0.7f);
	matLeftWall.m_specular.set(0.1f, 0.1f, 0.1f);
	matLeftWall.m_emission.setGrayLevel(0.5);
	LeftWall->setMaterial(matLeftWall);

	// setup collision detector
	LeftWall->createAABBCollisionDetector(toolRadius);

	//////////////////////////////////////////////////////////////////////////
	// RightWall
	//////////////////////////////////////////////////////////////////////////
	// create a mesh that represents the Ceiling
	cMesh* RightWall = new cMesh();
	ODEWorld->addChild(RightWall);


	cMatrix3d rot_RightWall;
	rot_RightWall.setAxisAngleRotationRad(1, 0, 0, cDegToRad(90));  // 180° rotation about X-axis
	cCreatePlane(RightWall, 3.0, 1.8, cVector3d(0, 1.5, -0.3), rot_RightWall);

	// define some material properties and apply to mesh
	cMaterial matRightWall;
	matRightWall.setStiffness(0.3 * maxStiffness);
	matRightWall.setDynamicFriction(0.5);
	matRightWall.setStaticFriction(2.0);
	matRightWall.m_ambient.set(0.2f, 0.2f, 0.2f);
	matRightWall.m_diffuse.set(0.7f, 0.7f, 0.7f);
	matRightWall.m_specular.set(0.1f, 0.1f, 0.1f);
	matRightWall.m_emission.setGrayLevel(0.5);
	RightWall->setMaterial(matRightWall);

	// setup collision detector
	RightWall->createAABBCollisionDetector(toolRadius);

//	/////////////////////////////////////////////////////////////////////////
//	// CANVAS:
//	////////////////////////////////////////////////////////////////////////
//
//	// create a mesh
//	cMesh* canvas= new cMesh();
//	cImagePtr canvasOriginal;
//
//	// create a plane
//	cCreatePlane(canvas, 0.5, 0.5);
//
//	// create collision detector
//	canvas->createBruteForceCollisionDetector();
//
//	// add object to world
//	world->addChild(canvas);
//
//	// set the position of the object
//	canvas->setLocalPos(-0.25, 0.3, 0.0);
//	canvas->rotateAboutGlobalAxisRad(cVector3d(0, 1, 0), cDegToRad(90));
//	canvas->rotateAboutGlobalAxisRad(cVector3d(1, 0, 0), cDegToRad(90));
//
//	// set graphic properties
//	canvas->m_texture = cTexture2d::create();
//	// declare & initialize fileload
//	bool fileload = canvas->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/canvas.jpg"));
//
//	if (!fileload)
//	{
//#if defined(_MSVC)
//		fileload = canvas->m_texture->loadFromFile("../../../bin/resources/images/canvas.jpg");
//#endif
//	}
//	if (!fileload)
//	{
//		cout << "Error - Texture image failed to load correctly." << endl;
//		close();
//		return (-1);
//	}
//
//	// create a copy of canvas so that we can clear page when requested
//	canvasOriginal = canvas->m_texture->m_image->copy();
//
//	// we disable lighting properties for canvas
//	canvas->setUseMaterial(false);
//
//	// enable texture mapping
//	canvas->setUseTexture(true);
//
//	// set haptic properties
//	canvas->m_material->setStiffness(0.5 * maxStiffness);
//	canvas->m_material->setStaticFriction(0.20);
//	canvas->m_material->setDynamicFriction(0.15);
//	canvas->m_material->setHapticTriangleSides(true, false);
    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------
    
    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        // exit application
        exit(0);
    }

    // option g: enable/disable gravity
    if (key == 'g')
    {
        if (ODEWorld->getGravity().length() > 0.0)
        {
            ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));
        }
        else
        {
            ODEWorld->setGravity(cVector3d(0.0, 0.0,-9.81));
        }
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
}

//---------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic rate label
    labelHapticRate->setText (cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME    
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = cClamp(clock.getCurrentTimeSeconds(), 0.0001, 0.001);

        // restart the simulation clock
        clock.reset();
        clock.start();

        // update frequency counter
        frequencyCounter.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();


        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////

        // for each interaction point of the tool we look for any contact events
        // with the environment and apply forces accordingly
        int numInteractionPoints = tool->getNumHapticPoints();
        for (int i=0; i<numInteractionPoints; i++)
        {
            // get pointer to next interaction point of tool
            cHapticPoint* interactionPoint = tool->getHapticPoint(i);

            // check all contact points
            int numContacts = interactionPoint->getNumCollisionEvents();
            for (int i=0; i<numContacts; i++)
            {
                cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the ODE object itself.
                cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                // cast to ODE object
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                // if ODE object, we apply interaction forces
                if (ODEobject != NULL)
                {
                    ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
                                                       collisionEvent->m_globalPos);
                }
            }
        }

        // update simulation
        ODEWorld->updateDynamics(timeInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

