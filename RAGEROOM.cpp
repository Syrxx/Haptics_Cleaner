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

// a virtual object
cMultiMesh* object0;
cMultiMesh* object1;
cMultiMesh* object2;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> hapticDevice;

// a virtual tool representing the haptic device in the scene
cGenericTool* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

const int GRID = 10;
const int GRID_X = 10;
const int GRID_Y = 10;
cMesh* CeilingBlocks[GRID][GRID];
cMesh* GroundBlocks[GRID][GRID];
cMesh* BackWallBlocks[GRID][GRID];
cMesh* LeftWallBlocks[GRID_X][GRID_Y];
cMesh* RightWallBlocks[GRID_X][GRID_Y];
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

// last mouse position
int mouseX;
int mouseY;

string resourceRoot;
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to handle mouse click
void mouseClick(int button, int state, int x, int y);

// callback to handle mouse motion when button is pressed
void mouseMove(int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);

// Grabbing logic

bool graspActive;
cODEGenericBody* graspObject;
cVector3d graspPos;
cShapeLine* graspLine;
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
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMove);
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

	camera->setSphericalDeg(
		2.3,  // radius (distance from center)X
		5,  // polar angle (looking straight down Z)Z
		0  // azimuth angle Y
		);

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
    light->setLocalPos(0.0, 0.0, 2.0);

    // define the direction of the light beam
    light->setDir(0.0, 0.0,-1.0);

    // set uniform concentration level of light 
    light->setSpotExponent(0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(180);


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
    if (0)//(hapticDeviceInfo.m_actuatedGripper)
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

	// if the haptic device has a gripper, enable it as a user switch
	hapticDevice->setEnableGripperUserSwitch(true);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    //tool->setWorkspaceRadius(1.7);//original
	tool->setWorkspaceRadius(3.4);//Touch the back wall
    // define a radius for the virtual tool contact points (sphere)
    double toolRadius = 0.04;
    tool->setRadius(toolRadius, toolRadius);

	// hide the device sphere. only show proxy.
	tool->setShowContactPoints(true, false);

	// create a white cursor
	((cToolCursor*)tool)->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

	// oriente tool with camera
	tool->setLocalRot(camera->getLocalRot());

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
    ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0, 0.6), cVector3d(0.0, 0.0 ,-1.0));
    ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -width), cVector3d(0.0, 0.0 , 1.0));
    ODEGPlane2->createStaticPlane(cVector3d(0.0,  1.5, 0.0), cVector3d(0.0,-1.0, 0.0));
    ODEGPlane3->createStaticPlane(cVector3d(0.0, -1.5, 0.0), cVector3d(0.0, 1.0, 0.0));
    ODEGPlane4->createStaticPlane(cVector3d( width, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
    ODEGPlane5->createStaticPlane(cVector3d(-1.5, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));


    //////////////////////////////////////////////////////////////////////////
    // GROUND
    //////////////////////////////////////////////////////////////////////////
//
//    // create a mesh that represents the ground
//    cMesh* ground = new cMesh();
//    ODEWorld->addChild(ground);
//
//    // create a plane
//    double groundSize = 3.0;
//    cCreatePlane(ground, groundSize, groundSize);
//
//    // position ground in world where the invisible ODE plane is located (ODEGPlane1)
//    ground->setLocalPos(0.0, 0.0, -1.0);
//
//	// create texture property
//	cTexture2dPtr texture_gd = cTexture2d::create();
//	ground->setTexture(texture_gd);
//
//	// load texture from file
//	bool fileload = ground->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/brick-color.jpg"));
//	if (!fileload)
//	{
//#if defined(_MSVC)
//		fileload = ground->m_texture->loadFromFile("../../../../../bin/resources/images/brick-color.png");
//#endif
//	}
//	if (!fileload)
//	{
//		cout << "Error - Texture image 'brick-color.png' failed to load correctly." << endl;
//		close();
//		return (-1);
//	}
//
//	// define some material properties and apply to mesh
//	ground->setUseMaterial(false);
//	ground->setUseTexture(true);
//	ground->m_material->setStiffness(0.5 * maxStiffness);
//	ground->m_material->setStaticFriction(0.20);
//	ground->m_material->setDynamicFriction(0.15);
//	ground->m_material->setHapticTriangleSides(true, false);
//
//	//// setup collision detector
//	ground->createAABBCollisionDetector(toolRadius);
//
	// 地面格子数量（10x10）
	//////////////////////////////////////////////////////////////////////////
	// GROUND
	//////////////////////////////////////////////////////////////////////////
	double groundSize = 3.0;
	double cellSize_ground = groundSize / GRID;

	for (int i = 0; i < GRID; ++i)
	{
		for (int j = 0; j < GRID; ++j)
		{
			// 创建 mesh
			GroundBlocks[i][j] = new cMesh();
			ODEWorld->addChild(GroundBlocks[i][j]);

			// 计算格子中心点位置
			double x_center = -groundSize / 2.0 + (i + 0.5) * cellSize_ground;
			double y_center = -groundSize / 2.0 + (j + 0.5) * cellSize_ground;
			double z_center = -1.0;  // 地面 Z 坐标

			cVector3d pos(x_center, y_center, z_center);
			cMatrix3d rot;
			rot.identity();  // 地面不旋转

			// 创建格子平面
			cCreatePlane(GroundBlocks[i][j], cellSize_ground, cellSize_ground, pos, rot);

			// 使用材质颜色或纹理（这里先使用颜色）
			GroundBlocks[i][j]->m_material->setBlue();
			GroundBlocks[i][j]->setUseMaterial(true);
			GroundBlocks[i][j]->setUseTexture(false); // 如要纹理，改为 true 并加载

			// 设置触觉属性
			GroundBlocks[i][j]->m_material->setStiffness(0.5 * maxStiffness);
			GroundBlocks[i][j]->m_material->setStaticFriction(0.20);
			GroundBlocks[i][j]->m_material->setDynamicFriction(0.15);
			GroundBlocks[i][j]->m_material->setHapticTriangleSides(true, false);

			// 启用碰撞检测
			GroundBlocks[i][j]->createAABBCollisionDetector(toolRadius);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// Ceiling
	//////////////////////////////////////////////////////////////////////////
//	// create a mesh that represents the Ceiling
//	cMesh* Ceiling = new cMesh();
//	ODEWorld->addChild(Ceiling);
//
//	// create a plane
//	double CeilingSize = 3.0;
//	cMatrix3d rot_ceiling;
//	rot_ceiling.setAxisAngleRotationRad(1, 0, 0, cDegToRad(180));  // 180° rotation about X-axis
//	cCreatePlane(Ceiling, CeilingSize, CeilingSize, cVector3d(0, 0, 0.6), rot_ceiling);
//
//	// create texture property
//	cTexture2dPtr texture_cl = cTexture2d::create();
//	Ceiling->setTexture(texture_cl);
//
////	// load texture from file
////	fileload = Ceiling->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/brick-color.jpg"));
////	if (!fileload)
////	{
////#if defined(_MSVC)
////		fileload = Ceiling->m_texture->loadFromFile("../../../../../bin/resources/images/background.png");
////#endif
////	}
////	if (!fileload)
////	{
////		cout << "Error - Texture image 'background.png' failed to load correctly." << endl;
////		close();
////		return (-1);
////	}
//
//	// define some material properties and apply to mesh
//	Ceiling->setUseMaterial(false);
//	Ceiling->setUseTexture(true);
//	Ceiling->m_material->setStiffness(0.5 * maxStiffness);
//	Ceiling->m_material->setStaticFriction(0.20);
//	Ceiling->m_material->setDynamicFriction(0.15);
//	Ceiling->m_material->setHapticTriangleSides(true, false);
//
//	//// setup collision detector
//	Ceiling->createAABBCollisionDetector(toolRadius);

	//////////////////////////////////////////////////////////////////////////
	// Ceiling
	//////////////////////////////////////////////////////////////////////////
	double CeilingSize = 3.0;           // 整个天花板是 3x3 米
	double cellSize_ceiling = CeilingSize / GRID;
	cMatrix3d rot_ceiling;
	rot_ceiling.setAxisAngleRotationRad(1, 0, 0, cDegToRad(180));


	for (int i = 0; i < GRID; ++i)
	{
		for (int j = 0; j < GRID; ++j)
		{
			// 创建 mesh
			CeilingBlocks[i][j] = new cMesh();
			ODEWorld->addChild(CeilingBlocks[i][j]);

			// 计算位置：以 (0,0) 为中心，从 -1.5 到 +1.5 均匀分布
			double x_center = -CeilingSize / 2.0 + (i + 0.5) * cellSize_ceiling;
			double y_center = -CeilingSize / 2.0 + (j + 0.5) * cellSize_ceiling;
			double z_center = 0.6;

			cVector3d blockPos(x_center, y_center, z_center);

			// 创建每个平面块（旋转180°贴合天花板朝下）
			cCreatePlane(CeilingBlocks[i][j], cellSize_ceiling, cellSize_ceiling, blockPos, rot_ceiling);

			// 设置初始颜色和材质
			CeilingBlocks[i][j]->m_material->setBlue();
			CeilingBlocks[i][j]->setUseMaterial(true);

			// 设置触觉属性（如果你需要）
			CeilingBlocks[i][j]->m_material->setStiffness(0.5 * maxStiffness);
			CeilingBlocks[i][j]->m_material->setStaticFriction(0.2);
			CeilingBlocks[i][j]->m_material->setDynamicFriction(0.15);
			CeilingBlocks[i][j]->m_material->setHapticTriangleSides(true, false);
			CeilingBlocks[i][j]->m_material->setHapticTriangleSides(true, true);

			// 启用碰撞检测
			CeilingBlocks[i][j]->createAABBCollisionDetector(toolRadius);
		}
	}


	//////////////////////////////////////////////////////////////////////////
	// BackWall
	//////////////////////////////////////////////////////////////////////////
//	// create a mesh that represents the Ceiling
//	cMesh* BackWall = new cMesh();
//	ODEWorld->addChild(BackWall);
//
//
//	cMatrix3d rot_backwall;
//	rot_backwall.setAxisAngleRotationRad(0, 1, 0, cDegToRad(90));  // 180° rotation about X-axis
//	cCreatePlane(BackWall, 1.8, 3.0, cVector3d(-1.5, 0, -0.3), rot_backwall);
//
//	// create texture property
//	cTexture2dPtr texture_bw = cTexture2d::create();
//	BackWall->setTexture(texture_bw);
//
//	// load texture from file
//	fileload = BackWall->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/brick-color.jpg"));
//	if (!fileload)
//	{
//#if defined(_MSVC)
//		fileload = BackWall->m_texture->loadFromFile("../../../../../bin/resources/images/canvas.jpg");
//#endif
//	}
//	if (!fileload)
//	{
//		cout << "Error - Texture image 'canvas.jpg' failed to load correctly." << endl;
//		close();
//		return (-1);
//	}
//
//	// define some material properties and apply to mesh
//	BackWall->setUseMaterial(false);
//	BackWall->setUseTexture(true);
//	BackWall->m_material->setStiffness(0.5 * maxStiffness);
//	BackWall->m_material->setStaticFriction(0.20);
//	BackWall->m_material->setDynamicFriction(0.15);
//	BackWall->m_material->setHapticTriangleSides(true, false);
//
//	//// setup collision detector
//	BackWall->createAABBCollisionDetector(toolRadius);

	double width_backwall = 3.0;  // X方向宽度
	double height_backwall = 1.8; // Y方向高度
	double cellwidth_backwall = width_backwall / GRID;
	double cellHeight_backwall = height_backwall / GRID;

	cMatrix3d rot_backwall;
	rot_backwall.setAxisAngleRotationRad(0, 1, 0, cDegToRad(90)); // 平面竖直朝右

	for (int i = 0; i < GRID; ++i)
	{
		for (int j = 0; j < GRID; ++j)
		{
			BackWallBlocks[i][j] = new cMesh();
			ODEWorld->addChild(BackWallBlocks[i][j]);

			// 计算位置（左上角开始，从 (-1.5, ..., ...) 开始）
			double x = -1.5;
			double y_center = -width_backwall / 2.0 + (i + 0.5) * cellwidth_backwall;
			double z_center = -0.3 -height_backwall/2.0+(j + 0.5) * cellHeight_backwall;

			cVector3d pos(x, y_center, z_center);

			// 创建面板
			cCreatePlane(BackWallBlocks[i][j], cellHeight_backwall, cellwidth_backwall, pos, rot_backwall);

			// 设置颜色或材质（这里先用颜色）
			BackWallBlocks[i][j]->m_material->setGrayGainsboro();
			BackWallBlocks[i][j]->setUseMaterial(true);
			BackWallBlocks[i][j]->setUseTexture(false); // 禁用纹理，改为纯色显示
			BackWallBlocks[i][j]->setWireMode(false);

			// 设置触觉属性
			BackWallBlocks[i][j]->m_material->setStiffness(0.5 * maxStiffness);
			BackWallBlocks[i][j]->m_material->setStaticFriction(0.2);
			BackWallBlocks[i][j]->m_material->setDynamicFriction(0.15);
			BackWallBlocks[i][j]->m_material->setHapticTriangleSides(true, false);

			// 启用碰撞检测
			BackWallBlocks[i][j]->createAABBCollisionDetector(toolRadius);
		}
	}


	//////////////////////////////////////////////////////////////////////////
	// LeftWall
	//////////////////////////////////////////////////////////////////////////
//	// create a mesh that represents the Ceiling
//	cMesh* LeftWall = new cMesh();
//	ODEWorld->addChild(LeftWall);
//
//
//	cMatrix3d rot_LeftWall;
//	rot_LeftWall.setAxisAngleRotationRad(1, 0, 0, cDegToRad(270));  // 180° rotation about X-axis
//	cCreatePlane(LeftWall, 3.0, 1.8, cVector3d(0, -1.5, -0.3), rot_LeftWall);// cVector3d(front,right,up)
//
//	// create texture property
//	cTexture2dPtr texture_lw = cTexture2d::create();
//	LeftWall->setTexture(texture_lw);
//
//	// load texture from file
//	fileload = LeftWall->m_texture->loadFromFile("../../../../../bin/resources/images/ceiling.jpeg");
//	if (!fileload)
//	{
//#if defined(_MSVC)
//		fileload = LeftWall->m_texture->loadFromFile("../../../../../bin/resources/images/canvas.jpg");
//#endif
//	}
//	if (!fileload)
//	{
//		cout << "Error - Texture image 'canvas.jpg' failed to load correctly." << endl;
//		close();
//		return (-1);
//	}
//
//	// define some material properties and apply to mesh
//	LeftWall->setUseMaterial(false);
//	LeftWall->setUseTexture(true);
//	LeftWall->m_material->setStiffness(0.5 * maxStiffness);
//	LeftWall->m_material->setStaticFriction(0.20);
//	LeftWall->m_material->setDynamicFriction(0.15);
//	LeftWall->m_material->setHapticTriangleSides(true, false);
//	
//	// setup collision detector
//	LeftWall->createAABBCollisionDetector(toolRadius);
//
	//////////////////////////////////////////////////////////////////////////
	// LeftWall
	//////////////////////////////////////////////////////////////////////////

	double width_LeftWall = 3.0;   // 横向宽度（X）
	double height_LeftWall = 1.8;  // 垂直高度（Z）
	double cellWidth_LeftWall = width_LeftWall / GRID_X;
	double cellHeight_LeftWall = height_LeftWall / GRID_Y;

	cMatrix3d rot_LeftWall;
	rot_LeftWall.setAxisAngleRotationRad(1, 0, 0, cDegToRad(270));  // 向右墙贴上

	for (int i = 0; i < GRID_X; ++i)
	{
		for (int j = 0; j < GRID_Y; ++j)
		{
			LeftWallBlocks[i][j] = new cMesh();
			ODEWorld->addChild(LeftWallBlocks[i][j]);

			// 网格中心位置（根据原位置 cVector3d(0, -1.5, -0.3) 拓展）
			double x_center = -width_LeftWall / 2.0 + (i + 0.5) * cellWidth_LeftWall;
			double y = -1.5;  // 靠最左侧
			double z_center = -0.3 -height_LeftWall/2.0 + (j + 0.5) * cellHeight_LeftWall;

			cVector3d pos(x_center, y, z_center);  // front-right-up

			// 创建平面块
			cCreatePlane(LeftWallBlocks[i][j], cellWidth_LeftWall, cellHeight_LeftWall, pos, rot_LeftWall);

			// 使用颜色材质（默认灰色）
			LeftWallBlocks[i][j]->m_material->setGrayGainsboro();
			LeftWallBlocks[i][j]->setUseMaterial(true);
			LeftWallBlocks[i][j]->setUseTexture(false); // 关闭贴图，确保颜色生效
			LeftWallBlocks[i][j]->setWireMode(false);

			// 设置触觉属性
			LeftWallBlocks[i][j]->m_material->setStiffness(0.5 * maxStiffness);
			LeftWallBlocks[i][j]->m_material->setStaticFriction(0.2);
			LeftWallBlocks[i][j]->m_material->setDynamicFriction(0.15);
			LeftWallBlocks[i][j]->m_material->setHapticTriangleSides(true, false);

			// 启用碰撞检测
			LeftWallBlocks[i][j]->createAABBCollisionDetector(toolRadius);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// RightWall
	//////////////////////////////////////////////////////////////////////////
//	// create a mesh that represents the Ceiling
//	cMesh* RightWall = new cMesh();
//	ODEWorld->addChild(RightWall);
//
//
//	cMatrix3d rot_RightWall;
//	rot_RightWall.setAxisAngleRotationRad(1, 0, 0, cDegToRad(90));  // 180° rotation about X-axis
//	cCreatePlane(RightWall, 3.0, 1.8, cVector3d(0, 1.5, -0.3), rot_RightWall);
//
//	// create texture property
//	cTexture2dPtr texture_rw = cTexture2d::create();
//	RightWall->setTexture(texture_rw);
//
//	// load texture from file
//	fileload = RightWall->m_texture->loadFromFile(RESOURCE_PATH("../resources/images/brick-color.jpg"));
//	if (!fileload)
//	{
//#if defined(_MSVC)
//		fileload = RightWall->m_texture->loadFromFile("../../../../../bin/resources/images/canvas.jpg");
//#endif
//	}
//	if (!fileload)
//	{
//		cout << "Error - Texture image 'canvas.jpg' failed to load correctly." << endl;
//		close();
//		return (-1);
//	}
//
//	// define some material properties and apply to mesh
//	RightWall->setUseMaterial(false);
//	RightWall->setUseTexture(true);
//	RightWall->m_material->setStiffness(0.5 * maxStiffness);
//	RightWall->m_material->setStaticFriction(0.20);
//	RightWall->m_material->setDynamicFriction(0.15);
//	RightWall->m_material->setHapticTriangleSides(true, false);
//
//	//// setup collision detector
//	RightWall->createAABBCollisionDetector(toolRadius);
//
	//////////////////////////////////////////////////////////////////////////
	// RightWall
	//////////////////////////////////////////////////////////////////////////


	double width_RightWall = 3.0;   // 水平宽度（Z方向）
	double height_RightWall = 1.8;  // 垂直高度（X方向）
	double cellWidth_RightWall = width_RightWall / GRID_X;
	double cellHeight_RightWall = height_RightWall / GRID_Y;

	cMatrix3d rot_RightWall;
	rot_RightWall.setAxisAngleRotationRad(1, 0, 0, cDegToRad(90));  // 朝左方向

	for (int i = 0; i < GRID_X; ++i)
	{
		for (int j = 0; j < GRID_Y; ++j)
		{
			RightWallBlocks[i][j] = new cMesh();
			ODEWorld->addChild(RightWallBlocks[i][j]);

			// 每格的位置计算
			double x_center = -width_RightWall / 2.0 + (i + 0.5) * cellWidth_RightWall;
			double y = 1.5;  // Y 为固定值（靠右侧）
			double z_center = -0.3  -height_RightWall / 2.0 + (j + 0.5) *cellHeight_RightWall ;

			cVector3d pos(x_center, y, z_center);

			// 创建平面块
			cCreatePlane(RightWallBlocks[i][j], cellWidth_RightWall,cellHeight_RightWall, pos, rot_RightWall);

			// 材质颜色
			RightWallBlocks[i][j]->m_material->setGrayGainsboro();
			RightWallBlocks[i][j]->setUseMaterial(true);
			RightWallBlocks[i][j]->setUseTexture(false); // 禁用纹理以便看颜色

			// 触觉参数
			RightWallBlocks[i][j]->m_material->setStiffness(0.5 * maxStiffness);
			RightWallBlocks[i][j]->m_material->setStaticFriction(0.20);
			RightWallBlocks[i][j]->m_material->setDynamicFriction(0.15);
			RightWallBlocks[i][j]->m_material->setHapticTriangleSides(true, false);

			// 开启碰撞检测
			RightWallBlocks[i][j]->createAABBCollisionDetector(toolRadius);
		}
	}


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
void mouseClick(int button, int state, int x, int y)
{
	mouseX = x;
	mouseY = y;
}

//------------------------------------------------------------------------------

void mouseMove(int x, int y)
{
	// compute mouse motion
	int dx = x - mouseX;
	int dy = y - mouseY;
	mouseX = x;
	mouseY = y;

	// compute new camera angles
	double azimuthDeg = camera->getSphericalAzimuthDeg() + (0.5 * dy);
	double polarDeg = camera->getSphericalPolarDeg() + (-0.5 * dx);

	// assign new angles
	camera->setSphericalAzimuthDeg(azimuthDeg);
	camera->setSphericalPolarDeg(polarDeg);

	// line up tool with camera
	tool->setLocalRot(camera->getLocalRot());
}

//------------------------------------------------------------------------------

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

enum cMode
{
	IDLE,
	SELECTION
};

void updateHaptics(void)
{
	cMode state = IDLE;
	cGenericObject* selectedObject = NULL;
	cTransform tool_T_object;

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

		/////////////////////////////////////////////////////////////////////////
		// MANIPULATION
		/////////////////////////////////////////////////////////////////////////

		// compute transformation from world to tool (haptic device)
		cTransform world_T_tool = tool->getDeviceGlobalTransform();

		// get status of user switch
		bool button = tool->getUserSwitch(0);
		// Declare ahead, fill in only if safe
		cVector3d posA;
		cVector3d posB;

		// Only compute positions if tool and graspObject are safe
		if (graspActive && graspObject != nullptr &&
			tool->getNumHapticPoints() > 0 &&
			tool->getHapticPoint(0) != nullptr)
		{
			posA = graspObject->getLocalTransform() * graspPos;
			posB = tool->getHapticPoint(0)->getGlobalPosGoal();
		}


		//
		if (graspActive)
		{
			if (tool->getUserSwitch(0))
			{
				if (graspObject != nullptr &&
					tool->getNumHapticPoints() > 0 &&
					tool->getHapticPoint(0) != nullptr)
				{
					cVector3d posA = graspObject->getLocalTransform() * graspPos;
					cVector3d posB = tool->getHapticPoint(0)->getGlobalPosGoal();

					if (graspLine != nullptr)
					{
						graspLine->m_pointA = posA;
						graspLine->m_pointB = posB;
						graspLine->setShowEnabled(true);
					}

					// Apply spring force
					cVector3d force = 5.0 * (posB - posA);
					graspObject->addExternalForceAtPoint(force, posA);
					tool->addDeviceGlobalForce(-force);
				}
			}
			else
			{
				if (graspLine != nullptr)
				{
					graspLine->setShowEnabled(false);
				}
				graspActive = false;
				graspObject = nullptr;
			}
		}
		else
		{
			// Detect new object to grasp
			cHapticPoint* interactionPoint = tool->getHapticPoint(0);
			if (interactionPoint != nullptr && interactionPoint->getNumCollisionEvents() > 0)
			{
				cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);
				cGenericObject* obj = collisionEvent->m_object->getOwner()->getOwner();
				cODEGenericBody* body = dynamic_cast<cODEGenericBody*>(obj);

				if (body != nullptr)
				{
					body->addExternalForceAtPoint(
						-0.3 * interactionPoint->getLastComputedForce(),
						collisionEvent->m_globalPos);

					if (tool->getUserSwitch(0))
					{
						graspObject = body;
						graspPos = collisionEvent->m_localPos;
						graspActive = true;
					}
				}
			}
		}



		////
		//// STATE 1:
		//// Idle mode - user presses the user switch
		////
		//if ((state == IDLE) && (button == true))
		//{
		//	// check if at least one contact has occurred
		//	if (((cToolCursor*)tool)->m_hapticPoint->getNumCollisionEvents() > 0)
		//	{
		//		// get contact event
		//		cCollisionEvent* collisionEvent = ((cToolCursor*)tool)->m_hapticPoint->getCollisionEvent(0);
		//		// get object from contact event
		//		selectedObject = collisionEvent->m_object;
		//	}
		//	else
		//	{
		//		selectedObject = object0 ? object0 : (object1 ? object1 : object2);
		//	}
		//	// get transformation from object
		//	cTransform world_T_object = selectedObject->getGlobalTransform();
		//	// compute inverse transformation from contact point to object 
		//	cTransform tool_T_world = world_T_tool;
		//	tool_T_world.invert();
		//	// store current transformation tool
		//	tool_T_object = tool_T_world * world_T_object;
		//	// update state
		//	state = SELECTION;
		//}


		////
		//// STATE 2:
		//// Selection mode - operator maintains user switch enabled and moves object
		////
		//else if ((state == SELECTION) && (button == true))
		//{
		//	// compute new transformation of object in global coordinates
		//	cTransform world_T_object = world_T_tool * tool_T_object;

		//	// compute new transformation of object in local coordinates
		//	cTransform parent_T_world = selectedObject->getParent()->getLocalTransform();
		//	parent_T_world.invert();
		//	cTransform parent_T_object = parent_T_world * world_T_object;

		//	// assign new local transformation to object
		//	selectedObject->setLocalTransform(parent_T_object);

		//	// set zero forces when manipulating objects
		//	tool->setDeviceGlobalForce(0.0, 0.0, 0.0);

		//	tool->initialize();
		//}

		////
		//// STATE 3:
		//// Finalize Selection mode - operator releases user switch.
		////
		//else
		//{
		//	state = IDLE;
		//}

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
				if (object != NULL){
					for (int m = 0; m < GRID; ++m)
					{
						for (int n = 0; n < GRID; ++n)
						{
							if (object == CeilingBlocks[m][n])
							{
								// 🔥 highlight the touched block
								CeilingBlocks[m][n]->m_material->setRed();
							}
							else if (object == GroundBlocks[m][n])
							{
								// 🔥 highlight the touched block
								GroundBlocks[m][n]->m_material->setRed();
							}
							else if(object == BackWallBlocks[m][n])
							{
								// 🔥 highlight the touched block
								BackWallBlocks[m][n]->m_material->setRed();
							}
							else if(object == LeftWallBlocks[m][n])
							{
								// 🔥 highlight the touched block
								LeftWallBlocks[m][n]->m_material->setRed();
							}
							else if (object == RightWallBlocks[m][n])
							{
								// 🔥 highlight the touched block
								RightWallBlocks[m][n]->m_material->setRed();
							}

						}
					}
				}
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

