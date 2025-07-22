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
shared_ptr<cGenericHapticDevice> hapticDevice[2];

// a virtual tool representing the haptic device in the scene
cGenericTool* tool[2];

// Grabbing logic

bool graspActive[2];
cODEGenericBody* graspObject[2];
cVector3d graspPos[2];
cShapeLine* graspLine[2];

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

int score;

double maxStiffness[2];
double workspaceScaleFactor[2];

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
		3,//2.3,  // radius (distance from center)X
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
	double toolRadius = 0.04;
	handler = new cHapticDeviceHandler();
	int numHapticDevices = handler->getNumDevices();
	numHapticDevices = 2;  // clamp to 2

	for (int i = 0; i < numHapticDevices; i++)
	{
		// 获取设备句柄
		handler->getDevice(hapticDevice[i], i);

		// 创建 tool（可根据设备类型切换成 Gripper 或 Cursor）
		tool[i] = new cToolCursor(world);
		tool[i]->setHapticDevice(hapticDevice[i]);
		tool[i]->enableDynamicObjects(true);
		tool[i]->setWorkspaceRadius(1.5);
		tool[i]->setRadius(0.04, 0.04);
		tool[i]->setShowContactPoints(true, false);
		hapticDevice[i]->setEnableGripperUserSwitch(true);

		// 设置外观和方向
		((cToolCursor*)tool[i])->m_hapticPoint->m_sphereProxy->m_material->setWhite();
		tool[i]->setLocalRot(camera->getLocalRot());
		tool[i]->setWaitForSmallForce(true);

		// 启动 tool
		tool[i]->start();

		// 加入场景
		world->addChild(tool[i]);
	}


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

    // create an ODE world to simulate dynamic bodies
    ODEWorld = new cODEWorld(world);

    // add ODE world as a node inside world
    world->addChild(ODEWorld);

    // set some gravity
    ODEWorld->setGravity(cVector3d(0.00, 0.00,-59.81));

    // define damping properties
    ODEWorld->setAngularDamping(0.00002);
    ODEWorld->setLinearDamping(0.00002);

	for (int i = 0; i < numHapticDevices; i++) {
		// read the scale factor between the physical workspace of the haptic
		// device and the virtual workspace defined for the tool
		cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();
		workspaceScaleFactor[i] = tool[i]->getWorkspaceScaleFactor();
		maxStiffness[i] = info.m_maxLinearStiffness / workspaceScaleFactor[i];

		graspActive[i] = false;
		graspObject[i] = nullptr;
		graspLine[i] = new cShapeLine();
		graspLine[i]->m_colorPointA.setBlue();
		graspLine[i]->m_colorPointB.setBlue();
		graspLine[i]->setShowEnabled(false);
		world->addChild(graspLine[i]);
	}


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
    double size = 0.20;
    cCreateBox(object0, size, size, size);
    object0->createAABBCollisionDetector(toolRadius);

    cCreateBox(object1, size, size, size);
    object1->createAABBCollisionDetector(toolRadius);

    cCreateBox(object2, size, size, size);
    object2->createAABBCollisionDetector(toolRadius);
      
    // define some material properties for each cube
    cMaterial mat0, mat1, mat2;
    mat0.setRedIndian();
    mat0.setStiffness(0.2 * maxStiffness[0]);
    mat0.setDynamicFriction(0.6);
    mat0.setStaticFriction(0.6);
    object0->setMaterial(mat0);

    mat1.setBlueRoyal();
	mat1.setStiffness(0.2 * maxStiffness[0]);
    mat1.setDynamicFriction(0.6);
    mat1.setStaticFriction(0.6);
    object1->setMaterial(mat1);

    mat2.setGreenDarkSea();
	mat2.setStiffness(0.2 * maxStiffness[0]);
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
    ODEBody0->setMass(0.03);
    ODEBody1->setMass(0.02);
    ODEBody2->setMass(0.01);

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
			GroundBlocks[i][j]->m_material->setStiffness(0.2 * maxStiffness[0]);
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
			CeilingBlocks[i][j]->m_material->setStiffness(0.2 * maxStiffness[0]);
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
			BackWallBlocks[i][j]->m_material->setStiffness(0.2 * maxStiffness[0]);
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
			LeftWallBlocks[i][j]->m_material->setStiffness(0.2 * maxStiffness[0]);
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
			RightWallBlocks[i][j]->m_material->setStiffness(0.2 * maxStiffness[0]);
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
            ODEWorld->setGravity(cVector3d(0.0, 0.0,-59.81));
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

	//// line up tool with camera
	//tool->setLocalRot(camera->getLocalRot());
	for (int i = 0; i < 2; i++) {
		tool[i]->setLocalRot(camera->getLocalRot());
	}

}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    //tool->stop();
	for (int i = 0; i < 2; i++) {
		tool[i]->stop();
	}

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
int calculateScore()
{
	int score = 0;
	for (int m = 0; m < GRID; ++m)
	{
		for (int n = 0; n < GRID; ++n)
		{
			cColorf c;

			c = CeilingBlocks[m][n]->m_material->m_diffuse;
			if (c.getR() == 1.0 && c.getG() == 0.0 && c.getB() == 0.0) score++;

			c = GroundBlocks[m][n]->m_material->m_diffuse;
			if (c.getR() == 1.0 && c.getG() == 0.0 && c.getB() == 0.0) score++;

			c = BackWallBlocks[m][n]->m_material->m_diffuse;
			if (c.getR() == 1.0 && c.getG() == 0.0 && c.getB() == 0.0) score++;

			c = LeftWallBlocks[m][n]->m_material->m_diffuse;
			if (c.getR() == 1.0 && c.getG() == 0.0 && c.getB() == 0.0) score++;

			c = RightWallBlocks[m][n]->m_material->m_diffuse;
			if (c.getR() == 1.0 && c.getG() == 0.0 && c.getB() == 0.0) score++;
		}
	}
	return score;
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

		static int lastScore = -1;  // 初始值设为非法分数

		int currentScore = calculateScore();

		if (currentScore != lastScore)
		{
			printf("Score: %d\n", currentScore);
			lastScore = currentScore;
		}


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

		for (int i = 0; i < 2; i++) {
			tool[i]->updateFromDevice();
			tool[i]->computeInteractionForces();  // 保留 CHAI3D 内部力，如墙面、proxy 碰撞力

			//------------------------------------------------------------
			// Step 1: 设备线性阻尼（防止剧烈震动）
			//------------------------------------------------------------
			cVector3d velocity;
			hapticDevice[i]->getLinearVelocity(velocity);
			cVector3d dampingForce = -0.8 * velocity;
			tool[i]->addDeviceGlobalForce(dampingForce);

			//------------------------------------------------------------
			// Step 3: 应用到设备（必须）
			//------------------------------------------------------------
			tool[i]->applyToDevice();
		}



		/////////////////////////////////////////////////////////////////////////
		// MANIPULATION
		/////////////////////////////////////////////////////////////////////////
		for (int i = 0; i < 2; i++)
		{
			// Get current tool and grasp state
			cGenericTool* t = tool[i];

			// Compute transformation from world to tool
			cTransform world_T_tool = t->getDeviceGlobalTransform();

			// Get button state
			bool button = t->getUserSwitch(0);

			// Compute positions if safe
			cVector3d posA, posB;
			if (graspActive[i] && graspObject[i] != nullptr &&
				t->getNumHapticPoints() > 0 &&
				t->getHapticPoint(0) != nullptr)
			{
				posA = graspObject[i]->getLocalTransform() * graspPos[i];
				posB = t->getHapticPoint(0)->getGlobalPosGoal();
			}

			// Grasp active
			if (graspActive[i])
			{
				if (button)
				{
					if (graspObject[i] != nullptr &&
						t->getNumHapticPoints() > 0 &&
						t->getHapticPoint(0) != nullptr)
					{
						posA = graspObject[i]->getLocalTransform() * graspPos[i];
						posB = t->getHapticPoint(0)->getGlobalPosGoal();

						cVector3d graspForce = 3.0 * (posB - posA);

						graspObject[i]->addExternalForceAtPoint(graspForce, posA);
						tool[i]->addDeviceGlobalForce(-graspForce);

						if (graspLine[i] != nullptr)
						{
							graspLine[i]->m_pointA = posA;
							graspLine[i]->m_pointB = posB;
							graspLine[i]->setShowEnabled(true);
						}
					}
				}
				else
				{
					if (graspLine[i]) graspLine[i]->setShowEnabled(false);
					graspActive[i] = false;
					graspObject[i] = nullptr;
				}
			}
			else
			{
				// Attempt to grasp new object
				cHapticPoint* interactionPoint = t->getHapticPoint(0);
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

						if (button)
						{
							graspObject[i] = body;
							graspPos[i] = collisionEvent->m_localPos;
							graspActive[i] = true;
						}
					}
				}
			}

			t->applyToDevice();
		}


for (int p = 0; p < 2; p++)  // go through every player
{
	int numInteractionPoints = tool[p]->getNumHapticPoints();
	for (int i = 0; i < numInteractionPoints; i++)
	{
		cHapticPoint* interactionPoint = tool[p]->getHapticPoint(i);

		int numContacts = interactionPoint->getNumCollisionEvents();
		for (int j = 0; j < numContacts; j++)
		{
			cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(j);
			cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

			// 判断是否为 ODE 对象（可施加力反馈）
			cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

			if (object != NULL)
			{
				for (int m = 0; m < GRID; ++m)
				{
					for (int n = 0; n < GRID; ++n)
					{

						if (object == CeilingBlocks[m][n])
						{
							if (p == 0)
								CeilingBlocks[m][n]->m_material->setRed();     // 玩家1触碰 → 变红
							else if (p == 1)
								CeilingBlocks[m][n]->m_material->setBlue();    // 玩家2触碰 → 恢复
						}
						else if (object == GroundBlocks[m][n])
						{
							if (p == 0)
								GroundBlocks[m][n]->m_material->setRed();
							else if (p == 1)
								GroundBlocks[m][n]->m_material->setBlue();
						}
						else if (object == BackWallBlocks[m][n])
						{
							if (p == 0)
								BackWallBlocks[m][n]->m_material->setRed();
							else if (p == 1)
								BackWallBlocks[m][n]->m_material->setGrayGainsboro();
						}
						else if (object == LeftWallBlocks[m][n])
						{
							if (p == 0)
								LeftWallBlocks[m][n]->m_material->setRed();
							else if (p == 1)
								LeftWallBlocks[m][n]->m_material->setGrayGainsboro();
						}
						else if (object == RightWallBlocks[m][n])
						{
							if (p == 0)
								RightWallBlocks[m][n]->m_material->setRed();
							else if (p == 1)
								RightWallBlocks[m][n]->m_material->setGrayGainsboro();
						}
					}
				}
			}


			// ✅ 对 ODE 对象施加力反馈
			/*if (ODEobject != NULL)
			{
				ODEobject->addExternalForceAtPoint(
					-0.3 * interactionPoint->getLastComputedForce(),
					collisionEvent->m_globalPos
					);
			}*/
			// ✅ 只在未抓取状态下轻推 cube，避免和 grasp 冲突
			if (ODEobject != nullptr && !graspActive[p]) {
				ODEobject->addExternalForceAtPoint(
					-0.3 * interactionPoint->getLastComputedForce(),
					collisionEvent->m_globalPos
					);
			}

		}
	}
}

        // update simulation
        ODEWorld->updateDynamics(timeInterval);
    }

    // exit haptics thread
    simulationFinished = true;
}

