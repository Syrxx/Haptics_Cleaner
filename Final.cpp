
//add ult that make spheres fast
#include "chai3d.h"
#include "CODE.h"

#include <algorithm>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <string>

#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif

using namespace chai3d;
using namespace std;

//------------------------------------------------------------------------------
// Stereo / window flags
//------------------------------------------------------------------------------
cStereoMode stereoMode = C_STEREO_DISABLED;
bool fullscreen = false;
bool mirroredDisplay = false;

//------------------------------------------------------------------------------
// CHAI3D core pointers
//------------------------------------------------------------------------------
cWorld*       world = nullptr;
cCamera*      camera = nullptr;
cSpotLight*   light = nullptr;

// Haptics
cHapticDeviceHandler* handler = nullptr;
shared_ptr<cGenericHapticDevice> hapticDevice[2];
cToolCursor* tool[2] = { nullptr, nullptr };
double workspaceScaleFactor[2] = { 1.0, 1.0 };
double maxStiffness[2] = { 0.0, 0.0 };

//------------------------------------------------------------------------------
// Paintable walls + ground
//------------------------------------------------------------------------------
struct PaintWall
{
	cMesh* mesh = nullptr;              // visual mesh
	cTexture2dPtr texture;              // texture we paint into
	cImagePtr originalImage;            // backup for clearing
	double brushScale = 0.8;
	double pixelToWorldScale = 1.0;
};

// Timer
cPrecisionClock gameClock;
bool gameStarted = false;
bool gameEnded = false;
double gameDuration = 30; // seconds        -----------------------------------------------------------------
bool countdownRunning = false;
double countdownStartTime = 0.0;
double countdownDuration = 3.0; // 3 seconds
cPrecisionClock brushClock;


// ULTIMATE STATE
bool ultReady[2] = { false, false };
bool ultUsed[2] = { false, false };
bool ultActive[2] = { false, false };
cPrecisionClock ultTimer[2];
double ultDuration[2] = { 10.0, 10.0 }; // both 3 seconds
cLabel* labelUltStatus[2]; // 

bool floatingMode = false;
bool canErase = true;
bool mirrorActivated = false;


int currentGame = 0;


cFont* font = NEW_CFONTCALIBRI20();

// UI Labels
cLabel* labelTimer;
cLabel* labelStartMessage;
cLabel* labelWinnerMessage;
cLabel* labelScore = nullptr;






PaintWall groundWall;   // not paintable
PaintWall backWall;     // paintable
PaintWall leftWall;     // paintable
PaintWall rightWall;    // paintable

cMesh* frontBlockingWall = nullptr;   // invisible
cMesh* ceilingBlockingWall = nullptr;   // invisible
// Camera spherical coordinates
double camAzimuthDeg = 0.0;
double camPolarDeg = 90.0;
double camRadius = 3.0;
//------------------------------------------------------------------------------
// ODE world & dynamic objects
//------------------------------------------------------------------------------
cODEWorld* ODEWorld = nullptr;
cODEGenericBody* ODECube[20] = { nullptr, nullptr, nullptr };
//HanOi tower
cODEGenericBody* hanoiRings[5];
cMesh* hanoiRingMeshes[5];
cMesh* hanoiBase;
cMesh* hanoiRods[3];

//---------------------------------------------------------------------------------to do

// Magnetic Spheres
const int NUM_SPHERES_A = 5;
const int NUM_SPHERES_B = 5;
const double SPHERE_RADIUS = 0.1;
cODEGenericBody* spheres_A[NUM_SPHERES_A];
cODEGenericBody* spheres_B[NUM_SPHERES_B];
cVector3d sphereVel_A[NUM_SPHERES_A];
cVector3d sphereVel_B[NUM_SPHERES_B];

// Room
const double FLOOR_Z = -1.0;


// For grasping logic
const int NUM_TOOLS = 2;

bool isGrasping[NUM_TOOLS] = { false, false };
cODEGenericBody* graspObject[NUM_TOOLS] = { nullptr, nullptr };
cVector3d graspOffset[NUM_TOOLS];

cShapeLine* graspLine[NUM_TOOLS] = { nullptr, nullptr };
double graspSpringStiffness = 10.0;


//------------------------------------------------------------------------------
// Widgets
//------------------------------------------------------------------------------
cLabel* labelHapticRate = nullptr;
//grabbing
// --- For grabbing state ---
enum cMode { IDLE, SELECTION };
cMode state[2] = { IDLE, IDLE };
cGenericObject* selectedObject[2] = { nullptr, nullptr };
cTransform tool_T_object[2];

//------------------------------------------------------------------------------
// Misc state
//------------------------------------------------------------------------------
bool simulationRunning = false;
bool simulationFinished = true;
bool countdownFinished = false;


cFrequencyCounter frequencyCounter;

// Screen
int screenW, screenH, windowW, windowH, windowPosX, windowPosY;
int mouseX = 0, mouseY = 0;

// Paths --------------------------------------------------------------------
string resourceRoot;
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

static const string PATH_CANVAS = R"(D:\chai3d-3.1.1\bin\resources\images\canvas.jpg)";
static const string PATH_STONE = R"(D:\chai3d-3.1.1\bin\resources\images\brick-color.png)";
static const string PATH_FOAM = R"(D:\chai3d-3.1.1\bin\resources\images\whitefoam.jpg)";
static const string PATH_FLOOR = R"(D:\chai3d-3.1.1\bin\resources\images\lines.png)";

static const string PATH_SPHERE_1 = R"(D:\chai3d-3.1.1\bin\resources\images\spheremap-1.jpg)";
static const string PATH_SPHERE_2 = R"(D:\chai3d-3.1.1\bin\resources\images\spheremap-6.jpg)";

// Painting constants
const double K_INK = 30;
const double K_SIZE = 10.0;
const int    BRUSH_SIZE = 25;

cColorb getDynamicToolColor(int toolIndex)
{
	double t = brushClock.getCurrentTimeSeconds();
	double hue = fmod((t * 60.0) + toolIndex * 180.0, 360.0); // Offset hues for each player

	double s = 1.0;
	double v = 1.0;
	double c = v * s;
	double x = c * (1 - fabs(fmod(hue / 60.0, 2) - 1));
	double m = v - c;

	double r = 0, g = 0, b = 0;

	if (hue < 60)      { r = c; g = x; b = 0; }
	else if (hue < 120){ r = x; g = c; b = 0; }
	else if (hue < 180){ r = 0; g = c; b = x; }
	else if (hue < 240){ r = 0; g = x; b = c; }
	else if (hue < 300){ r = x; g = 0; b = c; }
	else               { r = c; g = 0; b = x; }

	return cColorb((r + m) * 255, (g + m) * 255, (b + m) * 255);
}


// Forward declarations
void resizeWindow(int w, int h);
void keySelect(unsigned char key, int x, int y);
void mouseClick(int button, int state, int x, int y);
void mouseMove(int x, int y);
void updateGraphics(void);
void graphicsTimer(int data);
void closeApp(void);
void updateHaptics(void);
void mouseButton(int button, int state, int x, int y);

// Helpers

static cMesh* createPlane(double sx, double sy, const cVector3d& pos, const cMatrix3d& rot,
	const cColorf& color, bool textured = false,
	cTexture2dPtr tex = nullptr, bool hapticSides = true,
	double stiff = 0.9, double sFric = 0.2, double dFric = 0.4,
	double texLevel = 20)
{
	cMesh* m = new cMesh();
	cCreatePlane(m, sx, sy, pos, rot);
	m->setUseMaterial(true);
	m->setUseMaterial(true);
	m->m_material = cMaterial::create();
	m->m_material->setColor(cColorf(1.0, 1.0, 1.0));

	m->m_material->setStiffness(stiff);
	m->m_material->setStaticFriction(sFric);
	m->m_material->setDynamicFriction(dFric);
	m->m_material->setTextureLevel(texLevel);
	m->m_material->setViscosity(0.15);
	m->m_material->setHapticTriangleSides(hapticSides, false);

	if (textured && tex)
	{
		m->setUseTexture(true);
		m->setTexture(tex);
	}

	m->createAABBCollisionDetector(0.05);
	return m;
}


// for scoring
int leftWallTotalPixels = 0;
int rightWallTotalPixels = 0;
int backWallTotalPixels = 0;
// Compute how much of the wall has been painted 
double computePaintCoverage(const PaintWall& w, int total)
{
	if (!w.texture || !w.originalImage || total == 0) return 0.0;

	int painted = 0;
	cColorb current, original;

	int maxX = std::min(w.originalImage->getWidth(), w.texture->m_image->getWidth());
	int maxY = std::min(w.originalImage->getHeight(), w.texture->m_image->getHeight());

	for (int x = 0; x < maxX; ++x)
	{
		for (int y = 0; y < maxY; ++y)
		{
			w.originalImage->getPixelColor(x, y, original);
			w.texture->m_image->getPixelColor(x, y, current);

			if (original.getA() > 0 &&
				(current.getR() != original.getR() ||
				current.getG() != original.getG() ||
				current.getB() != original.getB()))
			{
				painted++;
			}
		}
	}

	return 100.0 * painted / total;
}
double computeUnpaintedCoverage(const PaintWall& w, int total)
{
	if (!w.texture || !w.originalImage || total == 0) return 0.0;

	int untouched = 0;
	cColorb current, original;

	int maxX = std::min(w.originalImage->getWidth(), w.texture->m_image->getWidth());
	int maxY = std::min(w.originalImage->getHeight(), w.texture->m_image->getHeight());

	for (int x = 0; x < maxX; ++x)
	{
		for (int y = 0; y < maxY; ++y)
		{
			w.originalImage->getPixelColor(x, y, original);
			w.texture->m_image->getPixelColor(x, y, current);

			if (original.getA() > 0 &&
				current.getR() == original.getR() &&
				current.getG() == original.getG() &&
				current.getB() == original.getB())
			{
				untouched++;
			}
		}
	}

	return 100.0 * untouched / total;
}
inline std::string cExtractFileFromPath(const std::string& path)
{
	size_t pos = path.find_last_of("/\\");
	return (pos != std::string::npos) ? path.substr(pos + 1) : path;
}
inline bool loadTextureAbsolute(cTexture2dPtr& tex, const string& absPath)
{
	bool ok = tex->loadFromFile(absPath);
#if defined(_MSVC)
	if (!ok) ok = tex->loadFromFile("./././bin/resources/images/" + cExtractFileFromPath(absPath));
#endif
	return ok;
}

void clearWalls()
{
	backWall.originalImage->copyTo(backWall.texture->m_image);
	leftWall.originalImage->copyTo(leftWall.texture->m_image);
	rightWall.originalImage->copyTo(rightWall.texture->m_image);
	backWall.texture->markForUpdate();
	leftWall.texture->markForUpdate();
	rightWall.texture->markForUpdate();
}

void setupPaintWall(PaintWall& w, const string& absImagePath,
	double kStiff, double kSfric, double kDfric, double texLevel)
{
	w.texture = cTexture2d::create();
	if (!loadTextureAbsolute(w.texture, absImagePath))
		cout << "Error - failed to load: " << absImagePath << endl;

	w.mesh->setUseTexture(true);
	w.mesh->setTexture(w.texture);

	w.mesh->m_material->setStiffness(kStiff);
	w.mesh->m_material->setStaticFriction(kSfric);
	w.mesh->m_material->setDynamicFriction(kDfric);
	w.mesh->m_material->setTextureLevel(texLevel);
	w.mesh->m_material->setHapticTriangleSides(true, false);

	w.texture->m_image->convert(GL_RGBA);
	w.originalImage = w.texture->m_image->copy();

}

void paintPixels(PaintWall& w, int px, int py, const cColorb& color, double force, double dt)
{
	if (!w.texture || !w.texture->m_image) return;

	double size = cClamp((K_SIZE * force), 0.0, (double)BRUSH_SIZE);
	for (int x = -BRUSH_SIZE; x<BRUSH_SIZE; ++x)
	{
		for (int y = -BRUSH_SIZE; y<BRUSH_SIZE; ++y)
		{
			double dist = sqrt((double)(x*x + y*y));
			if (dist <= size)
			{
				cColorb oldCol, newCol;
				w.texture->m_image->getPixelColor(px + x, py + y, oldCol);
				double factor = cClamp(K_INK * dt * cClamp(force, 0.0, 10.0) * cClamp(1.0 - dist / size, 0.0, 1.0), 0.0, 1.0);
				newCol.setR((1.0 - factor) * oldCol.getR() + factor * color.getR());
				newCol.setG((1.0 - factor) * oldCol.getG() + factor * color.getG());
				newCol.setB((1.0 - factor) * oldCol.getB() + factor * color.getB());
				w.texture->m_image->setPixelColor(px + x, py + y, newCol);
			}
		}
	}
	w.texture->markForUpdate();
}

void erasePixels(PaintWall& w, int px, int py, double force, double dt)
{
	if (!w.texture || !w.texture->m_image || !w.originalImage) return;

	double size = cClamp((K_SIZE * force), 0.0, (double)BRUSH_SIZE);
	for (int x = -BRUSH_SIZE; x < BRUSH_SIZE; ++x)
	{
		for (int y = -BRUSH_SIZE; y < BRUSH_SIZE; ++y)
		{
			double dist = sqrt((double)(x * x + y * y));
			if (dist <= size)
			{
				cColorb restoreCol;
				w.originalImage->getPixelColor(px + x, py + y, restoreCol);
				w.texture->m_image->setPixelColor(px + x, py + y, restoreCol);
			}
		}
	}
	w.texture->markForUpdate();
}



//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "Demo: Haptic Art Room" << endl;
	cout << "By Syrine & Yuedan." << endl;
	cout << "-----------------------------------" << endl << endl;

	cout << "Keyboard Options:" << endl;
	cout << "[SPACE] - Start game" << endl;
	cout << "[r]     - Restart game" << endl;
	//cout << "[1]     - Painting mode" << endl;
	//cout << "[2]     - Sorting mode (TODO)" << endl;
	cout << "[f]     - Toggle fullscreen" << endl;
	cout << "[m]     - Mirror display" << endl;
	cout << "[x]     - Exit application" << endl << endl;

	cout << "Haptic Device Buttons:" << endl;
	cout << "Big Button   - Grab/Throw objects" << endl;
	//cout << "Side Button  - Activate ultimate (when ready)" << endl << endl;


	resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);
	brushClock.start(true);

	glutInit(&argc, argv);
	screenW = glutGet(GLUT_SCREEN_WIDTH);
	screenH = glutGet(GLUT_SCREEN_HEIGHT);
	windowW = 0.8 * screenH;
	windowH = 0.5 * screenH;
	windowPosY = (screenH - windowH) / 2;
	windowPosX = windowPosY;
	glutInitWindowPosition(windowPosX, windowPosY);
	glutInitWindowSize(windowW, windowH);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(argv[0]);
#ifdef GLEW_VERSION
	glewInit();
#endif
	glutDisplayFunc(updateGraphics);
	glutKeyboardFunc(keySelect);
	glutMouseFunc(mouseClick);
	glutMouseFunc(mouseButton);
	glutMotionFunc(mouseMove);
	glutReshapeFunc(resizeWindow);
	glutSetWindowTitle("CHAI3D Room Paint");
	if (fullscreen) glutFullScreen();

	world = new cWorld();
	world->m_backgroundColor.setWhite();
	camera = new cCamera(world);
	world->addChild(camera);
	// Initial camera spherical setup
	// Initial camera spherical setup
	camAzimuthDeg = 0.0;
	camPolarDeg = 90.0;
	camRadius = 4.0;

	// Apply camera setup manually (same as above)
	double azimuthRad = cDegToRad(camAzimuthDeg);
	double polarRad = cDegToRad(camPolarDeg);
	double x = camRadius * sin(polarRad) * cos(azimuthRad);
	double y = camRadius * sin(polarRad) * sin(azimuthRad);
	double z = camRadius * cos(polarRad);
	camera->setSphericalDeg(3.5, 5.0, 0.0);   // radius, polar, azimuth



	// Main spotlight - top center
	light = new cSpotLight(world);
	world->addChild(light);
	light->setEnabled(true);
	light->setLocalPos(0.0, 0.0, 2.0);
	light->setDir(0.0, 0.0, -1.0);
	light->setCutOffAngleDeg(60);
	light->setSpotExponent(2.0);
	light->setShadowMapEnabled(true);
	light->m_shadowMap->setQualityMedium();
	light->m_ambient.set(0.3, 0.3, 0.3);      // soft ambient glow
	light->m_diffuse.set(1.0, 0.95, 0.8);     // warm lighting
	light->m_specular.set(1.0, 1.0, 1.0);     // highlights

	// Secondary fill light - soft bluish tint from side
	cSpotLight* light2 = new cSpotLight(world);
	world->addChild(light2);
	light2->setEnabled(true);
	light2->setLocalPos(-1.5, -1.5, 1.0);
	light2->setDir(1.0, 1.0, -0.5);
	light2->setCutOffAngleDeg(90);
	light2->setSpotExponent(1.5);
	light2->m_ambient.set(0.1, 0.1, 0.1);     // gentle shadow fill
	light2->m_diffuse.set(0.5, 0.5, 0.7);     // cool light tone
	light2->m_specular.set(0.6, 0.6, 0.6);    // soft highlights
	cSpotLight* light3 = new cSpotLight(world);
	world->addChild(light3);
	light3->setEnabled(true);
	light3->setLocalPos(0.0, 2.0, 1.5);  // behind camera
	light3->setDir(0.0, -1.0, -0.3);
	light3->setCutOffAngleDeg(180);
	light3->setSpotExponent(0.2);
	light3->m_ambient.set(0.2, 0.2, 0.2);
	light3->m_diffuse.set(0.3, 0.3, 0.4);
	light3->m_specular.set(0.1, 0.1, 0.1);
	// Gentle ambient fill to avoid harsh shadows
	cDirectionalLight* ambientLight = new cDirectionalLight(world);
	world->addChild(ambientLight);
	ambientLight->setEnabled(true);
	ambientLight->setDir(0.0, 0.0, -1.0);  // Direction doesn't matter much
	ambientLight->m_ambient.set(0.3, 0.3, 0.3);  // soft overall light
	ambientLight->m_diffuse.set(0.2, 0.2, 0.2);
	ambientLight->m_specular.set(0.1, 0.1, 0.1);

	labelTimer = new cLabel(font);
	labelTimer->setText("Time: " + cStr(gameDuration, 1) + "s");
	labelTimer->m_fontColor.setRed();
	labelTimer->setFontScale(2.5);

	//ult logic
	labelUltStatus[0] = new cLabel(font);
	labelUltStatus[1] = new cLabel(font);
	labelUltStatus[0]->m_fontColor.setBlack();
	labelUltStatus[1]->m_fontColor.setBlack();
	labelUltStatus[0]->setFontScale(2.0);
	labelUltStatus[1]->setFontScale(2.0);

	// get the position of the timer
	int ultY = 815;

	//middle
	int label0_X = (windowW - labelUltStatus[0]->getWidth()) / 2;
	int label1_X = (windowW - labelUltStatus[1]->getWidth()) / 2;

	
	labelUltStatus[0]->setLocalPos(label0_X, ultY);
	labelUltStatus[1]->setLocalPos(label1_X, ultY - 40);  // 第二条再往下 40px


	labelUltStatus[0]->setEnabled(false);
	labelUltStatus[1]->setEnabled(false);
	camera->m_frontLayer->addChild(labelUltStatus[0]);
	camera->m_frontLayer->addChild(labelUltStatus[1]);


	///score label
	camera->m_frontLayer->addChild(labelTimer);
	labelScore = new cLabel(font);
	labelScore->m_fontColor.set(1.0, 1.0, 1.0, 3.0);
	labelScore->setFontScale(1.8);
	camera->m_frontLayer->addChild(labelScore);



	// Start message (bottom-center)
	labelStartMessage = new cLabel(font);
	labelStartMessage->setText("Press SPACE to start Paint challenge");
	labelStartMessage->m_fontColor.set(1.0, 1.0, 0.0, 3.0); // Full RGB yellow with full opacity
	labelStartMessage->setFontScale(2.5);
	//labelStartMessage->setLocalPos(20, -40);



	camera->m_frontLayer->addChild(labelStartMessage);

	// Winner message (center screen)
	labelWinnerMessage = new cLabel(font);
	labelWinnerMessage->setText("");
	labelWinnerMessage->m_fontColor.setRed();
	labelWinnerMessage->setFontScale(2.5);
	camera->m_frontLayer->addChild(labelWinnerMessage);


	handler = new cHapticDeviceHandler();
	// grasp
	for (int i = 0; i < NUM_TOOLS; ++i)
	{
		graspLine[i] = new cShapeLine();
		graspLine[i]->setShowEnabled(false);
		graspLine[i]->setLineWidth(2.0);
		if (i == 0)
			graspLine[i]->m_colorPointA.setBlue();
		else
			graspLine[i]->m_colorPointA.setRed();

		graspLine[i]->m_colorPointB = graspLine[i]->m_colorPointA;
		world->addChild(graspLine[i]);
	}

	int numDevices = 2;
	double toolRadius = 0.04;

	for (int i = 0; i<numDevices; ++i)
	{
		handler->getDevice(hapticDevice[i], i);
		if (!hapticDevice[i]) continue;
		tool[i] = new cToolCursor(world);
		tool[i]->setHapticDevice(hapticDevice[i]);
		tool[i]->enableDynamicObjects(false);  //<--ignore ODE dynamic bodies
		tool[i]->setWorkspaceRadius(3.0);
		tool[i]->setRadius(toolRadius, toolRadius);
		tool[i]->setShowContactPoints(true, false);
		hapticDevice[i]->setEnableGripperUserSwitch(true);
		tool[i]->setLocalRot(camera->getLocalRot());
		tool[i]->setWaitForSmallForce(true);

		tool[i]->start();
		tool[i]->m_hapticPoint->m_sphereProxy->m_material->setColor(getDynamicToolColor(i));
		workspaceScaleFactor[i] = tool[i]->getWorkspaceScaleFactor();
		maxStiffness[i] = hapticDevice[i]->getSpecifications().m_maxLinearStiffness / workspaceScaleFactor[i];


		world->addChild(tool[i]);
	}

	ODEWorld = new cODEWorld(world);
	world->addChild(ODEWorld);
	ODEWorld->setGravity(cVector3d(0, 0, -9.81));
	ODEWorld->setLinearDamping(0.05);
	ODEWorld->setAngularDamping(0.05);

	const double ROOM_W = 3.0, ROOM_D = 3.0, ROOM_H = 3.0;
	const double HALF_W = ROOM_W*0.5, HALF_D = ROOM_D*0.5, FLOOR_Z = -1.0;

	cTexture2dPtr floorTex = cTexture2d::create();
	loadTextureAbsolute(floorTex, PATH_FLOOR);
	groundWall.mesh = createPlane(ROOM_W, ROOM_D, cVector3d(0, 0, FLOOR_Z), cMatrix3d(), cColorf(1, 1, 1), true, floorTex, true, 0.4*maxStiffness[0], 0.2, 0.2, 0.2);
	groundWall.mesh->setHapticEnabled(false);



	// Back wallcCreatePlane(
	cMatrix3d rotBack;  rotBack.setAxisAngleRotationDeg(0, 1, 0, 90);
	backWall.mesh = createPlane(ROOM_H, ROOM_D,
		cVector3d(-HALF_W, 0, FLOOR_Z + ROOM_H * 0.5), rotBack, cColorf(1, 1, 1));
	world->addChild(backWall.mesh);
	setupPaintWall(backWall, PATH_CANVAS, 0.8 * maxStiffness[0], 0.3, 0.2, 1.0);
	//backWall.brushScale = 1.0;  // normal

	// Left wall
	cMatrix3d rotLeft;  rotLeft.setAxisAngleRotationDeg(1, 0, 0, 270);
	leftWall.mesh = createPlane(ROOM_W, ROOM_H,
		cVector3d(0, -HALF_D, FLOOR_Z + ROOM_H * 0.5), rotLeft, cColorf(1, 1, 1));
	world->addChild(leftWall.mesh);

	setupPaintWall(leftWall, PATH_FOAM,
		0.1 * maxStiffness[0],   // very soft
		0.0,                     // slippery
		0.3,                     // some sliding
		0.8);                          ////////////////////////////////////////SCALING DOESNT WORK!! NEEDS TO BE FIXED. SIDE WALLS MORE STRECHED?///////////////////////////////////////////////////////////////////<- TO-DO



	// BACK WALL Normal Map
	cNormalMapPtr normalBack = cNormalMap::create();
	normalBack->createMap(backWall.texture);
	backWall.mesh->m_normalMap = normalBack;
	backWall.mesh->m_material->setTextureLevel(5.0);
	backWall.mesh->m_material->setUseHapticShading(false);



	// LEFT WALL Normal Map
	cNormalMapPtr normalLeft = cNormalMap::create();
	normalLeft->createMap(leftWall.texture);
	leftWall.mesh->m_normalMap = normalLeft;
	leftWall.mesh->m_material->setUseHapticShading(true);
	leftWall.mesh->m_material->setTextureLevel(20.0);



	// Right wall
	cMatrix3d rotRight; rotRight.setAxisAngleRotationDeg(1, 0, 0, 90);
	rightWall.mesh = createPlane(ROOM_W, ROOM_H,
		cVector3d(0, HALF_D, FLOOR_Z + ROOM_H * 0.5), rotRight, cColorf(1, 1, 1));
	world->addChild(rightWall.mesh);

	setupPaintWall(rightWall, PATH_STONE,
		0.8 * maxStiffness[0],   // strong resistance
		0.2,                     // some grip
		0.2,                     // less slide
		0.8);                    //
	// RIGHT WALL Normal Map
	cNormalMapPtr normalRight = cNormalMap::create();
	normalRight->createMap(rightWall.texture);
	rightWall.mesh->m_normalMap = normalRight;
	rightWall.mesh->m_material->setUseHapticShading(true);
	rightWall.mesh->m_material->setTextureLevel(10.0);


	//---------------------------------------------------------------------------
	// Invisible Blocking Walls: FRONT and CEILING
	//---------------------------------------------------------------------------

	cMaterial invisibleMat;
	invisibleMat.setWhite();
	invisibleMat.setTransparencyLevel(1.0);

	// FRONT BLOCKING WALL
	cMatrix3d rotFront;
	rotFront.setAxisAngleRotationDeg(0, 1, 0, -90);
	frontBlockingWall = new cMesh();
	cCreatePlane(frontBlockingWall, ROOM_H, ROOM_D,
		cVector3d(HALF_W, 0, FLOOR_Z + ROOM_H * 0.5), rotFront);

	frontBlockingWall->setMaterial(invisibleMat);
	frontBlockingWall->setUseMaterial(true);
	frontBlockingWall->setUseTransparency(true);
	frontBlockingWall->setShowEnabled(false);
	frontBlockingWall->createAABBCollisionDetector(0.05);
	frontBlockingWall->setHapticEnabled(true);
	frontBlockingWall->m_material->setStiffness(0.6 * maxStiffness[0]);
	world->addChild(frontBlockingWall);

	// CEILING BLOCKING WALL
	cMatrix3d rotCeiling;
	rotCeiling.setAxisAngleRotationDeg(1, 0, 0, 180);
	ceilingBlockingWall = new cMesh();
	cCreatePlane(ceilingBlockingWall, ROOM_W, ROOM_D,
		cVector3d(0, 0, FLOOR_Z + ROOM_H), rotCeiling);

	ceilingBlockingWall->setMaterial(invisibleMat);
	ceilingBlockingWall->setUseMaterial(true);
	ceilingBlockingWall->setUseTransparency(true);
	ceilingBlockingWall->setShowEnabled(false);
	ceilingBlockingWall->createAABBCollisionDetector(0.05);
	ceilingBlockingWall->setHapticEnabled(true);
	ceilingBlockingWall->m_material->setStiffness(0.6 * maxStiffness[0]);
	world->addChild(ceilingBlockingWall);



	// ODE planes

	// Create physics-only ODE floor (for object collisions)
	cODEGenericBody* odeFloor = new cODEGenericBody(ODEWorld);

	//-------------------------------------------------------
	////---- FLOOR ---------------------------------------
	//--------------------------------
	odeFloor->createStaticPlane(cVector3d(0.0, 0.0, FLOOR_Z), cVector3d(0.0, 0.0, 1.0));

	cMesh* visualFloor = new cMesh();
	world->addChild(visualFloor);

	double floorSize = 3.0;
	double floorHeight = 0.0;  // Z = 0 same as ODE floor
	cVector3d floorPos(0.0, 0.0, FLOOR_Z);

	cMatrix3d floorRot;
	floorRot.setAxisAngleRotationDeg(1, 0, 0, -0); // Laying flat in XY, facing up

	cCreatePlane(visualFloor, floorSize, floorSize, floorPos, floorRot);

	// Texture
	cTexture2dPtr texFloor = cTexture2d::create();
	bool ok = loadTextureAbsolute(texFloor, PATH_FLOOR);

	// Assign texture + enable
	visualFloor->setUseTexture(true);
	visualFloor->setTexture(texFloor);

	// Generate normal map
	cNormalMapPtr floorNormalMap = cNormalMap::create();
	floorNormalMap->createMap(texFloor);
	visualFloor->m_normalMap = floorNormalMap;

	// Material propertie
	visualFloor->m_material->setWhite();
	visualFloor->m_material->setStiffness(0.5 * maxStiffness[0]);
	visualFloor->m_material->setStaticFriction(0.2);
	visualFloor->m_material->setDynamicFriction(0.3);
	visualFloor->m_material->setTextureLevel(9);
	visualFloor->m_material->setHapticTriangleSides(true, false);
	visualFloor->m_material->setUseHapticShading(true);
	visualFloor->createAABBCollisionDetector(toolRadius);
	visualFloor->setHapticEnabled(true);


	//creating createStaticPlane
	for (int i = 0; i<6; ++i) {
		cODEGenericBody* plane = new cODEGenericBody(ODEWorld);
		if (i == 0) plane->createStaticPlane(cVector3d(0, 0, FLOOR_Z), cVector3d(0, 0, 1));
		if (i == 1) plane->createStaticPlane(cVector3d(0, 0, FLOOR_Z + ROOM_H), cVector3d(0, 0, -1));
		if (i == 2) plane->createStaticPlane(cVector3d(-HALF_W, 0, 0), cVector3d(1, 0, 0));
		if (i == 3) plane->createStaticPlane(cVector3d(HALF_W, 0, 0), cVector3d(-1, 0, 0));
		if (i == 4) plane->createStaticPlane(cVector3d(0, -HALF_D, 0), cVector3d(0, 1, 0));
		if (i == 5) plane->createStaticPlane(cVector3d(0, HALF_D, 0), cVector3d(0, -1, 0));
	}


	std::vector<cColorf> cubeColors = {
		cColorf(0.4f, 0.6f, 0.9f),  // Steel Blue
		cColorf(0.7f, 0.4f, 0.8f),  // Violet
		cColorf(0.9f, 0.5f, 0.4f),  // Coral
		cColorf(0.3f, 0.7f, 0.6f),  // Teal Mint
		cColorf(0.85f, 0.7f, 0.3f), // Gold
		cColorf(0.5f, 0.5f, 0.5f),  // Mid Gray
		cColorf(0.2f, 0.6f, 0.9f),  // Ocean Blue
	};
	// Boxes
	for (int i = 0; i < 20; ++i) {
		ODECube[i] = new cODEGenericBody(ODEWorld);

		cMesh* box = new cMesh();
		cCreateBox(box, 0.2, 0.2, 0.2);

		// Pick a random color from the curated palette
		int colorIndex = rand() % cubeColors.size();
		cColorf chosenColor = cubeColors[colorIndex];

		cMaterial* boxMaterial = new cMaterial();
		boxMaterial->setColor(chosenColor);
		boxMaterial->setStiffness(0.2 * maxStiffness[0]);
		boxMaterial->setStaticFriction(3);
		boxMaterial->setDynamicFriction(1);

		box->setMaterial(*boxMaterial);
		box->createAABBCollisionDetector(toolRadius);

		ODECube[i]->setImageModel(box);
		ODECube[i]->createDynamicBox(0.2, 0.2, 0.2);
		ODECube[i]->setMass(0.02 + 0.01 * i);
		ODECube[i]->setLocalPos(0.0, -0.4 + 0.4 * i, FLOOR_Z + 0.5);
	}



	// Load texture only once
	cTexture2dPtr texture_1 = cTexture2d::create();
	texture_1->loadFromFile(PATH_SPHERE_1);
	// Enable spherical mapping
	texture_1->setSphericalMappingEnabled(true);

	// Load texture only once
	cTexture2dPtr texture_2 = cTexture2d::create();
	texture_2->loadFromFile(PATH_SPHERE_2);
	// Enable spherical mapping
	texture_2->setSphericalMappingEnabled(true);

	// --- Create magnetic spheres_A ---
	for (int i = 0; i < NUM_SPHERES_A; i++)
	{
		// Create mesh and sphere geometry
		cMesh* mesh = new cMesh();
		cCreateSphere(mesh, SPHERE_RADIUS, 32, 32); // higher resolution for smoother texture

		// Apply texture
		mesh->setTexture(texture_1);
		mesh->setUseTexture(true);

		// Apply material
		cMaterialPtr mat = cMaterial::create();
		mat->setWhite();
		mat->setStiffness(0.3 * maxStiffness[0]);
		mat->setStaticFriction(0.3);
		mat->setDynamicFriction(0.2);
		mesh->setMaterial(mat);

		// Enable collision and haptics
		mesh->createAABBCollisionDetector(toolRadius);
		mesh->setHapticEnabled(true);

		// Compute initial position in circle
		double angle = i * 0.4;
		double radius = 0.5;
		cVector3d pos(
			radius * cos(angle),
			radius * sin(angle),
			FLOOR_Z + SPHERE_RADIUS + 0.02 * (i % 3)
			);

		// Create ODE body
		cODEGenericBody* sphereBody = new cODEGenericBody(ODEWorld);
		sphereBody->setImageModel(mesh);
		sphereBody->createDynamicSphere(SPHERE_RADIUS);
		sphereBody->setMass(0.02);
		sphereBody->setLocalPos(pos);

		// Add to world and store
		world->addChild(mesh);
		spheres_A[i] = sphereBody;
		sphereVel_A[i] = cVector3d(0, 0, 0);
	}

	// --- Create Type-B magnetic spheres ---
	for (int i = 0; i < NUM_SPHERES_B; i++)
	{
		cMesh* mesh = new cMesh();
		cCreateSphere(mesh, SPHERE_RADIUS, 32, 32);

		// Apply texture
		mesh->setTexture(texture_2);
		mesh->setUseTexture(true);

		// Optional: different texture or color
		cMaterialPtr mat = cMaterial::create();
		mat->setStiffness(0.3 * maxStiffness[0]);
		mesh->setMaterial(mat);

		mesh->createAABBCollisionDetector(toolRadius);
		mesh->setHapticEnabled(true);

		cVector3d pos(
			0.3 * cos(i * 0.8),
			0.3 * sin(i * 0.8),
			FLOOR_Z + SPHERE_RADIUS + 0.05
			);

		cODEGenericBody* body = new cODEGenericBody(ODEWorld);
		body->setImageModel(mesh);
		body->createDynamicSphere(SPHERE_RADIUS);
		body->setMass(0.02);
		body->setLocalPos(pos);

		world->addChild(mesh);
		spheres_B[i] = body;
		sphereVel_B[i] = cVector3d(0, 0, 0);
	}




	//---------------------------------------------------------------



	labelHapticRate = new cLabel(font);
	labelHapticRate->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelHapticRate);



	cThread* hThread = new cThread(); hThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	atexit(closeApp);
	glutTimerFunc(50, graphicsTimer, 0);
	glutMainLoop();
	return 0;
}

//------------------------------------------------------------------------------
void resizeWindow(int w, int h){ windowW = w; windowH = h; }
void keySelect(unsigned char key, int x, int y)
{
	if (key == 27 || key == 'x') exit(0);

	if (key == 'r')
	{
		cout << "Restart" << endl;
		gameStarted = false;
		gameEnded = false;
		countdownRunning = false;
		countdownFinished = false;
		floatingMode = false;
		ODEWorld->setGravity(cVector3d(0, 0, -9.81));

		ultUsed[0] = false;
		ultUsed[1] = false;
		ultReady[0] = false;
		ultReady[1] = false;
		ultActive[0] = false;
		ultActive[1] = false;
		labelUltStatus[0]->setEnabled(false);
		labelUltStatus[1]->setEnabled(false);
		labelUltStatus[0]->setText("");
		labelUltStatus[1]->setText("");


		labelScore->setText("");

		labelWinnerMessage->setText("");
		labelStartMessage->setShowEnabled(true);
		clearWalls();

	}

	if (key == ' ')
	{
		// Start countdown only if not already running
		if (!gameStarted && !countdownRunning && !gameEnded)
		{
			cout << "Starting countdown..." << endl;
			countdownRunning = true;
			countdownFinished = false;
			gameClock.reset();
			gameClock.start();
			clearWalls();
			labelScore->setText("");




			floatingMode = true;
			ODEWorld->setGravity(cVector3d(0, 0, 0));

			labelWinnerMessage->setText("");
			labelStartMessage->setFontScale(4.0);
			labelStartMessage->setShowEnabled(true);
			labelStartMessage->setText("3");
		}

		// Optional: toggle floating mode manually with extra space press (after start)
		if (gameStarted)
		{
			floatingMode = !floatingMode;
			if (floatingMode)
			{
				cout << "Floating mode ON" << endl;
				ODEWorld->setGravity(cVector3d(0, 0, 0));
			}
			else
			{
				cout << "Gravity ON" << endl;
				ODEWorld->setGravity(cVector3d(0, 0, -9.81));
			}
		}
	}

	if (key == 'f')
	{
		if (fullscreen)
		{
			glutReshapeWindow(windowW, windowH);
			fullscreen = false;
		}
		else
		{
			glutFullScreen();
			fullscreen = true;
		}
	}

	if (key == 'm')
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}

	if (key == 'c')
	{
		backWall.originalImage->copyTo(backWall.texture->m_image);
		leftWall.originalImage->copyTo(leftWall.texture->m_image);
		rightWall.originalImage->copyTo(rightWall.texture->m_image);

		backWall.texture->markForUpdate();
		leftWall.texture->markForUpdate();
		rightWall.texture->markForUpdate();
	}
}
void updateCameraPosition()
{
	double azimuthRad = cDegToRad(camAzimuthDeg);
	double polarRad = cDegToRad(camPolarDeg);

	double x = camRadius * sin(polarRad) * cos(azimuthRad);
	double y = camRadius * sin(polarRad) * sin(azimuthRad);
	double z = camRadius * cos(polarRad);

	cVector3d eye(x, y, z);
	cVector3d lookAt(0.0, 0.0, 0.0);
	cVector3d up(0.0, 0.0, 1.0);

	camera->set(eye, lookAt, up);
}

void mouseClick(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		if (button == 3) // Scroll up
		{
			camRadius -= 0.2;
		}
		else if (button == 4) // Scroll down
		{
			camRadius += 0.2;
		}

		// Clamp camera radius (so it doesn’t go too close/far)
		camRadius = cClamp(camRadius, 1.0, 10.0);

		// Update camera after scroll
		updateCameraPosition();
	}
}


void mouseButton(int button, int state, int x, int y)
{
	if (button == 3) // Scroll up
	{
		camRadius -= 0.1;
		camRadius = cClamp(camRadius, 1.0, 10.0);  // adjust bounds as needed
	}
	else if (button == 4) // Scroll down
	{
		camRadius += 0.1;
		camRadius = cClamp(camRadius, 1.0, 10.0);
	}

	// Force camera to update immediately
	double azimuthRad = cDegToRad(camAzimuthDeg);
	double polarRad = cDegToRad(camPolarDeg);
	double x_cam = camRadius * sin(polarRad) * cos(azimuthRad);
	double y_cam = camRadius * sin(polarRad) * sin(azimuthRad);
	double z_cam = camRadius * cos(polarRad);
	cVector3d eye(x_cam, y_cam, z_cam);
	cVector3d lookAt(0.0, 0.0, 0.0);
	cVector3d up(0.0, 0.0, 1.0);
	camera->set(eye, lookAt, up);
}

void mouseMove(int x, int y)
{
	int dx = x - mouseX;
	int dy = y - mouseY;
	mouseX = x;
	mouseY = y;

	double sensitivity = 0.03;
	camAzimuthDeg -= sensitivity * dx;  // horizontal drag -> rotate around
	camPolarDeg -= sensitivity * dy;  // vertical drag -> look up/down

	// clamp polar angle to avoid flipping
	camPolarDeg = cClamp(camPolarDeg, 20.0, 160.0);

	// convert to radians
	double azimuthRad = cDegToRad(camAzimuthDeg);
	double polarRad = cDegToRad(camPolarDeg);

	// convert spherical to Cartesian camera position
	double x_cam = camRadius * sin(polarRad) * cos(azimuthRad);
	double y_cam = camRadius * sin(polarRad) * sin(azimuthRad);
	double z_cam = camRadius * cos(polarRad);

	cVector3d eye(x_cam, y_cam, z_cam);
	cVector3d lookAt(0.0, 0.0, 0.0);
	cVector3d up(0.0, 0.0, 1.0);

	camera->set(eye, lookAt, up);

	for (int i = 0; i < 2; ++i)
	if (tool[i]) tool[i]->setLocalRot(camera->getLocalRot());
}



void closeApp(void){ simulationRunning = false; while (!simulationFinished) cSleepMs(100); for (int i = 0; i<2; ++i) if (tool[i]) tool[i]->stop(); }
void graphicsTimer(int d){ if (simulationRunning) glutPostRedisplay(); glutTimerFunc(50, graphicsTimer, 0); }


void computeAndShowWinner()
{
	// Compute scores
	double p1 = (computePaintCoverage(leftWall, leftWallTotalPixels)
		+ computePaintCoverage(rightWall, rightWallTotalPixels)
		+ computePaintCoverage(backWall, backWallTotalPixels)) / 3.0;

	double p2 = (computeUnpaintedCoverage(leftWall, leftWallTotalPixels)
		+ computeUnpaintedCoverage(rightWall, rightWallTotalPixels)
		+ computeUnpaintedCoverage(backWall, backWallTotalPixels)) / 3.0;

	// Decide winner
	string winnerText;
	if (p1 > p2)
		winnerText = "Player 1 Wins!";
	else if (p2 > p1)
		winnerText = "Player 2 Wins!";
	else
		winnerText = "It's a Tie!";

	//-------------------------------------------------------------------
	// Winner Label: Centered + Bigger + Opaque Red
	//-------------------------------------------------------------------
	labelWinnerMessage->setText(winnerText);
	labelWinnerMessage->setFontScale(4.5);
	labelWinnerMessage->m_fontColor.set(1.0, 0.0, 0.0, 1.0); // Fully opaque red

	int xMid = (windowW - labelWinnerMessage->getWidth()) / 2;
	int yMid = (windowH - labelWinnerMessage->getHeight()) / 2;
	labelWinnerMessage->setLocalPos(xMid - 100, 70);
	labelWinnerMessage->setEnabled(true);

	//-------------------------------------------------------------------
	// Score Label: Underneath + Pretty formatting
	//-------------------------------------------------------------------
	string scoreText = "(P1) Painter Score: " + cStr(p1, 1) + "%"     
		+ "(P2)Cleaner Score : " + cStr(p2, 1) + " %";
	labelScore->setFontScale(2.0);
	labelScore->setText(scoreText);
	labelScore->m_fontColor.setBlack();

	int xScore = (windowW - labelScore->getWidth()) / 2;
	int yScore = yMid - 80;
	labelScore->setLocalPos(xScore, yScore);
	labelScore->setEnabled(true);

	//-------------------------------------------------------------------
	// Console
	//-------------------------------------------------------------------
	cout << "\n====== FINAL SCORES ======\n";
	cout << "P1 (Painted):  " << p1 << " %\n";
	cout << "P2 (Erased):   " << p2 << " %\n";
	cout << "Winner: " << winnerText << endl;
}




void updateGraphics(void)
{
	// Update haptic rate
	labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");
	labelHapticRate->setLocalPos((int)(0.5*(windowW - labelHapticRate->getWidth())), 15);

	// Render
	world->updateShadowMaps(false, mirroredDisplay);
	camera->renderView(windowW, windowH);
	glutSwapBuffers();
	glFinish();
	GLenum err = glGetError();
	if (err != GL_NO_ERROR)
		printf("Error: %s\n", gluErrorString(err));

	// In-game logic
	if (gameStarted && !gameEnded)
	{
		double timeLeft = gameDuration - gameClock.getCurrentTimeSeconds();
		timeLeft = cClamp(timeLeft, 0.0, gameDuration);

		// Update timer label (top-right)
		stringstream ss;
		ss << "Time: " << fixed << setprecision(1) << timeLeft << "s";
		labelTimer->setText(ss.str());
		int centerX = (windowW - labelTimer->getWidth()) / 2;
		int topY = windowH - labelTimer->getHeight() - 60; // 40 px from the top
		labelTimer->setLocalPos(centerX, topY);
		labelStartMessage->setLocalPos(30, windowH - labelStartMessage->getHeight() - 40);

		if (timeLeft <= 0.0)
		{
			gameEnded = true;              // Prevent future updates
			computeAndShowWinner();       // Run winner logic once
		}

	}
	else if (!gameStarted && !gameEnded)
	{
		labelTimer->setText("Painting Challenge duration: " + cStr(gameDuration, 1) + "s");
		int centerX = (windowW - labelTimer->getWidth()) / 2;
		int topY = windowH - labelTimer->getHeight() - 60;

		labelTimer->setLocalPos(centerX, topY);
		labelStartMessage->setLocalPos(30, windowH - labelStartMessage->getHeight() - 40);
	}


	if (countdownRunning && !countdownFinished)
	{
		double elapsed = gameClock.getCurrentTimeSeconds();
		int secondsLeft = int(countdownDuration - elapsed) + 1;

		if (secondsLeft > 1)
		{
			labelStartMessage->setShowEnabled(true);
			labelStartMessage->setText(cStr(secondsLeft));
		}
		else if (secondsLeft == 1)
		{
			labelStartMessage->setText("START!");
			clearWalls();

		}
		else
		{
			//clearWalls(); // clean walls right before gameplay starts

			// Countdown finished, start the game
			countdownRunning = false;
			countdownFinished = true;
			labelStartMessage->setText("");
			labelStartMessage->setFontScale(2.5);



			leftWall.texture->m_image->copyTo(leftWall.originalImage);
			rightWall.texture->m_image->copyTo(rightWall.originalImage);
			backWall.texture->m_image->copyTo(backWall.originalImage);

			// Count visible pixels
			auto countVisiblePixels = [](const PaintWall& w) -> int {
				int width = w.originalImage->getWidth();
				int height = w.originalImage->getHeight();
				int total = 0;
				cColorb pixel;
				for (int x = 0; x < width; ++x)
				for (int y = 0; y < height; ++y)
				{
					w.originalImage->getPixelColor(x, y, pixel);
					if (pixel.getA() > 0) total++;
				}
				return total;
			};
			// Count total target pixels
			leftWallTotalPixels = countVisiblePixels(leftWall);
			rightWallTotalPixels = countVisiblePixels(rightWall);
			backWallTotalPixels = countVisiblePixels(backWall);
			gameStarted = true;
			gameEnded = false;

			gameClock.reset();
			gameClock.start();
		}

		// Position the countdown label in the middle
		int labelX = (windowW - labelStartMessage->getWidth()) / 2;
		int labelY = (windowH - labelStartMessage->getHeight()) / 2;
		labelStartMessage->setLocalPos(labelX, labelY);
	}






}



void updateHaptics(void)
{
	simulationRunning = true;
	simulationFinished = false;

	cPrecisionClock clock;
	clock.reset();

	while (simulationRunning)
	{
		clock.stop();
		double dt = clock.getCurrentTimeSeconds();
		double elapsedTime = gameClock.getCurrentTimeSeconds();

		////Mirror
		//if (elapsedTime >= 5.0 && elapsedTime < 10.0 && !mirrorActivated) {
		//	mirroredDisplay = true;
		//	camera->setMirrorVertical(mirroredDisplay);
		//	mirrorActivated = true;
		//}
		//else if (elapsedTime >= 10.0 && mirrorActivated) {
		//	mirroredDisplay = false;
		//	camera->setMirrorVertical(mirroredDisplay);
		//	mirrorActivated = false;
		//}
		//----------------------------------------
		// 1) Unlock ultimates at 1/3 of the game
		//----------------------------------------
		const double oneThird = 0;//gameDuration / 3.0;
		for (int i = 0; i < 2; ++i) {
			if (!ultReady[i] && !ultUsed[i] && gameStarted && elapsedTime >= oneThird) {
				ultReady[i] = true;
				labelUltStatus[i]->setText(
					i == 0
					? "ULT READY: Silence the Cleaner"
					: "ULT READY: Mirror the Room"
					);
				labelUltStatus[i]->setEnabled(true);
				cout << "ULTIMATE READY for Player " << (i + 1) << endl;
			}
		}
		clock.reset();
		clock.start();

		world->computeGlobalPositions(true);

		//----------------------------------------
		// 2) Haptic tool update, grabbing + paint/erase
		//----------------------------------------
		for (int i = 0; i < 2; ++i)
		{
			if (!tool[i]) continue;

			tool[i]->updateFromDevice();
			tool[i]->computeInteractionForces();

			// light damping
			cVector3d v;
			hapticDevice[i]->getLinearVelocity(v);
			tool[i]->addDeviceGlobalForce(-4.0 * v);

			cHapticPoint* hp = tool[i]->getHapticPoint(0);


			//------------------------------------------------------------------
			// GRAB LOGIC 
			//------------------------------------------------------------------
			cVector3d toolPos = tool[i]->getDeviceGlobalPos();
			bool buttonPressed = false;
			hapticDevice[i]->getUserSwitch(0, buttonPressed);


			if (!isGrasping[i] && buttonPressed)
			{
				if (hp && hp->getNumCollisionEvents() > 0)
				{
					cCollisionEvent* ev = hp->getCollisionEvent(0);
					cGenericObject* obj = ev->m_object->getOwner()->getOwner();
					cODEGenericBody* body = dynamic_cast<cODEGenericBody*>(obj);
					if (body)
					{
						isGrasping[i] = true;
						graspObject[i] = body;
						graspOffset[i] = ev->m_globalPos - body->getGlobalPos();
						graspLine[i]->setShowEnabled(true);
					}
				}
			}
			else if (isGrasping[i] && buttonPressed && graspObject[i])
			{
				cVector3d targetPos = toolPos - graspOffset[i];
				cVector3d currentPos = graspObject[i]->getGlobalPos();
				cVector3d f = graspSpringStiffness * (targetPos - currentPos);

				graspObject[i]->addExternalForceAtPoint(f, currentPos);
				tool[i]->addDeviceGlobalForce(-f);

				graspLine[i]->m_pointA = toolPos;
				graspLine[i]->m_pointB = currentPos;
			}
			else if (isGrasping[i] && !buttonPressed)
			{
				isGrasping[i] = false;
				graspObject[i] = nullptr;
				graspLine[i]->setShowEnabled(false);
			}


			// paint / erase
			if (hp && hp->getNumCollisionEvents() > 0)
			{
				auto ev = hp->getCollisionEvent(0);
				PaintWall* w = nullptr;
				if (ev->m_object == backWall.mesh)  w = &backWall;
				else if (ev->m_object == leftWall.mesh)  w = &leftWall;
				else if (ev->m_object == rightWall.mesh) w = &rightWall;

				if (w)
				{
					unsigned tri = ev->m_index;
					cVector3d tex = ev->m_triangles->getTexCoordAtPosition(tri, ev->m_localPos);
					int px, py;
					w->texture->m_image->getPixelLocation(tex, px, py);
					double forceMag = tool[i]->getDeviceGlobalForce().length();

					if (i == 0) {
						paintPixels(*w, px, py, getDynamicToolColor(i), forceMag, dt);
					}
					else if (i == 1 && canErase) {  // ← uses canErase flag
						erasePixels(*w, px, py, forceMag, dt);
					}
				}
			}


			//----------------------------------------
			// 3) Check for ultimate activation
			//----------------------------------------
			if (ultReady[i] && !ultUsed[i])
			{
				bool anyPressed = false;
				for (int b = 1; b <= 3; ++b) {
					bool pressed = false;
					hapticDevice[i]->getUserSwitch(b, pressed);
					if (pressed) { anyPressed = true; break; }
				}
				if (anyPressed)
				{
					ultUsed[i] = true;
					ultActive[i] = true;
					ultTimer[i].reset();
					ultTimer[i].start();
					labelUltStatus[i]->setEnabled(true);

					if (i == 0) {
						// Painter’s ult
						canErase = false;
						labelUltStatus[0]->setText("ULT ACTIVE: Cleaner silenced!");
						cout << "Painter ultimate triggered!" << endl;
					}
					else {
						// Eraser’s ult
						mirroredDisplay = true;
						camera->setMirrorVertical(mirroredDisplay);
						labelUltStatus[1]->setText("ULT ACTIVE: Mirror the Room!");
						cout << "Eraser ultimate triggered!" << endl;
					}
				}
			}

			tool[i]->applyToDevice();
		}


		// --- Magnetic sphere physics update ---
		const double K_MAGNET = 150.0;          // magnetic force constant (increased) 
		const double SPHERE_STIFFNESS = 500.0;  // collision repulsion force
		const double SPHERE_MASS = 0.04;        // mass of each sphere
		const double K_DAMPING = 0.99;          // velocity damping factor
		const double MIN_ATTRACT_DIST = 0.06;   // minimum distance to start attraction
		const double MAX_ATTRACT_DIST = 0.3;    // maximum distance for attraction
		const double REPEL_STRENGTH = 400.0;	// strength for A-A or B-B repulsion

		// Initialize forces
		cVector3d forceA[NUM_SPHERES_A];
		cVector3d forceB[NUM_SPHERES_B];
		for (int i = 0; i < NUM_SPHERES_A; i++) forceA[i].zero();
		for (int i = 0; i < NUM_SPHERES_B; i++) forceB[i].zero();

		// --- A-A repulsion ---
		for (int i = 0; i < NUM_SPHERES_A; i++)
		{
			for (int j = i + 1; j < NUM_SPHERES_A; j++)
			{
				cVector3d pos0 = spheres_A[i]->getLocalPos();
				cVector3d pos1 = spheres_A[j]->getLocalPos();
				cVector3d dir = pos1 - pos0;
				double dist = dir.length();
				if (dist > 1e-5) dir.normalize();

				if (dist < 2.5 * SPHERE_RADIUS)
				{
					cVector3d f = -REPEL_STRENGTH * (2.0 * SPHERE_RADIUS - dist) * dir;
					forceA[i] -= f;
					forceA[j] += f;
				}
			}
		}

		// --- B-B repulsion ---
		for (int i = 0; i < NUM_SPHERES_B; i++)
		{
			for (int j = i + 1; j < NUM_SPHERES_B; j++)
			{
				cVector3d pos0 = spheres_B[i]->getLocalPos();
				cVector3d pos1 = spheres_B[j]->getLocalPos();
				cVector3d dir = pos1 - pos0;
				double dist = dir.length();
				if (dist > 1e-5) dir.normalize();

				if (dist < 2.5 * SPHERE_RADIUS)
				{
					cVector3d f = -REPEL_STRENGTH * (2.0 * SPHERE_RADIUS - dist) * dir;
					forceB[i] -= f;
					forceB[j] += f;
				}
			}
		}

		// --- A-B attraction ---
		for (int i = 0; i < NUM_SPHERES_A; i++)
		{
			for (int j = 0; j < NUM_SPHERES_B; j++)
			{
				cVector3d posA = spheres_A[i]->getLocalPos();
				cVector3d posB = spheres_B[j]->getLocalPos();
				cVector3d dir = posB - posA;
				double dist = dir.length();
				if (dist > 1e-5) dir.normalize();

				if (dist > MIN_ATTRACT_DIST && dist < MAX_ATTRACT_DIST)
				{
					// Attraction pulls A toward B, and B toward A
					double strength = K_MAGNET * (dist - MIN_ATTRACT_DIST);
					cVector3d f = strength * dir;  // Negative to pull them together

					forceA[i] += f;
					forceB[j] -= f;
				}
			}
		}

		// --- Apply forces to each sphere ---
		for (int i = 0; i < NUM_SPHERES_A; i++)
			spheres_A[i]->addExternalForce(forceA[i]);

		for (int i = 0; i < NUM_SPHERES_B; i++)
			spheres_B[i]->addExternalForce(forceB[i]);


		if (floatingMode)
		{
			for (int i = 0; i < 10; ++i)
			{
				if (!ODECube[i]) continue;

				// Small random drifting force
				double fx = 0.5 * sin(0.3 * elapsedTime + i);
				double fy = 0.5 * cos(0.2 * elapsedTime + i * 2);
				double fz = 0.5 * sin(0.5 * elapsedTime + i * 1.5);

				cVector3d floatForce(fx, fy, fz);
				ODECube[i]->addExternalForce(floatForce);

				// : small torque for rotation
				cVector3d torque(0.01 * sin(elapsedTime + i), 0.01 * cos(elapsedTime + i), 0.01);
				ODECube[i]->addExternalTorque(torque);
			}
		}
		ODEWorld->updateDynamics(dt);
		// 5) Ultimate timeout (per player)
		for (int i = 0; i < 2; ++i) {
			if (ultActive[i] &&
				ultTimer[i].getCurrentTimeSeconds() >= ultDuration[i])
			{
				ultActive[i] = false;
				labelUltStatus[i]->setText("");
				labelUltStatus[i]->setEnabled(false);
				cout << "ULTIMATE ended for Player " << (i + 1) << endl;


				if (i == 0) {
					// Painter’s ult ended → Cleaner can erase again
					canErase = true;
				}
				else {
					// Eraser’s ult ended → turn off the mirror
					mirroredDisplay = false;
					camera->setMirrorVertical(mirroredDisplay);
					cout << "Cleaner ultimate ended: MIRROR OFF" << endl;
				}
			}
		}

		// hz counter
		frequencyCounter.signal(1);
	}

	simulationFinished = true;




}