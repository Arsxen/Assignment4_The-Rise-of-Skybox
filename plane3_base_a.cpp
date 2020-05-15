//|___________________________________________________________________
//!
//! \file plane3_base.cpp
//!
//! \brief Base source code for the third plane assignment.
//!
//! Author: Mores Prachyabrued.
//!
//! Author:
//! Krittin Chatrinan 6088022
//! Archawat Silachote 6088168
//! Sunat Praphanwong 6088130 
//!
//! Keyboard inputs for plane and propeller (subpart):
//!   Plane1 Control:
//!		i - Forward translation(+z)
//!		k - Backward translation(-z)
//!		j / l - Yaws(+-y rotation)
//!		u / o - Rolls(+-z rotation)
//!		p / , -Pitches(+-x rotation)
//!
//!	  Plane2 Control :
//!		w - Forward translation(+z)
//!		s - Backward translation(-z)
//!		a / d - Yaws(+-y rotation)
//!		q / e - Rolls(+-z rotation)
//!		x / z - Pitches(+-x rotation)
//!
//!		h / g - Rotate the subpart B(Left Wing)
//!		t / r - Rotate the subpart C(Right Wing)
//!		c / n - Rotate the subpart A(Gun Platform)
//!
//!	  Light Control
//!		1,3 = translates light up/down (+y/-y)
//!		2,8 = translates light (+z/-z)
//!		4,6 = translates light (+x/-x)
//!		9   = toggles diffuse light on/off
//!		5   = toggles specular light on/off
//!		7   = toggles ambient light on/off
//!
//! Mouse inputs for world-relative camera:
//!   Hold left button and drag  = controls azimuth and elevation
//!                                (Press CTRL (and hold) before left button to restrict to azimuth control only,
//!                                 Press SHIFT (and hold) before left button to restrict to elevation control only)
//!   Hold right button and drag = controls distance
//!
//! TODO: Extend the code to satisfy the requirements given in the assignment handout
//!
//! Note: Good programmer uses good comments! :)
//|___________________________________________________________________

#define _CRT_SECURE_NO_WARNINGS

//|___________________
//|
//| Includes
//|___________________

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>

#include <gmtl/gmtl.h>

#include <GL/glut.h>

//|___________________
//|
//| Constants
//|___________________

// Plane dimensions
const float P_WIDTH = 3.0f;
const float P_LENGTH = 3.0f;
const float P_HEIGHT = 1.5f;

// Plane transforms
const gmtl::Vec3f PLANE_FORWARD(0, 0, 1.0f);            // Plane's forward translation vector (w.r.t. local frame)
const float PLANE_ROTATION = 5.0f;                      // Plane rotated by 5 degs per input

// Propeller dimensions (subpart)
const float GUNBARREL_RADIUS = 0.02f;
const float GUNBARREL_HEIGHT = 0.5f;

// Propeller transforms
const gmtl::Point3f WING_POS(0, 0, 0);     // Propeller position on the plane (w.r.t. plane's frame)
const gmtl::Point3f PLATFORM_POS(0, 0, (P_HEIGHT * 2) / 3);
const gmtl::Point3f GUNBARREL_POS(0.1f, -0.525f, -0.3f);
const float PROPELLER_ROTATION = 2.0f;                  // Propeller rotated by 5 degs per input

// Camera's view frustum
const float CAM_FOV = 90.0f;                     // Field of view in degs

// Keyboard modifiers
enum KeyModifier { KM_SHIFT = 0, KM_CTRL, KM_ALT };

// Textures
enum TextureID { TID_SKYBACK = 0, TID_SKYLEFT, TID_SKYBOTTOM, TID_SKYTOP, TID_SKYFRONT, TID_SKYRIGHT, TEXTURE_NB };  // Texture IDs, with the last ID indicating the total number of textures

// Skybox
const float SB_SIZE = 1000.0f;                     // Skybox dimension

// Lighting
const GLfloat NO_LIGHT[] = { 0.0, 0.0, 0.0, 1.0 };
const GLfloat AMBIENT_LIGHT[] = { 0.1, 0.1, 0.1, 1.0 };
const GLfloat DIFFUSE_LIGHT[] = { 0.5, 0.5, 0.5, 1.0 };
const GLfloat SPECULAR_LIGHT[] = { 0.5, 0.5, 0.5, 1.0 };

// Materials
const GLfloat DARKRED_COL[] = { 0.1, 0.0, 0.0, 1.0 };
const GLfloat BRIGHTRED_COL[] = { 0.7, 0.0, 0.0, 1.0 };
const GLfloat DARKBLUE_COL[] = { 0.0, 0.0, 0.1, 1.0 };
const GLfloat BRIGHTBLUE_COL[] = { 0.0, 0.0, 0.7, 1.0 };
const GLfloat DARK_COL[] = { 0.1, 0.1, 0.1, 1.0 };
const GLfloat MEDIUMWHITE_COL[] = { 0.7, 0.7, 0.7, 1.0 };
const GLfloat SPECULAR_COL[] = { 0.7, 0.7, 0.7, 1.0 };
const GLfloat GREEN_COL[] = { 0.55,0.95,0.64,1.0 };
const GLfloat DARKGREEN_COL[] = { 0.0, 0.1, 0.0, 1.0 };
const GLfloat DARKGRAY_COL[] = { 0.26, 0.26, 0.26, 1.0 };
const GLfloat GRAY_COL[] = { 0.75, 0.75, 0.75, 1.0 };
const GLfloat WHITE_COL[] = { 1.0, 1.0, 1.0, 1.0 };
const GLfloat PASTEL_COL[] = { 1.0, 0.69, 0.94, 1.0 };
const GLfloat PASTELDARKER_COL[] = { 0.90, 0.59, 0.69, 1.0 };

// Sphere Constant
const int space = 10;
const int VertexCount = (90 / space) * (360 / space) * 4;

//|___________________
//|
//| Global Variables
//|___________________

// Track window dimensions, initialized to 800x600
int w_width = 800;
int w_height = 600;

// Plane pose (position-quaternion pair)
gmtl::Point4f plane_p;      // Position (using explicit homogeneous form; see Quaternion example code)
gmtl::Quatf plane_q;        // Quaternion

// Plane 1 pose
gmtl::Point4f plane1_p;		// Position
gmtl::Quatf plane1_q;		// Quaternion

// Quaternions to rotate plane
gmtl::Quatf zrotp_q;        // Positive and negative Z rotations
gmtl::Quatf zrotn_q;

gmtl::Quatf xrotp_q;		// Positive and negative X rotations
gmtl::Quatf xrotn_q;

gmtl::Quatf yrotp_q;		// Positive and negative Y rotations
gmtl::Quatf yrotn_q;

// Propeller rotation (subpart)
float pp_angle = 0;         // Rotation angle for subpart C
float pp_angle_2 = 0;       // Rotation angle for subpart B
float platform = 0;			// Rotation angle for subpart A

// Mouse & keyboard
int mx_prev = 0, my_prev = 0;
bool mbuttons[3] = { false, false, false };
bool kmodifiers[3] = { false, false, false };

// Cameras
int cam_id = 0;                                // Selects which camera to view
int camctrl_id = 0;                                // Selects which camera to control
float distance[3] = { 5.0f, 5.0f, 5.0f };     // Distance of the camera from world's origin.
float elevation[3] = { -45.0f, 0.0f, -15.0f }; // Elevation of the camera. (in degs)
float azimuth[3] = { 15.0,  -45.0f, -10.0f }; // Azimuth of the camera. (in degs)

// Lighting
gmtl::Point4f light_pos(0.0, 20.0, 20.0, 1.0);
bool is_diffuse_on = true;
bool is_ambient_on = true;
bool is_specular_on = true;

// Textures
GLuint textures[TEXTURE_NB];                           // Textures
GLuint satellite_wing;
GLuint earth_texture;
GLuint plane_wing;

//Sphere Vertices
typedef struct
{
	int X;
	int Y;
	int Z;
	double U;
	double V;
}VERTICES;
VERTICES SphereVertices[VertexCount];

//|___________________
//|
//| Function Prototypes
//|___________________

gmtl::Vec3f FindNormal(const gmtl::Point3f& p1, const gmtl::Point3f& p2, const gmtl::Point3f& p3);
void InitTransforms();
void InitGL(void);
void DisplayFunc(void);
void KeyboardFunc(unsigned char key, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void ReshapeFunc(int w, int h);
void DrawCoordinateFrame(const float l);
void DrawPlaneBody(const float width, const float length, const float height);
void DrawLeftWing(const float width, const float length, const float height);
void DrawRightWing(const float width, const float length, const float height);
void DrawPlatform(const float width, const float length, const float height);
void DrawGunBarrel(const float radius, const float height);
void DrawSkybox(const float s);
void SetLight(const gmtl::Point4f& pos, const bool is_ambient, const bool is_diffuse, const bool is_specular);
void drawCamera(const float radius, const float height);
void drawLight(const float radius);
void drawSatellite();
void CreateSphere(double R, double H, double K, double Z);
void DrawSphere();
gmtl::Point3f sphericalToRectangular(float distance, float azimuth, float elevation);
gmtl::Vec3f calculateSurfaceNormal(std::vector<gmtl::Point3f> vertices);
void LoadPPM(const char* fname, unsigned int* w, unsigned int* h, unsigned char** data, const int mallocflag);

//|____________________________________________________________________
//|
//| Function: FindNormal
//|
//! \param p1	[in] Point 1.
//! \param p2	[in] Point 2.
//! \param p3	[in] Point 3.
//! \return Normalized surface normal.
//!
//! Finds the surface normal of a triangle. The input must be in CCW order.
//|____________________________________________________________________

gmtl::Vec3f FindNormal(const gmtl::Point3f& p1,
	const gmtl::Point3f& p2,
	const gmtl::Point3f& p3)
{
	gmtl::Vec3f v12 = p2 - p1;
	gmtl::Vec3f v13 = p3 - p1;

	gmtl::Vec3f normal;
	gmtl::cross(normal, v12, v13);
	gmtl::normalize(normal);

	return normal;
}

//|____________________________________________________________________
//|
//| Function: InitTransforms
//|
//! \param None.
//! \return None.
//!
//! Initializes all the transforms
//|____________________________________________________________________

void InitTransforms()
{
	const float COSTHETA_D2 = cos(gmtl::Math::deg2Rad(PLANE_ROTATION / 2));  // cos() and sin() expect radians
	const float SINTHETA_D2 = sin(gmtl::Math::deg2Rad(PLANE_ROTATION / 2));

	// Inits plane pose
	plane_p.set(1.0f, 0.0f, 4.0f, 1.0f);
	plane_q.set(0, 0, 0, 1);

	// Inits plane1 pose
	plane1_p.set(1.0f, 0, -10.f, 1.0f);
	plane1_q.set(0, 0, 0, 1);

	// Z rotations (roll)
	zrotp_q.set(0, 0, SINTHETA_D2, COSTHETA_D2);      // +Z
	zrotn_q = gmtl::makeConj(zrotp_q);                // -Z

	// X rotation (pitch)
	xrotp_q.set(SINTHETA_D2, 0, 0, COSTHETA_D2);      // +X
	xrotn_q = gmtl::makeConj(xrotp_q);				  // -X

	// Y rotation (yaw)
	yrotp_q.set(0, SINTHETA_D2, 0, COSTHETA_D2);      // +Y
	yrotn_q = gmtl::makeConj(yrotp_q);

	// Sphere
	CreateSphere(300, 0, 0, 0);
}

//|____________________________________________________________________
//|
//| Function: InitGL
//|
//! \param None.
//! \return None.
//!
//! OpenGL initializations
//|____________________________________________________________________

void InitGL(void)
{
	unsigned char* img_data;               // Texture image data
	unsigned int  width;                   // Texture width
	unsigned int  height;                  // Texture height

	glClearColor(0.7f, 0.7f, 0.7f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	//|___________________________________________________________________
	//|
	//| Setup lighting
	//|___________________________________________________________________

	  // Disable global ambient
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, NO_LIGHT);

	// NOTE: for specular reflections, the "local viewer" model produces better
	// results than the default, but is slower. The default would not use the correct
	// vertex-to-eyepoint vector, treating it as always parallel to Z.
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

	// Disable two sided lighting because we cannot see the inside of the most object in the scene (except satellite)
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

	// Enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	//|___________________________________________________________________
	//|
	//| Setup texturing
	//|___________________________________________________________________

	  // Describe how data will be stored in memory
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	// Select the method for combining texture color with the lighting equation
	  // (look up the third parameter)
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// Generate and setup texture objects
	glGenTextures(TEXTURE_NB, textures);

	// Skybox back wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBACK]);
	LoadPPM("skybox_back.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox left wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYLEFT]);
	LoadPPM("skybox_left.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox buttom wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBOTTOM]);
	LoadPPM("skybox_bottom.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox front wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYFRONT]);
	LoadPPM("skybox_front.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox right wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYRIGHT]);
	LoadPPM("skybox_right.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Skybox top wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYTOP]);
	LoadPPM("skybox_top.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Satellite Wing Texture
	glGenTextures(1, &satellite_wing);
	glBindTexture(GL_TEXTURE_2D, satellite_wing);
	LoadPPM("satellite_wing.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Earth Texture
	glGenTextures(1, &earth_texture);
	glBindTexture(GL_TEXTURE_2D, earth_texture);
	LoadPPM("earth.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	// Plane Wing Texture
	glGenTextures(1, &plane_wing);
	glBindTexture(GL_TEXTURE_2D, plane_wing);
	LoadPPM("wing_texture.ppm", &width, &height, &img_data, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, img_data);
	free(img_data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

//|____________________________________________________________________
//|
//| Function: DisplayFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT display callback function: called for every redraw event.
//|____________________________________________________________________

void DisplayFunc(void)
{
	gmtl::AxisAnglef aa;    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	gmtl::Vec3f axis;       // Axis component of axis-angle representation
	float angle;            // Angle component of axis-angle representation

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(CAM_FOV, (float)w_width / w_height, 0.1f, 1000.0f);     // Check MSDN: google "gluPerspective msdn"

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//|____________________________________________________________________
	//|
	//| Setting up view transform by:
	//| "move up to the world frame by composing all of the (inverse) transforms from the camera up to the world node"
	//|____________________________________________________________________

	switch (cam_id) {
	case 0:
		// For the world-relative camera
		glTranslatef(0, 0, -distance[0]);
		glRotatef(-elevation[0], 1, 0, 0);
		glRotatef(-azimuth[0], 0, 1, 0);
		break;

	case 1:
		// For plane2's camera
		glTranslatef(0, 0, -distance[1]);
		glRotatef(-elevation[1], 1, 0, 0);
		glRotatef(-azimuth[1], 0, 1, 0);

		gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
		axis = aa.getAxis();
		angle = aa.getAngle();
		glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
		glTranslatef(-plane_p[0], -plane_p[1], -plane_p[2]);
		break;

	case 2:
		// For plane2's camera
		glTranslatef(0, 0, -distance[2]);
		glRotatef(-elevation[2], 1, 0, 0);
		glRotatef(-azimuth[2], 0, 1, 0);

		gmtl::set(aa, plane1_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
		axis = aa.getAxis();
		angle = aa.getAngle();
		glRotatef(-gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
		glTranslatef(-plane1_p[0], -plane1_p[1], -plane1_p[2]);
		break;
	}

	//|____________________________________________________________________
	//|
	//| Draw traversal begins, start from world (root) node
	//|____________________________________________________________________

	  // Set light position wrt world
	SetLight(light_pos, is_ambient_on, is_diffuse_on, is_specular_on);
	// DrawLight
	glPushMatrix();
	glTranslatef(light_pos[0], light_pos[1], light_pos[2]);
	drawLight(0.5);
	glPopMatrix();

	// World node: draws world coordinate frame
	DrawCoordinateFrame(10);

	// World-relative camera:
	if (cam_id != 0) {
		glPushMatrix();
		glRotatef(azimuth[0], 0, 1, 0);
		glRotatef(elevation[0], 1, 0, 0);
		glTranslatef(0, 0, distance[0]);
		drawCamera(0.5, 1);
		DrawCoordinateFrame(1);
		glPopMatrix();
	}
	//World Camera-relative skybox
	if (cam_id == 0) {
		glPushMatrix();
		//get position of world-relative camera
		gmtl::Point3f position = sphericalToRectangular(distance[0], azimuth[0], elevation[0]);
		glTranslatef(position[0], position[1], position[2]);
		DrawSkybox(SB_SIZE);
		glPopMatrix();
	}

	// Plane 1:
	glPushMatrix();
	gmtl::set(aa, plane1_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	axis = aa.getAxis();
	angle = aa.getAngle();
	glTranslatef(plane1_p[0], plane1_p[1], plane1_p[2]);
	glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
	DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);

	DrawCoordinateFrame(3);

	//Drawing other parts of plane without Push/Pop Matrix to merge it to 1 object
	//Draw Left Wing
	glTranslatef(WING_POS[0], WING_POS[1], WING_POS[2]);
	glRotatef(0, 0, 0, 1);
	DrawLeftWing(P_WIDTH, P_LENGTH, P_HEIGHT);

	//Draw Right Wing
	glTranslatef(WING_POS[0], WING_POS[1], WING_POS[2]);
	glRotatef(0, 0, 0, 1);
	DrawRightWing(P_WIDTH, P_LENGTH, P_HEIGHT);

	//Draw Platform
	glTranslatef(PLATFORM_POS[0], PLATFORM_POS[1], PLATFORM_POS[2]);
	DrawPlatform(P_WIDTH, P_LENGTH, P_HEIGHT);

	//Draw Left Barrel
	glTranslatef(-GUNBARREL_POS[0], GUNBARREL_POS[1], GUNBARREL_POS[2]);
	DrawGunBarrel(GUNBARREL_RADIUS, GUNBARREL_HEIGHT);
	glTranslatef(GUNBARREL_POS[0], -GUNBARREL_POS[1], -GUNBARREL_POS[2]); // Reset position

	//Draw Right Barrel
	glTranslatef(GUNBARREL_POS[0], GUNBARREL_POS[1], GUNBARREL_POS[2]);
	DrawGunBarrel(GUNBARREL_RADIUS, GUNBARREL_HEIGHT);

	// Plane 1's camera:
	if (cam_id != 2) {
		glPushMatrix();
		glRotatef(azimuth[2], 0, 1, 0);
		glRotatef(elevation[2], 1, 0, 0);
		glTranslatef(0, 0, distance[2]);
		drawCamera(0.5, 1);
		DrawCoordinateFrame(1);
		glPopMatrix();
	}
	// Plane 1's camera-relative skybox
	if (cam_id == 2) {
		glPushMatrix();
		//get position of world-relative camera
		gmtl::Point3f position = sphericalToRectangular(distance[2], azimuth[2], elevation[2]);
		glTranslatef(position[0], position[1], position[2]);
		DrawSkybox(SB_SIZE);
		glPopMatrix();
	}

	glPopMatrix();

	// Plane 2 body:
	glPushMatrix();
	gmtl::set(aa, plane_q);                    // Converts plane's quaternion to axis-angle form to be used by glRotatef()
	axis = aa.getAxis();
	angle = aa.getAngle();
	glTranslatef(plane_p[0], plane_p[1], plane_p[2]);
	glRotatef(gmtl::Math::rad2Deg(angle), axis[0], axis[1], axis[2]);
	DrawPlaneBody(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(3);

	// Plane 2's camera:
	if (cam_id != 1) {
		glPushMatrix();
		glRotatef(azimuth[1], 0, 1, 0);
		glRotatef(elevation[1], 1, 0, 0);
		glTranslatef(0, 0, distance[1]);
		drawCamera(0.5, 1);
		DrawCoordinateFrame(1);
		glPopMatrix();
	}
	// Plane 2's camera-relative skybox
	if (cam_id == 1) {
		glPushMatrix();
		//get position of world-relative camera
		gmtl::Point3f position = sphericalToRectangular(distance[1], azimuth[1], elevation[1]);
		glTranslatef(position[0], position[1], position[2]);
		DrawSkybox(SB_SIZE);
		glPopMatrix();
	}

	// LeftWing (subpart B):
	glPushMatrix();
	glTranslatef(WING_POS[0], WING_POS[1], WING_POS[2]);
	glRotatef(pp_angle, 0, 0, 1);
	DrawLeftWing(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(1);
	glPopMatrix();

	// RightWing (subpart C):
	glPushMatrix();
	glTranslatef(WING_POS[0], WING_POS[1], WING_POS[2]);
	glRotatef(pp_angle_2, 0, 0, 1);
	DrawRightWing(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(1);
	glPopMatrix();

	// Plaform (subpart A):
	glPushMatrix();
	glTranslatef(PLATFORM_POS[0], PLATFORM_POS[1], PLATFORM_POS[2]);
	glRotatef(platform, 0, 1, 0);
	DrawPlatform(P_WIDTH, P_LENGTH, P_HEIGHT);
	DrawCoordinateFrame(1);

	//Left Gun Barrel (subsubpart A)
	glPushMatrix();
	glTranslatef(-GUNBARREL_POS[0], GUNBARREL_POS[1], GUNBARREL_POS[2]);
	DrawGunBarrel(GUNBARREL_RADIUS, GUNBARREL_HEIGHT);
	DrawCoordinateFrame(1);
	glPopMatrix();

	//Right Gun Barrel (subsubpart B)
	glPushMatrix();
	glTranslatef(GUNBARREL_POS[0], GUNBARREL_POS[1], GUNBARREL_POS[2]);
	DrawGunBarrel(GUNBARREL_RADIUS, GUNBARREL_HEIGHT);
	DrawCoordinateFrame(1);
	glPopMatrix();
	glPopMatrix();

	glPopMatrix();

	//Satellite1
	glPushMatrix();
	glTranslatef(10, 10, 20);
	glRotatef(60, 1, 0, 0);
	drawSatellite();
	glPopMatrix();

	//Satellite2
	glPushMatrix();
	glTranslatef(-20, 35, 20);
	glRotatef(120, 1, 0, 0);
	drawSatellite();
	glPopMatrix();

	//Earth
	glPushMatrix();
	glTranslatef(0, 0,400);
	DrawSphere();
	glPopMatrix();

	glutSwapBuffers();                          // Replaces glFlush() to use double buffering
}

//|____________________________________________________________________
//|
//| Function: KeyboardFunc
//|
//! \param key    [in] Key code.
//! \param x      [in] X-coordinate of mouse when key is pressed.
//! \param y      [in] Y-coordinate of mouse when key is pressed.
//! \return None.
//!
//! GLUT keyboard callback function: called for every key press event.
//|____________________________________________________________________

void KeyboardFunc(unsigned char key, int x, int y)
{
	switch (key) {
		//|____________________________________________________________________
		//|
		//| Camera switch
		//|____________________________________________________________________

	case 'v': // Select camera to view
		cam_id = (cam_id + 1) % 3;
		printf("View camera = %d\n", cam_id);
		break;
	case 'b': // Select camera to control
		camctrl_id = (camctrl_id + 1) % 3;
		printf("Control camera = %d\n", camctrl_id);
		break;

		//|____________________________________________________________________
		//|
		//| Plane controls
		//|____________________________________________________________________

			  // Plane Number 2
	case 'w': { // Forward translation of the plane (+Z translation)
		gmtl::Quatf v_q = plane_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
		plane_p = plane_p + v_q.mData;
	} break;
	case 's': { // Backward translation of the plane (-Z translation)
		gmtl::Quatf v_q = plane_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane_q);
		plane_p = plane_p + v_q.mData;
	} break;
	case 'a': { // Yaws the plane (+Y rot)
		plane_q = plane_q * yrotp_q;
		break;
	}
	case 'd': { // Yaws the plane (-Y rot)
		plane_q = plane_q * yrotn_q;
		break;
	}
	case 'q': { // Rolls the plane (+Z rot)
		plane_q = plane_q * zrotp_q;
		break;
	}
	case 'e': // Rolls the plane (-Z rot)
	{
		plane_q = plane_q * zrotn_q;
		break;
	}
	case 'x': // Pitches the plane (+X rot)
	{
		plane_q = plane_q * xrotp_q;
		break;
	}
	case 'z': { // Pitches the plane (-X rot)
		plane_q = plane_q * xrotn_q;
		break;
	}

			// Plane 1

	case 'i': // Forward translation of the plane (+Z translation)
	{
		gmtl::Quatf v_q = plane1_q * gmtl::Quatf(PLANE_FORWARD[0], PLANE_FORWARD[1], PLANE_FORWARD[2], 0) * gmtl::makeConj(plane1_q);
		plane1_p = plane1_p + v_q.mData;
		break;
	}
	case 'k': // Backward translation of the plane (-Z translation)
	{
		gmtl::Quatf v_q = plane1_q * gmtl::Quatf(-PLANE_FORWARD[0], -PLANE_FORWARD[1], -PLANE_FORWARD[2], 0) * gmtl::makeConj(plane1_q);
		plane1_p = plane1_p + v_q.mData;
		break;
	}
	case 'j':  // Yaws the plane (+Y rot)
	{
		plane1_q = plane1_q * yrotp_q;
		break;
	}
	case 'l':  // Yaws the plane (-Y rot)
	{
		plane1_q = plane1_q * yrotn_q;
		break;
	}
	case 'u': // Rolls the plane (+Z rot)
	{
		plane1_q = plane1_q * zrotp_q;
		break;
	}
	case 'o': // Rolls the plane (-Z rot)
	{
		plane1_q = plane1_q * zrotn_q;
		break;
	}
	case 'p': // Pitches the plane (+X rot)
	{
		plane1_q = plane1_q * xrotp_q;
		break;
	}
	case ',': { // Pitches the plane (-X rot)
		plane1_q = plane1_q * xrotn_q;
		break;
	}

//|____________________________________________________________________
//|
//| Propeller controls (subpart)
//|____________________________________________________________________

	case 'h': // Rotates propeller subpart C (+Pos value)
		pp_angle += PROPELLER_ROTATION;
		if (pp_angle >= 30.0f)
		{
			pp_angle = 30.0f;
		}
		break;
	case 't': // Rotates propeller subpart B (+Pos value)
		pp_angle_2 += PROPELLER_ROTATION;
		if (pp_angle_2 >= 30.0f)
		{
			pp_angle_2 = 30.0f;
		}
		break;
	case 'g': // Rotates propeller subpart C (-Pos value)
		pp_angle -= PROPELLER_ROTATION;
		if (pp_angle <= -30.0f)
		{
			pp_angle = -30.0f;
		}
		break;
	case 'r': // Rotates propeller subpart B (-Pos value)
		pp_angle_2 -= PROPELLER_ROTATION;
		if (pp_angle_2 <= -30.0f)
		{
			pp_angle_2 = -30.0f;
		}
		break;

	case 'c': // Gun Control subpart A (+Pos value)
		platform += PROPELLER_ROTATION;
		if (platform >= 90.0f)
		{
			platform = 90.0f;
		}
		break;
	case 'n': // Gun Control subpart A (-Pos value)
		platform -= PROPELLER_ROTATION;
		if (platform <= -90.0f)
		{
			platform = -90.0f;
		}
		break;

		//|____________________________________________________________________
		//|
		//| Lighting controls
		//|____________________________________________________________________

	case '1': // Light up (+Y translation)
		light_pos[1]++;
		printf("Light-Y = %.2f\n", light_pos[1]);
		break;
	case '3': // Light down (-Y translation)
		light_pos[1]--;
		printf("Light-Y = %.2f\n", light_pos[1]);
		break;
	case '8':
		light_pos[2]++;
		break;
	case '2':
		light_pos[2]--;
		break;
	case '4':
		light_pos[0]--;
		break;
	case '6':
		light_pos[0]++;
		break;
	case '9': // Toggles diffuse light ON/OFF
		is_diffuse_on = !is_diffuse_on;
		printf("Light-diffuse = %s\n", is_diffuse_on ? "ON" : "OFF");
		break;
	case '5':
		is_specular_on = !is_specular_on;
		printf("Light-Specular = %s\n", is_specular_on ? "ON" : "OFF");
		break;
	case '7':
		is_ambient_on = !is_ambient_on;
		printf("Light-Ambient = %s\n", is_ambient_on ? "ON" : "OFF");
		break;
	}

	glutPostRedisplay();                    // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MouseFunc
//|
//! \param button     [in] one of GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, or GLUT_RIGHT_BUTTON.
//! \param state      [in] one of GLUT_UP (event is due to release) or GLUT_DOWN (press).
//! \param x          [in] X-coordinate of mouse when an event occured.
//! \param y          [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT mouse-callback function: called for each mouse click.
//|____________________________________________________________________

void MouseFunc(int button, int state, int x, int y)
{
	int km_state;

	// Updates button's sate and mouse coordinates
	if (state == GLUT_DOWN) {
		mbuttons[button] = true;
		mx_prev = x;
		my_prev = y;
	}
	else {
		mbuttons[button] = false;
	}

	// Updates keyboard modifiers
	km_state = glutGetModifiers();
	kmodifiers[KM_SHIFT] = km_state & GLUT_ACTIVE_SHIFT ? true : false;
	kmodifiers[KM_CTRL] = km_state & GLUT_ACTIVE_CTRL ? true : false;
	kmodifiers[KM_ALT] = km_state & GLUT_ACTIVE_ALT ? true : false;

	//glutPostRedisplay();      // Asks GLUT to redraw the screen
}

//|____________________________________________________________________
//|
//| Function: MotionFunc
//|
//! \param x      [in] X-coordinate of mouse when an event occured.
//! \param y      [in] Y-coordinate of mouse when an event occured.
//! \return None.
//!
//! GLUT motion-callback function: called for each mouse motion.
//|____________________________________________________________________

void MotionFunc(int x, int y)
{
	int dx, dy, d;

	if (mbuttons[GLUT_LEFT_BUTTON] || mbuttons[GLUT_RIGHT_BUTTON]) {
		// Computes distances the mouse has moved
		dx = x - mx_prev;
		dy = y - my_prev;

		// Updates mouse coordinates
		mx_prev = x;
		my_prev = y;

		// Hold left button to rotate camera
		if (mbuttons[GLUT_LEFT_BUTTON]) {
			if (!kmodifiers[KM_CTRL]) {
				elevation[camctrl_id] += dy;            // Elevation update
			}
			if (!kmodifiers[KM_SHIFT]) {
				azimuth[camctrl_id] += dx;             // Azimuth update
			}
		}

		// Hold right button to zoom
		if (mbuttons[GLUT_RIGHT_BUTTON]) {
			if (abs(dx) >= abs(dy)) {
				d = dx;
			}
			else {
				d = -dy;
			}
			distance[camctrl_id] += d;
		}

		glutPostRedisplay();      // Asks GLUT to redraw the screen
	}
}

//|____________________________________________________________________
//|
//| Function: ReshapeFunc
//|
//! \param None.
//! \return None.
//!
//! GLUT reshape callback function: called everytime the window is resized.
//|____________________________________________________________________

void ReshapeFunc(int w, int h)
{
	// Track the current window dimensions
	w_width = w;
	w_height = h;
	glViewport(0, 0, (GLsizei)w_width, (GLsizei)w_height);
}

//|____________________________________________________________________
//|
//| Function: DrawCoordinateFrame
//|
//! \param l      [in] length of the three axes.
//! \return None.
//!
//! Draws coordinate frame consisting of the three principal axes.
//|____________________________________________________________________

void DrawCoordinateFrame(const float l)
{
	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);
	// X axis is red
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(l, 0.0f, 0.0f);

	// Y axis is green
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, l, 0.0f);

	// Z axis is blue
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, l);
	glEnd();

	glEnable(GL_LIGHTING);
}

void drawCylinder(const float radius, const float height) {
	int a;
	float x, y, xp, yp, mag;
	bool bright_col = true;

	xp = radius;
	yp = 0;
	glBegin(GL_QUADS);
	for (a = 1; a <= 360; a++) {
		x = radius * cos(gmtl::Math::deg2Rad((float)a));
		y = radius * sin(gmtl::Math::deg2Rad((float)a));

		//mag = sqrt(pow(x, 2) + pow(y, 2) + pow(height / 2, 2));
		gmtl::Vec3f normal(x, y, 0.0);
		gmtl::normalize(normal);
		glNormal3f(normal[0], normal[1], normal[2]);
		glVertex3f(xp, yp, height);
		glVertex3f(xp, yp, 0);
		glVertex3f(x, y, 0);
		glVertex3f(x, y, height);

		xp = x;
		yp = y;
	}
	glEnd();
}

void DrawPlaneBody(const float width, const float length, const float height)
{
	float w = width / 2;
	float l = length / 2;

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	//Plane Core
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKGRAY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, GRAY_COL);
	glColor3ub(126, 198, 204);
	GLUquadric* sphere = gluNewQuadric();
	drawCylinder(0.5f, (height * 2) / 3);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARKGRAY_COL);
	gluSphere(sphere, 0.5f, 360, 360);
	glPushMatrix();
	glTranslatef(0, 0, (height * 2) / 3);
	gluSphere(sphere, 0.5f, 360, 360);
	glPopMatrix();
}

void DrawLeftWing(const float width, const float length, const float height)
{
	float w = width / 2;
	float l = length / 2;

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	float mag = sqrt(pow(1.0f, 2) + pow(3.0f, 2) + pow(0.0f, 2));
	float mag1 = sqrt(pow(2.0f, 2) + pow(0.0f, 2) + pow(-3.0f, 2));
	float mag2 = sqrt(pow(4.5f, 2) + pow(0.0f, 2) + pow(3.0f, 2));
	float mag3 = sqrt(pow(0.946f, 2) + pow(2.795f, 2) + pow(0.0f, 2));
	float mag4 = sqrt(pow(2.15f, 2) + pow(6.0f, 2) + pow(0.0f, 2));
	float mag5 = sqrt(pow(0.285f, 2) + pow(0.7875f, 2) + pow(-0.0275f, 2));

	

	glBegin(GL_QUADS);
	// Left Wing (top plane)
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE_COL);
	glNormal3f(1.0f / mag, 3.0f / mag, 0.0f / mag);
	glVertex3f(0.5f, 0.0f, -0.5f);
	glVertex3f(w + 0.5f, -0.5f, 0.5f);
	glVertex3f(w + 0.5f, -0.5f, (height / 2) + 0.5f);
	glVertex3f(0.5f, 0.0f, (2 * height) + 0.5f);


	// Left Wing (bottom plane)
	glNormal3f(-1.0f / mag, -3.0f / mag, 0.0f / mag);
	glVertex3f(0.5f, -0.08f, -0.5f);
	glVertex3f(w + 0.5f, -0.58f, 0.5f);
	glVertex3f(w + 0.5f, -0.58f, (height / 2) + 0.5f);
	glVertex3f(0.5f, -0.08f, (2 * height) + 0.5f);

	//Connected parts between top plane and bottom plane
	glColor3ub(54, 208, 219);
	glNormal3f(2.0f / mag1, 0.0f / mag1, -3.0f / mag1);
	glVertex3f(0.5f, 0.0f, -0.5f);
	glVertex3f(w + 0.5f, -0.5f, 0.5f);
	glVertex3f(w + 0.5f, -0.58f, 0.5f);
	glVertex3f(0.5f, -0.08f, -0.5f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(w + 0.5f, -0.5f, 0.5f);
	glVertex3f(w + 0.5f, -0.5f, (height / 2) + 0.5f);
	glVertex3f(w + 0.5f, -0.58f, (height / 2) + 0.5f);
	glVertex3f(w + 0.5f, -0.58f, 0.5f);

	glNormal3f(4.5f / mag2, 0.0f / mag2, 3.0f / mag2);
	glVertex3f(w + 0.5f, -0.5f, (height / 2) + 0.5f);
	glVertex3f(0.5f, 0.0f, (2 * height) + 0.5f);
	glVertex3f(0.5f, -0.08f, (2 * height) + 0.5f);
	glVertex3f(w + 0.5f, -0.58f, (height / 2) + 0.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(0.5f, 0.0f, -0.5f);
	glVertex3f(0.5f, 0.0f, (2 * height) + 0.5f);
	glVertex3f(0.5f, -0.08f, (2 * height) + 0.5f);
	glVertex3f(0.5f, -0.08f, -0.5f);
	glEnd();

	// Left Wing details
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, plane_wing);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE_COL);
	glBegin(GL_TRIANGLES);

	//Piece1
	glNormal3f(0.946f / mag3, 2.795f / mag3, 0.0f / mag3);

	glTexCoord2f(0, 0);
	glVertex3f(0.6f, -0.011f, (2 * height) + 0.2f);

	glTexCoord2f(0, 0.5);
	glVertex3f(w + 0.4f, -0.451f, (height / 2) + 0.5f);

	glTexCoord2f(0.5, 0.5);
	glVertex3f(0.6f, -0.011f, (height / 2) + 0.3f);
	glEnd();

	//Piece2
	glBegin(GL_QUADS);
	glNormal3f(2.15f / mag4, 6.0f / mag4, 0.0f / mag4);

	glTexCoord2f(0, 0.15f);
	glVertex3f(0.7f, -0.021f, (height / 2) + 0.2f);

	glTexCoord2f(0.5f, 0.25f);
	glVertex3f(w + 0.4f, -0.451f, (height / 2) + 0.4f);

	glTexCoord2f(0.5f, 0);
	glVertex3f(w + 0.4f, -0.451f, 0.55f);

	glTexCoord2f(0, 0);
	glVertex3f(0.7f, -0.021f, 0.55f);


	//Piece3
	glNormal3f(0.285f / mag5, 0.7875f / mag5, -0.0275f / mag5);

	glTexCoord2f(0.1f, 0.1f);
	glVertex3f(0.7f, -0.021f, 0.45f);

	glTexCoord2f(0.5, 0.2f);
	glVertex3f(w + 0.25f, -0.401f, 0.45f);

	glTexCoord2f(0, 0);
	glVertex3f(0.6f, -0.011f, -0.3f);

	glTexCoord2f(0, 0.15f);
	glVertex3f(0.6f, -0.011f, 0.4f);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

void DrawRightWing(const float width, const float length, const float height)
{
	float w = width / 2;
	float l = length / 2;
	float mag = sqrt(pow(1.0f, 2) + pow(-3.0f, 2) + pow(0.0f, 2));
	float mag1 = sqrt(pow(-2.0f, 2) + pow(0.0f, 2) + pow(-3.0f, 2));
	float mag2 = sqrt(pow(-4.5f, 2) + pow(0.0f, 2) + pow(3.0f, 2));
	float mag3 = sqrt(pow(-0.946f, 2) + pow(2.795f, 2) + pow(0.0f, 2));
	float mag4 = sqrt(pow(-0.43f, 2) + pow(1.2f, 2) + pow(0.0f, 2));
	float mag5 = sqrt(pow(-0.39f, 2) + pow(1.15f, 2) + pow(0.0f, 2));

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	glBegin(GL_QUADS);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE_COL);
	// Right Wing (top plane)
	glColor3ub(35, 80, 119);
	glNormal3f(-1.0f / mag, 3.0f / mag, 0.0f / mag);
	glVertex3f(-0.5f, 0.0f, (2 * height) + 0.5f);
	glVertex3f(-(w + 0.5f), -0.5f, (height / 2) + 0.5f);
	glVertex3f(-(w + 0.5f), -0.5f, 0.5f);
	glVertex3f(-0.5f, 0.0f, -0.5f);

	// Right Wing (bottom plane)
	glNormal3f(1.0f / mag, -3.0f / mag, 0.0f / mag);
	glVertex3f(-0.5f, -0.08f, (2 * height) + 0.5f);
	glVertex3f(-(w + 0.5f), -0.58f, (height / 2) + 0.5f);
	glVertex3f(-(w + 0.5f), -0.58f, 0.5f);
	glVertex3f(-0.5f, -0.08f, -0.5f);


	//Connected parts between top plane and bottom plane
	glColor3ub(54, 208, 219);
	glNormal3f(-2.0f / mag1, 0.0f / mag1, -3.0f / mag1);
	glVertex3f(-0.5f, -0.08f, -0.5f);
	glVertex3f(-(w + 0.5f), -0.58f, 0.5f);
	glVertex3f(-(w + 0.5f), -0.5f, 0.5f);
	glVertex3f(-0.5f, 0.0f, -0.5f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-(w + 0.5f), -0.58f, 0.5f);
	glVertex3f(-(w + 0.5f), -0.58f, (height / 2) + 0.5f);
	glVertex3f(-(w + 0.5f), -0.5f, (height / 2) + 0.5f);
	glVertex3f(-(w + 0.5f), -0.5f, 0.5f);

	glNormal3f(-4.5f / mag2, 0.0f / mag2, 3.0f / mag2);
	glVertex3f(-(w + 0.5f), -0.58f, (height / 2) + 0.5f);
	glVertex3f(-0.5f, -0.08f, (2 * height) + 0.5f);
	glVertex3f(-0.5f, 0.0f, (2 * height) + 0.5f);
	glVertex3f(-(w + 0.5f), -0.5f, (height / 2) + 0.5f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-0.5f, -0.08f, -0.5f);
	glVertex3f(-0.5f, -0.08f, (2 * height) + 0.5f);
	glVertex3f(-0.5f, 0.0f, (2 * height) + 0.5f);
	glVertex3f(-0.5f, 0.0f, -0.5f);
	glEnd();

	// Right Wing details
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, plane_wing);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, GRAY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, MEDIUMWHITE_COL);
	glBegin(GL_TRIANGLES);
	
	//Piece 1
	glNormal3f(-0.946f / mag3, 2.795f / mag3, 0.0f / mag3);

	glTexCoord2f(0.5, 0.5);
	glVertex3f(-0.6f, -0.011f, (height / 2) + 0.3f);

	glTexCoord2f(0, 0.5);
	glVertex3f(-(w + 0.4f), -0.451f, (height / 2) + 0.5f);

	glTexCoord2f(0, 0);
	glVertex3f(-0.6f, -0.011f, (2 * height) + 0.2f);

	glEnd();

	//Piece 2
	glBegin(GL_QUADS);
	glNormal3f(-0.43f / mag4, 1.2f / mag4, 0.0f / mag4);

	glTexCoord2f(0, 0.15f);
	glVertex3f(-0.7f, -0.021f, 0.55f);

	glTexCoord2f(0.5f, 0.25f);
	glVertex3f(-(w + 0.4f), -0.451f, 0.55f);

	glTexCoord2f(0.5f, 0);
	glVertex3f(-(w + 0.4f), -0.451f, (height / 2) + 0.4f);

	glTexCoord2f(0, 0);
	glVertex3f(-0.7f, -0.021f, (height / 2) + 0.2f);

	//Piece 3
	glNormal3f(-0.39f / mag5, 1.15f / mag5, 0.0f / mag5);

	glTexCoord2f(0.5f, 0.15f);
	glVertex3f(-0.6f, -0.011f, 0.4f);

	glTexCoord2f(0.5, 0.25f);
	glVertex3f(-0.6f, -0.011f, -0.3f);

	glTexCoord2f(0, 0);
	glVertex3f(-(w + 0.25f), -0.401f, 0.45f);

	glTexCoord2f(0, 0.15f);
	glVertex3f(-0.7f, -0.021f, 0.45f);
	glEnd();

	glDisable(GL_TEXTURE_2D);
}

void DrawPlatform(const float width, const float length, const float height)
{
	float w = width / 2;
	float l = length / 2;

	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	//Platform
	glBegin(GL_QUADS);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARK_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, DARKGRAY_COL);
	glColor3ub(113, 195, 132);
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-0.125f, -0.5f, 0.0f);
	glVertex3f(0.125f, -0.5f, 0.0f);
	glVertex3f(0.125f, -0.5f, -0.3f);
	glVertex3f(-0.125f, -0.5f, -0.3f);

	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(-0.125f, -0.55f, -0.3f);
	glVertex3f(0.125f, -0.55f, -0.3f);
	glVertex3f(0.125f, -0.55f, 0.0f);
	glVertex3f(-0.125f, -0.55f, 0.0f);

	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(-0.125f, -0.5f, 0.0f);
	glVertex3f(-0.125f, -0.5f, -0.3f);
	glVertex3f(-0.125f, -0.55f, -0.3f);
	glVertex3f(-0.125f, -0.55f, 0.0f);

	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.125f, -0.55f, 0.0f);
	glVertex3f(0.125f, -0.55f, -0.3f);
	glVertex3f(0.125f, -0.5f, -0.3f);
	glVertex3f(0.125f, -0.5f, 0.0f);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(-0.125f, -0.55f, 0.0f);
	glVertex3f(0.125f, -0.55f, 0.0f);
	glVertex3f(0.125f, -0.5f, 0.0f);
	glVertex3f(-0.125f, -0.5f, 0.0f);

	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.125f, -0.55f, -0.3f);
	glVertex3f(-0.125f, -0.55f, -0.3f);
	glVertex3f(-0.125f, -0.5f, -0.3f);
	glVertex3f(0.125f, -0.5f, -0.3f);
	glEnd();
}

void DrawGunBarrel(const float radius, const float height)
{
	// Sets materials
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, DARKGRAY_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, GRAY_COL);
	//Gun Barrel
	glColor3ub(0, 0, 0);
	drawCylinder(radius, height);
}

void drawCamera(const float radius, const float height) {
	glDisable(GL_LIGHTING);
	glColor3ub(237, 176, 102);
	glutSolidCone(radius, height, 10, 10);
	glEnable(GL_LIGHTING);
}

void drawLight(const float radius) {
	glDisable(GL_LIGHTING);
	glColor3f(1.0f, 1.0f, 1.0f);
	glutSolidSphere(radius, 20, 20);
	glEnable(GL_LIGHTING);
}

void drawSatelliteLeftWing(const float width, const float height, const float offset)
{
	//Enable the texture of satellite wing
	glEnable(GL_TEXTURE_2D);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE_COL);

	glBindTexture(GL_TEXTURE_2D, satellite_wing);
	glBegin(GL_QUADS);

	glNormal3f(0, 1, 0);
	glTexCoord2f(1, 1);
	glVertex3f(1.5f, 0, height + offset);
	glTexCoord2f(0, 1);
	glVertex3f(width, 0, height + offset);
	glTexCoord2f(0, 0);
	glVertex3f(width, 0, offset);
	glTexCoord2f(1, 0);
	glVertex3f(1.5f, 0, offset);

	glEnd();
	glDisable(GL_TEXTURE_2D);
}
void drawSatelliteRightWing(const float width, const float height, const float offset)
{
	//Enable the texture of satellite wing
	glEnable(GL_TEXTURE_2D);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE_COL);

	glBindTexture(GL_TEXTURE_2D, satellite_wing);
	glBegin(GL_QUADS);

	glNormal3f(0, 1, 0);
	glTexCoord2f(1, 1);
	glVertex3f(-1.5f, 0, offset);
	glTexCoord2f(0, 1);
	glVertex3f(-width, 0, offset);
	glTexCoord2f(0, 0);
	glVertex3f(-width, 0, height + offset);
	glTexCoord2f(1, 0);
	glVertex3f(-1.5f, 0, height + offset);

	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void drawSatellite() {
	//Enable Two-Side Lighting because we can see inside of the satellite and the wing is planar
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, PASTELDARKER_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, PASTEL_COL);
	drawCylinder(1.5f, 10.0f);
	drawSatelliteLeftWing(12.0f, 5.0f, 2.5f);
	drawSatelliteRightWing(12.0f, 5.0f, 2.5f);
	DrawCoordinateFrame(2);
	glPopMatrix();

	//Disable Two-Side Lighting
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
}


//|____________________________________________________________________
//|
//| Function: CreateSphere
//|
//! \param R      Radius
//! \param H      
//! \param K     
//! \param L     
//! \return None.
//!
//! Compute the vertices of the sphere and store it to [SphereVertices]
//! Reference: http://www.codeincodeblock.com/2011/06/texture-map-in-solid-sphere-using.html
//|____________________________________________________________________
void CreateSphere(double R, double H, double K, double Z) {
	int n;
	double a;
	double b;
	const double PI = gmtl::Math::PI;
	n = 0;
	for (b = 0; b <= 90 - space; b += space) {
		for (a = 0; a <= 360 - space; a += space)
		{
			SphereVertices[n].X = R * sin((a) / 180 * PI) * sin((b) / 180 * PI) - H;
			SphereVertices[n].Y = R * cos((a) / 180 * PI) * sin((b) / 180 * PI) + K;
			SphereVertices[n].Z = R * cos((b) / 180 * PI) - Z;
			SphereVertices[n].V = (2 * b) / 360;
			SphereVertices[n].U = (a) / 360;

			n++;
			SphereVertices[n].X = R * sin((a) / 180 * PI) * sin((b + space) / 180 * PI) - H;
			SphereVertices[n].Y = R * cos((a) / 180 * PI) * sin((b + space) / 180 * PI) + K;
			SphereVertices[n].Z = R * cos((b + space) / 180 * PI) - Z;
			SphereVertices[n].V = (2 * (b + space)) / 360;
			SphereVertices[n].U = (a) / 360;
			n++;
			SphereVertices[n].X = R * sin((a + space) / 180 * PI) * sin((b) / 180 * PI) - H;
			SphereVertices[n].Y = R * cos((a + space) / 180 * PI) * sin((b) / 180 * PI) + K;
			SphereVertices[n].Z = R * cos((b) / 180 * PI) - Z;
			SphereVertices[n].V = (2 * b) / 360;
			SphereVertices[n].U = (a + space) / 360;
			n++;
			SphereVertices[n].X = R * sin((a + space) / 180 * PI) * sin((b + space) / 180 * PI) - H;
			SphereVertices[n].Y = R * cos((a + space) / 180 * PI) * sin((b + space) / 180 * PI) + K;
			SphereVertices[n].Z = R * cos((b + space) / 180 * PI) - Z;
			SphereVertices[n].V = (2 * (b + space)) / 360;
			SphereVertices[n].U = (a + space) / 360;
			n++;
		}
	}
}

//|____________________________________________________________________
//|
//| Function: DrawSphere
//|
//! \param None.
//! \return None.
//!
//! Draw the sphere with normal, material, and texture
//! Reference: http://www.codeincodeblock.com/2011/06/texture-map-in-solid-sphere-using.html
//|____________________________________________________________________
void DrawSphere() {
	glEnable(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	int b;

	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 20.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, MEDIUMWHITE_COL);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, WHITE_COL);

	glRotatef (90, 1, 0, 0);
	glBindTexture(GL_TEXTURE_2D, earth_texture);
	glBegin(GL_TRIANGLE_STRIP);

	// First 2 Point without normal because there is still no triangle at this point 
	for (b = 0; b < 2; b++) {
		glTexCoord2f(SphereVertices[b].U, SphereVertices[b].V);
		glVertex3f(SphereVertices[b].X, SphereVertices[b].Y, -SphereVertices[b].Z);
	}
	// Draw the first half of sphere
	// Adding a new point create a new triangle, so the normal vector = current_vertex - origin (which is 0,0,0)
	for (b = 2; b < VertexCount; b++)
	{
		glTexCoord2f(SphereVertices[b].U, SphereVertices[b].V);
		gmtl::Vec3f normal(SphereVertices[b].X, SphereVertices[b].Y, -SphereVertices[b].Z);
		gmtl::normalize(normal);
		glNormal3f(normal[0], normal[1], normal[2]);
		glVertex3f(SphereVertices[b].X, SphereVertices[b].Y, -SphereVertices[b].Z);
	}
	// Draw the second half of the sphere
	// Adding a new point create a new triangle, so the normal vector = current_vertex - origin (which is 0,0,0)
	for (b = 0; b < VertexCount; b++)
	{
		glTexCoord2f(SphereVertices[b].U, -SphereVertices[b].V);
		gmtl::Vec3f normal(SphereVertices[b].X, SphereVertices[b].Y, SphereVertices[b].Z);
		gmtl::normalize(normal);
		glNormal3f(normal[0], normal[1], normal[2]);
		glVertex3f(SphereVertices[b].X, SphereVertices[b].Y, SphereVertices[b].Z);
	}

	glEnd();
	glDisable(GL_TEXTURE_2D);
}

//|____________________________________________________________________
//|
//| Function: sphericalToRectangular
//|
//! \param distance      Distance of the camera from the origin.
//! \param azimuth       Azimuth of the camera. (in degs)
//! \param elevation     Elevation of the camera. (in degs)
//! \return gmtl::Point3f the x/y/z position in the rectangular coordinate system (Cartesian coordinate system)
//!
//! Convert Spherical Coordinates to Rectangular Coordinate (Cartesian coordinate system) using vector decomposition
//|____________________________________________________________________
gmtl::Point3f sphericalToRectangular(float distance, float azimuth, float elevation) {
	float theta = gmtl::Math::deg2Rad(azimuth);
	float phi = gmtl::Math::deg2Rad(-elevation);
	float x = distance * cos(phi) * sin(theta);
	float y = distance * sin(phi);
	float z = distance * cos(phi) * cos(theta);
	return gmtl::Point3f(x, y, z);
}

//|____________________________________________________________________
//|
//| Function: DrawSkybox
//|
//! \param s      [in] Skybox size.
//! \return None.
//!
//! Draws a skybox.
//|____________________________________________________________________

void DrawSkybox(const float s)
{
	float s2 = s / 2;

	// Turn on texture mapping and disable lighting
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);

	// Back wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBACK]);  // Specify which texture will be used
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(-s2, -s2, -s2);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(s2, s2, -s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(-s2, s2, -s2);
	glEnd();

	// Left wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYLEFT]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-s2, -s2, -s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-s2, s2, -s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(-s2, s2, s2);
	glEnd();

	// Bottom wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYBOTTOM]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-s2, -s2, -s2);
	glEnd();

	// Top Wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYTOP]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-s2, s2, s2);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(s2, s2, s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, s2, -s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-s2, s2, -s2);
	glEnd();

	// Front Wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYFRONT]);  // Specify which texture will be used
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-s2, -s2, s2);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, s2, s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-s2, s2, s2);
	glEnd();

	// Right Wall
	glBindTexture(GL_TEXTURE_2D, textures[TID_SKYRIGHT]);
	glBegin(GL_QUADS);
	glColor3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(s2, -s2, s2);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(s2, -s2, -s2);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(s2, s2, -s2);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(s2, s2, s2);
	glEnd();

	// Turn off texture mapping and enable lighting
	glEnable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
}

//|____________________________________________________________________
//|
//| Function: SetLight
//|
//! \param pos          [in] Light position.
//! \param is_ambient   [in] Is ambient enabled?
//! \param is_diffuse   [in] Is diffuse enabled?
//! \param is_specular  [in] Is specular enabled?
//! \return None.
//!
//! Set light properties.
//|____________________________________________________________________

void SetLight(const gmtl::Point4f& pos, const bool is_ambient, const bool is_diffuse, const bool is_specular)
{
	glLightfv(GL_LIGHT0, GL_POSITION, pos.mData);
	if (is_ambient) {
		glLightfv(GL_LIGHT0, GL_AMBIENT, AMBIENT_LIGHT);
	}
	else {
		glLightfv(GL_LIGHT0, GL_AMBIENT, NO_LIGHT);
	}
	if (is_diffuse) {
		glLightfv(GL_LIGHT0, GL_DIFFUSE, DIFFUSE_LIGHT);
	}
	else {
		glLightfv(GL_LIGHT0, GL_DIFFUSE, NO_LIGHT);
	}
	if (is_specular) {
		glLightfv(GL_LIGHT0, GL_SPECULAR, SPECULAR_LIGHT);
	}
	else {
		glLightfv(GL_LIGHT0, GL_SPECULAR, NO_LIGHT);
	}
}

//|____________________________________________________________________
//|
//| Function: LoadPPM
//|
//! \param fname       [in]  Name of file to load.
//! \param w           [out] Width of loaded image in pixels.
//! \param h           [out] Height of loaded image in pixels.
//! \param data        [in/out] Image data address (input or output depending on mallocflag).
//! \param mallocflag  [in] 1 if memory not pre-allocated, 0 if data already points to allocated memory that can hold the image.
//! \return None.
//!
//! A very minimal Portable Pixelformat image file loader. Note that if new memory is allocated, free() should be used
//! to deallocate when it is no longer needed.
//|____________________________________________________________________

void LoadPPM(const char* fname, unsigned int* w, unsigned int* h, unsigned char** data, const int mallocflag)
{
	FILE* fp;
	char P, num;
	int max;
	char s[1000];

	if (!(fp = fopen(fname, "rb")))
	{
		perror("cannot open image file\n");
		exit(0);
	}

	fscanf(fp, "%c%c\n", &P, &num);
	if ((P != 'P') || (num != '6'))
	{
		perror("unknown file format for image\n");
		exit(0);
	}

	do
	{
		fgets(s, 999, fp);
	} while (s[0] == '#');

	sscanf(s, "%d%d", w, h);
	fgets(s, 999, fp);
	sscanf(s, "%d", &max);

	printf("%s -> w: %d h: %d\n", fname, *w, *h);

	if (mallocflag)
		if (!(*data = (unsigned char*)malloc(*w * *h * 3)))
		{
			perror("cannot allocate memory for image data\n");
			exit(0);
		}

	fread(*data, 3, *w * *h, fp);

	fclose(fp);
}

//|____________________________________________________________________
//|
//| Function: main
//|
//! \param None.
//! \return None.
//!
//! Program entry point
//|____________________________________________________________________

int main(int argc, char** argv)
{
	InitTransforms();

	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);     // Uses GLUT_DOUBLE to enable double buffering
	glutInitWindowSize(w_width, w_height);

	glutCreateWindow("Plane Episode 3");

	glutDisplayFunc(DisplayFunc);
	glutKeyboardFunc(KeyboardFunc);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MotionFunc);
	glutReshapeFunc(ReshapeFunc);

	InitGL();

	glutMainLoop();

	return 0;
}