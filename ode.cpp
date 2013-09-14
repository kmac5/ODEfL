/*
======================================================================
odefl.cpp

This is "ODE for Lightwave"

(C)2008 Kevin MacPhail

ChangeLog:
See the readme.txt

============================================
Some matrix wisdom from the ODE mailing list
============================================

Functions in ODE to multiply vectors by matrices, the standard thing to do
is to rotate a vector by a matrix.

dMultiply0, dMultiply1, dMultiply2.

To rotate a vector V by a matrix M, try:

dMultiply0(Result,M,V,3,3,1);

To multiply matrices together use instead:

dMultiply0(Result,M1,M2,3,3,3);

If you want to do it while at the same time transposing the matrix, use 
dMultiply1, dMultiply2 transposes both arguments (I believe, don't 
remember well).

--

How to determine the position and orientation of the objects.

You can get position (vector) and orientation (quaternion or 3x3 matrix) of
the bodies/geoms (that's described in user's guide, section 6.2) - and use
that for rendering.

4x4 matrix is just a composition of 3x3 rotation matrix, position vector and
a column/row containing (0,0,0,1). Depending on the graphics API you use
(opengl/d3d), matrices are laid out in column-major or row-major order.

For ODE (OpenGL style), a 4x4 transformation matrix is:

Xx Yx Zx Ox
Xy Yy Zy Oy
Xz Yz Zz Oz
0  0  0  1

Here, upper-left 3x3 part is a rotation matrix (with X,Y,Z being axis
vectors, and x,y,z being their components) - you get that directly from ODE,
or convert a quaternion into it. The Ox,Oy,Oz part is the position vector.

For D3D, the matrix is transposed (flipped along main diagonal):

Xx Xy Xz 0
Yx Yy Yz 0
Zx Zy Zz 0
Ox Oy Oz 1

Thus you have to transpose the rotation matrix obtained from ODE as well
(ODE uses OpenGL-style matrices)

--
Some of my own notes:
--

ODE uses a +Z up row-major coordinate system


       Z
       |
       |
       |
       |______Y
      /
    /
  X


Lightwave uses a +Y up column-major coordinate system


       Y
       |
       |
       |
       |______Z
      /
    /
  X

So, to convert a +Z up coordinate system to a +Y up coordinate system
we need to swap the axis, i.e. swap both the Z & Y rows and columns, then 
convert from row-major to column-major.

e.g.

0 1  2    0 1  2    0  2 1    0  8 4
4 5  6 -> 8 9 10 -> 8 10 9 -> 2 10 6
8 9 10    4 5  6    4  6 5    1  9 5

So, the order to read in the ODE 3x3 rotation matrix is: 0 8 4 2 10 6 1 9 5


Trimesh Indices
===============
If you ever find yourself wondering why the trimesh indices are stored in a 1d array and not
a 2d array as in the examples, read this: http://ode.petrucci.ch/viewtopic.php?t=209&highlight=trimesh

"Turns out it was the way I was allocating the data. Once I switched indices from a
2D array to a 1D array, it worked fine.  I suppose this has to do with how ODE accesses
trimesh data - it appears that it needs to know where the next chunk of data comes from
& relies on it being some value away (i.e. sizeof(float)*3), but if a 2D array is
dynamically allocated, one dimension at a time, the chunks themselves will be sequential,
but space 3 of chunk A is not directly before space 1 of chunk B in memory."


====================================================================== */


extern "C" {
#include <lwserver.h>			// all plug-ins need this       
#include <lwgeneric.h>			// for the LayoutGeneric class  
#include <lwpanel.h>			// for "classic" panels
#include <lwhost.h>				// for the LWMessageFuncs global 
#include <lwrender.h>			// for the LWRenderFuncs global  
#include <lwcomlib.h>			// for the common library	     
#include <lwhandler.h>
#include <lwmeshes.h>
#include <lwdisplce.h>
#include <lwenvel.h>
#include <lwmath.h>
#include <com_vecmatquat.h>
#include "objectdb.h"			// object database
#include "vecmat.h"				// common math functions
}

#include <stdio.h>				// for NULL #define
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ode/ode.h>
#include "image1.h"				// for logo image


#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for ODE with VC++, no precision loss complaints
#endif


////////////////////////////////////////////////////////////////////////////////////////////
// debug
////////////////////////////////////////////////////////////////////////////////////////////
#define DEBUG 0


////////////////////////////////////////////////////////////////////////////////////////////
// some ode constants
// these may change
////////////////////////////////////////////////////////////////////////////////////////////
#define NUM 100				// max number of objects
#define MAXC 10				// max number of children
#define GPB 1				// maximum number of geometries per body (was 3)
#define MAX_CONTACTS 8		// maximum number of contact points per body (was 4)
#define SPHERE 0
#define BOX 1
#define CYLINDER 2
#define CAPSULE 3
#define TRIMESH 4			// implemented with gimpact and opcode
#define NONSTATIC 0
#define STATIC 1
#define DISABLED 0
#define ENABLED 1
#define DISABLED_UNTIL_COLLISION 2
#define NUMBER_OF_CONTROLS 4

#define PAN_WIDTH 322		// 322x105 is the size of xpanel large thumbnail
#define PAN_HEIGHT 105
#define PAN_BORDER 10


////////////////////////////////////////////////////////////////////////////////////////////
// Displacment DEFINEs
////////////////////////////////////////////////////////////////////////////////////////////
#define INF_PIVLOCAL 0
#define ODEDISPLACE_VERSION 1


////////////////////////////////////////////////////////////////////////////////////////////
// Someday these may be translated
////////////////////////////////////////////////////////////////////////////////////////////
#define STR_TYPE_TEXT      "Object Type"
#define STR_GRAVITY_TEXT   "Gravity"


////////////////////////////////////////////////////////////////////////////////////////////
// Bounding Box Data Structure
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct st_BBox {
	LWDVector bbox[11];
	LWItemID id;
} BBox;


////////////////////////////////////////////////////////////////////////////////////////////
// The objMatrixData struct
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct MyObject {
	dBodyID body;
	dGeomID geom[GPB];
} MyObject;


////////////////////////////////////////////////////////////////////////////////////////////
// Key Data
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct st_KeyData {
	LWTime	lastKeyTime;
	double	lastKeyDelta[3];
	double	lastKeyPosition[3];
	double	lastKeyVelocity[3];
} KeyData;


////////////////////////////////////////////////////////////////////////////////////////////
// For the ObjectData data structure
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct st_ObjectData{
	LWDMatrix4* m;				//Lightwave rotation matrix
	dMatrix3 rotationMatrix;	//ODE rotation matrix
	LWItemID item;
	LWItemID parent;
	LWItemID rootParent;
	dGeomID childGeoms[MAXC];	//child geometry
	dGeomID selfGeom;
	int numChildren;
	int odeNum;
	int objType;
	int objStatic;
	int objGravity;
	int objUseMU;
	double objBounciness;
	double objBounceVel;
	double objDensity;
	double objCFM;
	double objERP;
	double objMU;
	double xpos, ypos, zpos;	// the object's starting position
	struct st_ObjectData *next;
	BBox* bounds;				// contains bounds and position/rotation data
	KeyData* objKey;
	double radius;				// radius of a sphere
} ObjectData;


////////////////////////////////////////////////////////////////////////////////////////////
// Displacement Instance data
////////////////////////////////////////////////////////////////////////////////////////////
typedef struct st_ODEDisplaceData {
   LWTime		time;			// current time
   LWFrame		frame;			// current frame
   LWItemID		item;
   int			flags;			// use INF_ flag bit definitions
   int			objType;
   int			objStatic;
   int			objGravity;
   int			objUseMU;
   double		objBounciness;
   double		objBounceVel;
   double		objDensity;
   double		objCFM;
   double		objERP;
   double		objMU;
   char			desc[ 80 ];		// text description
} ODEDisplaceData;


////////////////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////////////////
void DrawLogo(LWPanelID panel, DrMode dm);
XCALL_( int ) ODEGeneric( long version, GlobalFunc *global, LWLayoutGeneric *local, void *serverData );
void ODE_event( LWControl *ctrl, LWLayoutGeneric *local);
static void nearCallback (void *data, dGeomID o1, dGeomID o2);
XCALL_( static LWError )AddObject(LWItemID id);
void RemoveObject(LWItemID id);
ObjectData* FindObject( LWItemID id);
int CalcABVector (LWItemID id_a, LWItemID id_b, LWDVector delta );
int CalcBoundingBox ( LWItemID id );
int CalcKeyData ( ObjectData* test );
static int pntScanBoundingBox( ObjectData* test, LWPntID id );
LWEnvelopeID findEnv( LWChanGroupID group, char *name );
LWItemID findRootParents(ObjectData* test);
int sortList();
void KM_transpose4 ( LWDMatrix4 n, LWDMatrix4* m, int i );
void KM_transpose34 ( dMatrix3 n, dMatrix3 m );
void KM_dmatmul34( dMatrix3 a, dMatrix3 b, dMatrix3 c );
void KM_dtransform( LWDVector a, dMatrix3 m, LWDVector b );
void KM_dinitm34( dMatrix3 m, dReal a1, dReal b1, dReal c1, dReal d1,
							  dReal a2, dReal b2, dReal c2, dReal d2,
							  dReal a3, dReal b3, dReal c3, dReal d3 );
void KM_LWMAT_getScales(LWItemInfo *iteminfo, LWItemID id, LWTime time, LWDMatrix4 M);


////////////////////////////////////////////////////////////////////////////////////////////
//For ODE_Displace
////////////////////////////////////////////////////////////////////////////////////////////
XCALL_( static int )popCnt_VMAP( void *data );
XCALL_( static const char * )popName_VMAP( void *data, int idx );
XCALL_( static int )popCnt_Item( void *data );
XCALL_( static const char * )popName_Item( void *data, int idx );
static LWItemID popItemID( int idx );
XCALL_( static LWInstance )Create( void *priv, LWItemID item, LWError *err );
XCALL_( static void )Destroy( ODEDisplaceData *dat );
XCALL_( static LWError )Copy( ODEDisplaceData *to, ODEDisplaceData *from );
XCALL_( static LWError )Load( ODEDisplaceData *dat, const LWLoadState *ls );
XCALL_( static LWError )Save( ODEDisplaceData *dat, const LWSaveState *ss );
XCALL_( static const char * )Describe( ODEDisplaceData *dat );
XCALL_( static const LWItemID * )UseItems( ODEDisplaceData *dat );
XCALL_( static void )ChangeID( ODEDisplaceData *dat, const LWItemID *ids );
XCALL_( static LWError )Init( ODEDisplaceData *dat, int mode );
XCALL_( static void )Cleanup( ODEDisplaceData *dat );
XCALL_( static LWError )NewTime( ODEDisplaceData *dat, LWFrame fr, LWTime t );
XCALL_( static int )Flags( ODEDisplaceData *dat );
XCALL_( static void )Evaluate( ODEDisplaceData *dat, LWDisplacementAccess *da );
XCALL_( int )ODEDisplace( long version, GlobalFunc *global, LWDisplacementHandler *local, void *serverData);
XCALL_( static void )ChangeNotify( LWXPanelID panID, unsigned long cid, unsigned long vid, int event_code );
XCALL_( static void )DrawXpLogo(LWXPanelID pan, unsigned long cid, LWXPDrAreaID reg, int w, int h);
ODEDisplace_UI( long version, GlobalFunc *global, LWInterface *local, void *serverData );


////////////////////////////////////////////////////////////////////////////////////////////
// ODEfL globals
////////////////////////////////////////////////////////////////////////////////////////////
LWMessageFuncs *msg;			/* the message functions         */
LWPanelFuncs *panf;				/* the panel functions           */
LWSceneInfo *lwsi;				/* the scene functions           */
LWInterfaceInfo *intinfo;		/* the interface functions       */
LWPanelID panel;				/* the panel                     */
LWPanControlDesc desc;			/* used by macros in lwpanel.h   */
LWChannelInfo *chinfo;
static LWObjectFuncs   *objf     = NULL;
static LWEnvelopeFuncs *envf     = NULL;
static LWChannelInfo   *chanf    = NULL;
static LWItemInfo      *iteminfo = NULL;
static LWObjectInfo    *objinfo;
static LWSceneInfo    *scninfo = NULL;
static LWValue
   ival    = { LWT_INTEGER },
   ivecval = { LWT_VINT    },
   fval    = { LWT_FLOAT   },
   fvecval = { LWT_VFLOAT  },
   sval    = { LWT_STRING  };
static LWInstUpdate    *lwupdate = NULL;
static LWXPanelFuncs   *xpanf    = NULL;


////////////////////////////////////////////////////////////////////////////////////////////
// XPanels Stuff for ODE_Displace
////////////////////////////////////////////////////////////////////////////////////////////
enum { CH_LOGO = 0x8001, CH_TYPE, CH_STATIC, CH_GRAVITY, CH_BOUNCINESS, CH_BOUNCEVEL, CH_DENSITY, CH_CFM, CH_ERP, CH_MU_ENABLE, CH_MU };


////////////////////////////////////////////////////////////////////////////////////////////
//A global pointer to the array to hold the per-frame translation data for
//a single object. Will be read by the Displacement plugin included in this file.
////////////////////////////////////////////////////////////////////////////////////////////
LWControl *ctl[ NUMBER_OF_CONTROLS ];
ObjectData* objMatrixData = NULL;
float gravity[3] = {0.0,-9.81,0.0};
int stepSizeMultiplier = 10;
int groundPlane = ENABLED;

////////////////////////////////////////////////////////////////////////////////////////////
// ODEfL Global
// dynamics and collision objects
////////////////////////////////////////////////////////////////////////////////////////////
static int num=0;					// number of objects in simulation
static int nextobj=0;				// next object to recycle if num==NUM
static dWorldID world;
static dSpaceID space;
static MyObject obj[NUM];			// Max Number of possible objects
static dJointGroupID contactgroup;
//static int selected = -1;			// selected object
//static int show_aabb = 0;			// show geom AABBs?
//static int show_contacts = 0;		// show contact points?
//static int random_pos = 1;			// drop objects from random position?
static int write_world = 0;
static dGeomID TriMesh;				//Trimesh tests


/*
======================================================================
DrawLogo()

Courtesy Nik Lever, also thanks for his tga->.h converter
Draws the plugin graphic.
====================================================================== */

void DrawLogo(LWPanelID panel, DrMode dm)
{
    int x=0;
	int y=0;
	int i=0;
	int j=0;
	int index=0;
	int w=0;

	DrawFuncs *df;

	// Get panel width
    panf->get(panel,PAN_W,&w);
	df=panf->drawFuncs;
    index = 0;        

	/////////////////////////////////////////////////////////////////////
	//Depending on which app created the image used to generate image1.h,
	//we may need to flip the order in which we write y.
	/////////////////////////////////////////////////////////////////////
	//y = IMAGE1HEIGHT + PAN_BORDER;
	y = PAN_BORDER;
	for(j=0;j<IMAGE1HEIGHT;j++){
        x = (w-IMAGE1WIDTH)/2;
        for(i=0;i<IMAGE1WIDTH;i++){
            df->drawRGBPixel(panel,image1_data[index],
                 image1_data[index+1],image1_data[index+2],x++,y);       
			index+=3;
        }
		//y--;
		y++;
	}

	//Draw logo border lines
	df->drawLine(panel,COLOR_DK_GREY,PAN_BORDER,PAN_BORDER,PAN_WIDTH+PAN_BORDER,PAN_BORDER);
	df->drawLine(panel,COLOR_DK_GREY,PAN_BORDER,PAN_BORDER,PAN_BORDER,PAN_HEIGHT+PAN_BORDER);
	df->drawLine(panel,LWP_HILIGHT,PAN_WIDTH+PAN_BORDER,PAN_BORDER,PAN_WIDTH+PAN_BORDER,PAN_HEIGHT+PAN_BORDER);	
	df->drawLine(panel,LWP_HILIGHT,PAN_BORDER,PAN_HEIGHT+PAN_BORDER,PAN_WIDTH+PAN_BORDER,PAN_HEIGHT+PAN_BORDER);
}


/*
======================================================================
get_panel()

Create the main panel.
====================================================================== */

static LWPanelID get_panel( LWLayoutGeneric *local )
{
   LWPanControlDesc desc;
   LWPanelID panel;
   LWValue ival = { LWT_INTEGER };
   int i, w;

   if( !( panel = PAN_CREATE( panf, "ODEfL v0.4.3 BETA" )))
      return NULL;

   PAN_SETW( panf, panel, PAN_WIDTH + 20);
  
   ctl[ 0 ] = BUTTON_CTL( panf, panel, "Calculate This!" );
   MOVE_CON(ctl[0],10,PAN_HEIGHT+10);
   CON_SETEVENT( ctl[ 0 ], ODE_event, local);

   ctl[ 1 ] = FVEC_CTL( panf, panel, "Gravity");
   SET_FVEC(ctl[ 1 ], gravity[0], gravity[1], gravity[2]);

   ctl[ 2 ] = INT_CTL( panf, panel, "Step Size Multipier");
   SET_INT(ctl[ 2 ], stepSizeMultiplier);

   ctl[ 3 ] = BOOL_CTL( panf, panel, "Enable Ground Plane");
   SET_INT(ctl[ 3 ], groundPlane);


   //////////////////
   // align controls
   //////////////////
   for ( i = 1; i < NUMBER_OF_CONTROLS; i++ ) {
      w = CON_LW( ctl[ i ] );
      ival.intv.value = 100 - w;
      ctl[ i ]->set( ctl[ i ], CTL_X, &ival );
   }

   return panel;
}


/*
======================================================================
ODEGeneric()

The ODE activation function.
====================================================================== */

XCALL_( int )
ODEGeneric( long version, GlobalFunc *global, LWLayoutGeneric *local, void *serverData )
{
	int ok;
	ObjectData* test = objMatrixData;


   if ( version != LWLAYOUTGENERIC_VERSION )
      return AFUNC_BADVERSION;


   ///////////////////////////////////////////////
   // get the message, panels and other functions
   ///////////////////////////////////////////////
   msg		= (struct st_LWMessageFuncs *)	global( LWMESSAGEFUNCS_GLOBAL, GFUSE_TRANSIENT );
   panf		= (struct st_LWPanelFuncs *)	global( LWPANELFUNCS_GLOBAL, GFUSE_TRANSIENT );
   lwsi		= (struct st_LWSceneInfo *)		global( LWSCENEINFO_GLOBAL, GFUSE_TRANSIENT );
   intinfo	= (struct st_LWInterfaceInfo *)	global( LWINTERFACEINFO_GLOBAL, GFUSE_TRANSIENT );
   objinfo	= (struct st_LWObjectInfo *)	global( LWOBJECTINFO_GLOBAL, GFUSE_TRANSIENT );
   envf		= (struct st_LWEnvelopeFuncs *)	global( LWENVELOPEFUNCS_GLOBAL, GFUSE_TRANSIENT );
   chinfo	= (struct st_LWChannelInfo  *)	global( LWCHANNELINFO_GLOBAL, GFUSE_TRANSIENT );

   if ( !msg || !panf || !lwsi || !intinfo || !objinfo || !envf || !chinfo)
      return AFUNC_BADGLOBAL;


   //////////////////////////////////////////////////////
   // initialize the panels functions and create a panel
   //////////////////////////////////////////////////////
   panf->globalFun = global;

   panel = get_panel(local);
   if ( !panel ) return AFUNC_BADGLOBAL;

   //Draw logo defined in image1.h
   ( *panf->set )( panel, PAN_USERDRAW, DrawLogo );

   ok = panf->open( panel, PANF_BLOCKING );

   if ( !ok ) {
	  return AFUNC_OK; // do nothing and quit
   }

   GET_FVEC( ctl[ 1 ], gravity[0], gravity[1], gravity[2] );
   GET_INT ( ctl[ 2 ], stepSizeMultiplier );
   GET_INT ( ctl[ 3 ], groundPlane );

   //////////////////
   // free the panel
   //////////////////
   PAN_KILL( panf, panel );


   /////////
   // done!
   /////////
   return AFUNC_OK;

}


/*
======================================================================
ODE_event()

Button callback.

This is the callback that is activated when the "Calculate" button is
pushed.

====================================================================== */

void ODE_event( LWControl *ctrl, LWLayoutGeneric *local)
{
	////////////////////////////////////////////
	// Start the ODE calculations
	// This is where the heavy lifing gets done
	////////////////////////////////////////////
	LWDVector objPos;
	LWDVector objDelta;
	LWDVector objDeltaRotated;
	dMatrix3 invRotMatrix;
	ObjectData* test = objMatrixData;
	ObjectData* rp;	
	ObjectDB *odb;
	dMass mass;
	int lastKeyFrame;
	int i, j; // current frame - these are used in several places not as frames - clean it up
	int frameStart, frameEnd, framesPerSecond;
	int totalFrames;
	int num = 0;odefl.cpp
	int numC = 0;
	int count = 0;
	int currentVertex = 0;
	int vertexLoopOuter = 0;
	int vertexLoopInner = 0;
	int polyLoop = 0;
	int indexLoopOuter = 0;
	int indexLoopInner = 0;
	double stepSize;
	double lx, ly, lz;
	char buf[80];
	const dReal *R;		//pointer to a 4X4 matrix
	const dReal *pos;
	static dTriMeshDataID TriData;
	//int groundPlane = ENABLED;

	//dJointID foo;
	//dJointID foo1;
	//dJointID foo2;
	//dJointID foo3;

	//Create and init poly data
	float* vertices = (float*)malloc(sizeof(float));
	int* indices = (int*)malloc(sizeof(int));


	//////////////////////////////////////////////////////////////////
	//Get the scene frame info and calculate stepSize and totalFrames
	//////////////////////////////////////////////////////////////////
	frameStart = intinfo->previewStart;
	frameEnd = intinfo->previewEnd;
	framesPerSecond = lwsi->framesPerSecond;
	stepSize = 1.0/framesPerSecond;
	totalFrames = (frameEnd-frameStart)+1;


	//////////////////////////////////////////////
	// no objects added to the simulation so quit
	//////////////////////////////////////////////
	if (test == NULL) return;


	/////////////////////////////
	// Some Debugging statements
	/////////////////////////////
	if (DEBUG) {
		sprintf (buf, "The frameStart is %d", frameStart);
		msg->info( buf, NULL );
		sprintf(buf, "The stepSize is %f", stepSize);
		msg->info( buf, NULL );
		sprintf(buf, "The totalFrames is %d", totalFrames);
		msg->info( buf, NULL );
	}


	//////////////////////////////////////////////////////////////////
	// Reset the starting position of each object and re-evaluate the
	// objects for changed parenting data for each re-calculation
	//////////////////////////////////////////////////////////////////
	while (test != NULL)
	{
		iteminfo->param( test->item, LWIP_W_POSITION, 0, objPos );

		test->xpos = objPos[0];
		test->ypos = objPos[1];
		test->zpos = objPos[2];

		test->parent = iteminfo->parent(test->item);
		test = test->next;
	}

	
	sprintf (buf, "GoToFrame %d", frameStart);
	local->evaluate( local->data, buf );

	GET_FVEC( ctl[1], gravity[0], gravity[1], gravity[2] );
	GET_INT ( ctl[2], stepSizeMultiplier );
	GET_INT ( ctl[3], groundPlane );
	

	////////////////////////////////////////////////
	// Create the ODE world
	////////////////////////////////////////////////

	dInitODE(); // needed for gimpact
	world = dWorldCreate();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity (world, gravity[0], gravity[2], gravity[1]);
	dWorldSetCFM (world,1e-5);
	dWorldSetAutoDisableFlag (world,1);
	dWorldSetContactMaxCorrectingVel (world,0.1);
	dWorldSetContactSurfaceLayer (world,0.001);

	if (groundPlane == ENABLED)
	{
		dCreatePlane (space,0,0,1,0);
	}
//	dCreatePlane (space,0,0,1,0); // This could be set by the user later
	memset (obj,0,sizeof(obj));

	
	/////////////////////////////////////////////////////////////////////////////
	// Find any root parents for each object in the objMatrixData Linked List,
	// and  find any key frame data.
	/////////////////////////////////////////////////////////////////////////////

	test = objMatrixData; // reset pointer to the front of the linked list of objects

	while (test != NULL)
	{
		if (test->rootParent == NULL) //test only if root parent unknown
		{
			test->rootParent = findRootParents(test);		
		}
		CalcKeyData( test ); //look for keyframes
		test = test->next;
	}	

	sortList(); //guarantee rootParents are added to ODE first


	////////////////////////////////////////////////////////////////////////
	// Add a Body and Geom for each object in the objMatrixData Linked List
	////////////////////////////////////////////////////////////////////////

	test = objMatrixData; // reset pointer to the front of the linked list of objects

	
	while (test != NULL)
	{
		if ( CalcBoundingBox( test->item ))
			return; //error

		//sprintf(buf, "%f ", test->radius);
		//msg->info( buf, NULL );

		lx = test->bounds->bbox[1][0] - test->bounds->bbox[0][0];
		ly = test->bounds->bbox[1][2] - test->bounds->bbox[0][2];
		lz = test->bounds->bbox[1][1] - test->bounds->bbox[0][1];

		//// Matrix to rotate from world to item coordinates.
		//// NOTE: This includes scale
		test->rotationMatrix[0] = test->bounds->bbox[5][0];
		test->rotationMatrix[1] = test->bounds->bbox[5][2];
		test->rotationMatrix[2] = test->bounds->bbox[5][1];
		test->rotationMatrix[3] = 0;
		test->rotationMatrix[4] = test->bounds->bbox[7][0];
		test->rotationMatrix[5] = test->bounds->bbox[7][2];
		test->rotationMatrix[6] = test->bounds->bbox[7][1];
		test->rotationMatrix[7] = 0;
		test->rotationMatrix[8] = test->bounds->bbox[6][0];
		test->rotationMatrix[9] = test->bounds->bbox[6][2];
		test->rotationMatrix[10] = test->bounds->bbox[6][1];
		test->rotationMatrix[11] = 0;


		obj[num].body = dBodyCreate (world);

		if (test->objKey->lastKeyTime > 0.0)
			dBodyDisable(obj[num].body);

		if ( test->objType == SPHERE ){
			dMassSetSphere (&mass, test->objDensity, test->radius);
			
			if (test->rootParent == test->item) // object is its own root parent
			{
				obj[num].geom[0] = dCreateSphere (space,test->radius); // assumes one geom per object
				test->odeNum = num;
				test->numChildren = 0;
			}
			else //child objects are first attached to geometry transforms
			{
				rp = FindObject( test->rootParent);
				numC = rp->numChildren;

				//create the next object
				obj[num].geom[0] = dCreateSphere (space,test->radius); // assumes one geometry per object

				//add it to the child list of the root parent
				rp->childGeoms[numC] = obj[num].geom[0],
				rp->numChildren++;
			}
			test->selfGeom = obj[num].geom[0];
		}

		if ( test->objType == BOX ){
			dMassSetBox (&mass, test->objDensity, lx, ly, lz);

			if (test->rootParent == test->item) // object is its own root parent
			{
				obj[num].geom[0] = dCreateBox (space, lx, ly, lz);
				test->odeNum = num;
				test->numChildren = 0;
			}
			else //child objects are first attached to geometry transforms
			{
				rp = FindObject( test->rootParent);
				numC = rp->numChildren;

				//create the next object
				obj[num].geom[0] = dCreateBox (space, lx, ly, lz);

				//add it to the child list of the root parent
				rp->childGeoms[numC] = obj[num].geom[0],
				rp->numChildren++;
			}
			test->selfGeom = obj[num].geom[0];
		}

		if ( test->objType == CYLINDER ){
					
			//sprintf (buf, "lx is %f, ly is %f, lz is %f", lx, ly, lz);
			//msg->info( buf, NULL );

			dMassSetCylinder (&mass, test->objDensity, 3, lx/2, lz);

			if (test->rootParent == test->item) // object is its own root parent
			{
				obj[num].geom[0] = dCreateCylinder (space, lx/2, lz);
				test->odeNum = num;
				test->numChildren = 0;
			}
			else //child objects are first attached to geometry transforms
			{
				rp = FindObject( test->rootParent);
				numC = rp->numChildren;

				//create the next object
				obj[num].geom[0] = dCreateCylinder (space, lx/2, lz);

				//add it to the child list of the root parent
				rp->childGeoms[numC] = obj[num].geom[0],
				rp->numChildren++;
			}
			test->selfGeom = obj[num].geom[0];
		}

		if ( test->objType == CAPSULE ){
					
			//sprintf (buf, "lx is %f, ly is %f, lz is %f", lx, ly, lz);
			//msg->info( buf, NULL );

			dMassSetCapsule (&mass, test->objDensity, 3, lx/2, lz-lx);

			if (test->rootParent == test->item) // object is its own root parent
			{
				obj[num].geom[0] = dCreateCapsule (space, lx/2, lz-lx);
				test->odeNum = num;
				test->numChildren = 0;
			}
			else //child objects are first attached to geometry transforms
			{
				rp = FindObject( test->rootParent);
				numC = rp->numChildren;

				//create the next object
				obj[num].geom[0] = dCreateCapsule (space, lx/2, lz-lx);

				//add it to the child list of the root parent
				rp->childGeoms[numC] = obj[num].geom[0],
				rp->numChildren++;
			}
			test->selfGeom = obj[num].geom[0];
		}

		if ( test->objType == TRIMESH ){
					
			//sprintf (buf, "lx is %f, ly is %f, lz is %f", lx, ly, lz);
			//msg->info( buf, NULL );

			// Build the objectdb
			odb = getObjectDB( test->item, panf->globalFun );
			if ( !odb ) {
				msg->error( "Couldn't allocate object database for", iteminfo->name( test->item ));
				continue;
			}

			//Init geometry counts
			int vertexCount = odb->npoints;
			int indexCount = 0;

			//Calculate number of indices as the sum of (nverts-2)*3 for all polygons
			//Allows tripling meshes on the fly rather than using "odb->npolygons * 3" for 3 sided polys only
			for ( i = 0; i < odb->npolygons; i++ )
			{
				indexCount = indexCount + ((odb->pol[ i ].nverts - 2) * 3);
			}

			//Build the vertex 1d array
			vertices = (float*)malloc(vertexCount*3*sizeof(float));

			//Build the indices 1d array
			indices = (int*)malloc(indexCount*sizeof(int));

			// Fill the vertices array from the odb
			count = 0;
			for ( vertexLoopOuter = 0; vertexLoopOuter < odb->npoints; vertexLoopOuter++ )
			{
				for ( vertexLoopInner = 0; vertexLoopInner < 3; vertexLoopInner++ )
				{
					//Swap the Z and Y axis to convert from Lightwave space to ODE, aka mirroring
					currentVertex=0;
					if (vertexLoopInner==0) {currentVertex=0;}
					if (vertexLoopInner==1) {currentVertex=2;}
					if (vertexLoopInner==2) {currentVertex=1;}
					vertices[ count ] = odb->pt[ vertexLoopOuter ].pos[ 0 ][ currentVertex ];
					count++;
				}
			}

			// Fill the indices 1d array from the odb
			count = 0;
			for ( polyLoop = 0; polyLoop < odb->npolygons; polyLoop++ )
			{
				//Given a polygon with indices [0 1 2 3 4]
				//Loop 1: [0 1 2    ]
				//Loop 2: [0   2 3  ]
				//Loop 3: [0     3 4]
				for ( indexLoopOuter = 0; indexLoopOuter < odb->pol[ polyLoop ].nverts - 2; indexLoopOuter++ )
				{
					for ( indexLoopInner = 0; indexLoopInner < 3; indexLoopInner++ )
					{
						//Reverse the winding of the polys to compensate for the mirrored points
						currentVertex=0;
						if (indexLoopInner==0) {currentVertex=0;}
						if (indexLoopInner==1) {currentVertex=2+indexLoopOuter;}
						if (indexLoopInner==2) {currentVertex=1+indexLoopOuter;}
						indices[count] = odb->pol[ polyLoop ].v[ currentVertex ].index;
						count++;
					}
				}
			}

			if (test->rootParent == test->item) // object is its own root parent
			{
				dTriMeshDataID TriData = dGeomTriMeshDataCreate();
				dGeomTriMeshDataBuildSingle(TriData, vertices, 3 * sizeof(float), vertexCount, indices, indexCount, 3 * sizeof(int));
				obj[num].geom[0] = dCreateTriMesh(space, TriData, 0, 0, 0);
				dGeomSetData(obj[num].geom[0], TriData);
				test->odeNum = num;
				test->numChildren = 0;
			}
			//else //child objects are first attached to geometry transforms
			//{
			//	rp = FindObject( test->rootParent);
			//	numC = rp->numChildren;

			//	//create the next object
			//	obj[num].geom[0] = dCreateCapsule (space, lx/2, lz-lx);

			//	//add it to the child list of the root parent
			//	rp->childGeoms[numC] = obj[num].geom[0],
			//	rp->numChildren++;
			//}

			test->selfGeom = obj[num].geom[0];
			dMassSetTrimesh(&mass, test->objDensity, test->selfGeom);
			freeObjectDB( odb );
		}

		if (test->objStatic == NONSTATIC )
		{
			if (test->parent == NULL)
			{
				dGeomSetBody(obj[num].geom[0],obj[num].body);
				dGeomSetPosition (obj[num].geom[0],test->xpos,test->zpos,test->ypos);
				dGeomSetRotation(obj[num].geom[0], test->rotationMatrix);
				if ( test->objType == TRIMESH )
				{
					dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
				}
			}
			else
			{
				// Make a transpose rotation matrix from the parents rotation matrix
				rp = FindObject( test->rootParent);

				KM_transpose34 ( invRotMatrix, rp->rotationMatrix );
				CalcABVector(test->item, test->rootParent, objDelta);
				KM_dtransform ( objDelta, invRotMatrix, objDeltaRotated);

				//Set the postion of the ODE geoms
				dGeomSetBody(obj[num].geom[0],obj[rp->odeNum].body);
				dGeomSetOffsetPosition (rp->childGeoms[numC],objDeltaRotated[0],objDeltaRotated[2],objDeltaRotated[1]);
				dGeomSetRotation(rp->childGeoms[numC], test->rotationMatrix);
			}
		}
		else //Object is static, don't attach it to a body
		{
			dGeomSetPosition (obj[num].geom[0],test->xpos,test->zpos,test->ypos);
			dGeomSetRotation(obj[num].geom[0], test->rotationMatrix);
			if ( test->objType == TRIMESH )
			{
				dMassTranslate(&mass, -mass.c[0], -mass.c[1], -mass.c[2]);
			}
		}

		// Disable gravity for selected bodies
		if ((test->objGravity == DISABLED) || (test->objGravity == DISABLED_UNTIL_COLLISION))
		{
			dBodySetGravityMode(obj[num].body,0);
		}

		//Will need to account for composite objects and their center of mass... later
		dBodySetMass (obj[num].body,&mass);

		// Clear old data from transform matrices
		if (test->m != NULL)
		{
			free(test->m);
			test->m = NULL;
		}

		// Create the new Matrix Array
		test->m = (LWDMatrix4*)malloc(totalFrames * sizeof(LWDMatrix4));

		test = test->next;
		num++;
	}

	////////////////
	// Joint Test //
	////////////////
	
	// Test #1
	//foo = dJointCreateHinge(world,0);
	//dJointAttach(foo,obj[0].body,0);
	//dJointSetHingeAnchor (foo, 0.0, 0.0, 2.0);
	//dJointSetHingeAxis (foo, 0.0, 1.0, 0.0);

	// Test #2
	//foo1 = dJointCreateHinge(world,0);
	//dJointAttach(foo1,obj[0].body,0);
	//dJointSetHingeAnchor (foo1, 0.0, 0.0, 1.5);
	//dJointSetHingeAxis (foo1, 0.0, 1.0, 0.0);

	//foo2 = dJointCreateHinge(world,0);
	//dJointAttach(foo2,obj[1].body,0);
	//dJointSetHingeAnchor (foo2, 0.0, 0.0, 3.5);
	//dJointSetHingeAxis (foo2, 0.0, 1.0, 0.0);

	//foo3 = dJointCreateHinge(world,0);
	//dJointAttach(foo3,obj[2].body,0);
	//dJointSetHingeAnchor (foo3, 0.0, 0.0, 5.5);
	//dJointSetHingeAxis (foo3, 0.0, 1.0, 0.0);
	////////////////
	// Joint Test //
	////////////////


	// reset num and test for frame 0
	num = 0;
	test = objMatrixData;


	//Set translation matrix for each object at frame 0
	while (test != NULL)
	{
		pos = dGeomGetPosition (obj[num].geom[0]);
		R = dGeomGetRotation (obj[num].geom[0]);

		//Build the transformation matrix from ODE space back to Lightwave 
		test->m[0][0][0] = R[0];
		test->m[0][0][1] = R[8];
		test->m[0][0][2] = R[4];
		test->m[0][0][3] = 0;
		test->m[0][1][0] = R[2];
		test->m[0][1][1] = R[10];
		test->m[0][1][2] = R[6];
		test->m[0][1][3] = 0;
		test->m[0][2][0] = R[1];
		test->m[0][2][1] = R[9];
		test->m[0][2][2] = R[5];
		test->m[0][2][3] = 0;
		test->m[0][3][0] = pos[0] - test->xpos;
		test->m[0][3][1] = pos[2] - test->ypos;
		test->m[0][3][2] = pos[1] - test->zpos;
		test->m[0][3][3] = 1;

		num++;
		test = test->next;

	} // end while loop

	////////////////////////////////////////////////
	// Start the simulation loop
	////////////////////////////////////////////////
	for (i=0;i<totalFrames-1;i++)
	{

		for (j=0;j<stepSizeMultiplier;j++)
		{
			//bob = stepSize/stepSizeMultiplier;
			dSpaceCollide (space,0,&nearCallback);
			dWorldQuickStep (world,stepSize/stepSizeMultiplier);
		}
		
		if (DEBUG) {
			if (write_world) {
				FILE *f = fopen ("state.dif","wt");
				if (f) {
					dWorldExportDIF (world,f,"X");
					fclose (f);
				}
				write_world = 0;
			}
		}

		/////////////////////////////////////////////////////
		// Extract the geom motion from the sim for each step
		// Need to do this for each dGeomID (g)
		/////////////////////////////////////////////////////

		// reset num and test
		num = 0;
		test = objMatrixData;

		//Check all objects in the sim for keyframes.  If there is a second keyframe on this frame
		//enable the body and 
		while (test != NULL)
		{
			lastKeyFrame = test->objKey->lastKeyTime * framesPerSecond - 1;

			if ((i == lastKeyFrame) && (i != 0))
			{
				dBodyEnable(obj[num].body);
				
				dBodySetPosition(	obj[num].body,
									test->objKey->lastKeyPosition[0],
									test->objKey->lastKeyPosition[2],
									test->objKey->lastKeyPosition[1]
								);

				dBodySetLinearVel(	obj[num].body,
									test->objKey->lastKeyVelocity[0],
									test->objKey->lastKeyVelocity[2],
									test->objKey->lastKeyVelocity[1]
								);
			}

			pos = dGeomGetPosition (obj[num].geom[0]);
			R = dGeomGetRotation (obj[num].geom[0]);

			///////////////////////////////////////////////////////////////////////////////
			// Generate the translation matrix for the current object at the current frame.
			// Here we're converting an ODE matrix into something Lightwave understands.
			///////////////////////////////////////////////////////////////////////////////
			LWMAT_didentity4(test->m[i+1]);//TESTING

			test->m[i+1][0][0] = R[0];
			test->m[i+1][0][1] = R[8];
			test->m[i+1][0][2] = R[4];
			test->m[i+1][0][3] = 0;
			test->m[i+1][1][0] = R[2];
			test->m[i+1][1][1] = R[10];
			test->m[i+1][1][2] = R[6];
			test->m[i+1][1][3] = 0;
			test->m[i+1][2][0] = R[1];
			test->m[i+1][2][1] = R[9];
			test->m[i+1][2][2] = R[5];
			test->m[i+1][2][3] = 0;

			if ((i >= lastKeyFrame) && (lastKeyFrame != 0)){
				test->m[i+1][3][0] = pos[0] - test->xpos - test->objKey->lastKeyDelta[0];
				test->m[i+1][3][1] = pos[2] - test->ypos - test->objKey->lastKeyDelta[1];
				test->m[i+1][3][2] = pos[1] - test->zpos - test->objKey->lastKeyDelta[2];
				test->m[i+1][3][3] = 1;
			}
			else{
				test->m[i+1][3][0] = pos[0] - test->xpos;
				test->m[i+1][3][1] = pos[2] - test->ypos;
				test->m[i+1][3][2] = pos[1] - test->zpos;
				test->m[i+1][3][3] = 1;
			}

			num++;
			test = test->next;

		} // end while loop

		dJointGroupEmpty (contactgroup);

		sprintf (buf, "RefreshNow", NULL);
		local->evaluate( local->data, buf );

        sprintf (buf, "NextFrame", NULL);
		local->evaluate( local->data, buf );

	} //end for loop

	sprintf (buf, "GoToFrame %d", frameStart);
	local->evaluate( local->data, buf );

	free(vertices);
	free(indices);

	dJointGroupDestroy (contactgroup);
	dSpaceDestroy (space);
	dWorldDestroy (world);

}


/*
======================================================================
nearCallback()

For ODE, taken from test_boxstack.cpp

This is called by dSpaceCollide when two objects in space are
potentially colliding.
====================================================================== */

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	int i;
	ObjectData* test = objMatrixData;

	dReal sumBounce = 0;
	dReal sumBounceVelocity = 0.0;
	dReal sumCFM = 0;
	dReal sumERP = 0;
	dReal sumMU = 0;
	int useMU = 0;
	dReal geomIDO1Bounce = 0.0;
	dReal geomIDO2Bounce = 0.0;
	dReal geomIDO1BounceVelocity = 0.0;
	dReal geomIDO2BounceVelocity = 0.0;
	dReal geomIDO1CFM = 0.0;
	dReal geomIDO2CFM = 0.0;
	dReal geomIDO1ERP = 0.0;
	dReal geomIDO2ERP = 0.0;
	dReal geomIDO1MU = 0.0;
	dReal geomIDO2MU = 0.0;
	int geomIDO1UMU = 0;
	int geomIDO2UMU = 0;
	int geomIDO1GRAVITY = 0;
	int geomIDO2GRAVITY = 0;
	

	//Get the material properties of the two objects being tested
	while (test != NULL)
	{
		if (o1 == test->selfGeom)
		{
			geomIDO1Bounce = (test->objBounciness);
			geomIDO1BounceVelocity = (test->objBounceVel);
			geomIDO1CFM = (test->objCFM);
			geomIDO1ERP = (test->objERP);
			geomIDO1MU = (test->objMU);
			geomIDO1UMU = (test->objUseMU);
			geomIDO1GRAVITY = (test->objGravity);
		}

		if (o2 == test->selfGeom)
		{
			geomIDO2Bounce = (test->objBounciness);
			geomIDO2BounceVelocity = (test->objBounceVel);
			geomIDO2CFM = (test->objCFM);
			geomIDO2ERP = (test->objERP);
			geomIDO2MU = (test->objMU);
			geomIDO2UMU = (test->objUseMU);
			geomIDO2GRAVITY = (test->objGravity);
		}

		test->parent = iteminfo->parent(test->item);
		test = test->next;
	}

	sumBounce = (geomIDO1Bounce / 2.0) + (geomIDO2Bounce / 2.0);
	sumBounceVelocity = (geomIDO1BounceVelocity / 2.0) + (geomIDO2BounceVelocity / 2.0);
	sumCFM = (geomIDO1CFM / 2.0) + (geomIDO2CFM / 2.0);
	sumERP = (geomIDO1ERP / 2.0) + (geomIDO2ERP / 2.0);
	sumMU = (geomIDO1MU / 2.0) + (geomIDO2MU / 2.0);

	if ((geomIDO1UMU == 1) || (geomIDO2UMU == 1))
		useMU = 1;

  /////////////////////////////////////////////////////////////////////////
  // exit without doing anything if the two bodies are connected by a joint
  /////////////////////////////////////////////////////////////////////////
  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;


  ///////////////
  // Collision //
  ///////////////

  if ((dGeomGetClass(o1) != dPlaneClass) && (geomIDO1GRAVITY == DISABLED_UNTIL_COLLISION))
  //if (dGeomGetClass(o1) != dPlaneClass)
      dBodySetGravityMode(b1,1);
  if ((dGeomGetClass(o2) != dPlaneClass) && (geomIDO2GRAVITY == DISABLED_UNTIL_COLLISION))
  //if (dGeomGetClass(o2) != dPlaneClass)
	  dBodySetGravityMode(b2,1);

  // Generate the contacts
  dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
  for (i=0; i<MAX_CONTACTS; i++) {
	contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP;
	if (useMU)
		contact[i].surface.mu = dInfinity;
	else
		contact[i].surface.mu = sumMU;
    contact[i].surface.mu2 = 0;
    contact[i].surface.bounce = sumBounce;
    contact[i].surface.bounce_vel = sumBounceVelocity;
	contact[i].surface.soft_cfm = sumCFM;
	contact[i].surface.soft_erp = sumERP;
  }

  if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact)))
  {
	dMatrix3 RI;
    dRSetIdentity (RI);
	const dReal ss[3] = {0.02,0.02,0.02};
	for (i=0; i<numc; i++) {
		dJointID c = dJointCreateContact (world,contactgroup,contact+i);
		dJointAttach (c,b1,b2);
	}
  }
}


/*
======================================================================
KM_transpose4()
Calculates a transpose matrix.
This function could use some error checking.
====================================================================== */
void KM_transpose4 ( LWDMatrix4 n, LWDMatrix4* m, int i )
{

	LWMAT_dinitm4( n, m[i][0][0],m[i][1][0],m[i][2][0],0,
					  m[i][0][1],m[i][1][1],m[i][2][1],0,
					  m[i][0][2],m[i][1][2],m[i][2][2],0,
				      0         ,0         ,0         ,1);
}

/*
======================================================================
KM_transpose34()
Calculates a transpose matrix.
This function could use some error checking.
====================================================================== */
void KM_transpose34 ( dMatrix3 n, dMatrix3 m )
{
	KM_dinitm34( n, m[0],m[4],m[8],m[3],
					m[1],m[5],m[9],m[7],
					m[2],m[6],m[10],m[11]);
}

/*
======================================================================
KM_dinitm34()
Initiates a dMatrix3 matrix with the values provided.
====================================================================== */
void KM_dinitm34( dMatrix3 m, dReal a1, dReal b1, dReal c1, dReal d1,
							  dReal a2, dReal b2, dReal c2, dReal d2,
							  dReal a3, dReal b3, dReal c3, dReal d3 )
{
	m[0] = a1; m[1] = b1; m[2]  = c1; m[3]  = d1;
	m[4] = a2; m[5] = b2; m[6]  = c2; m[7]  = d2;
	m[8] = a3; m[9] = b3; m[10] = c3; m[11] = d3;
}


/*
======================================================================
KM_dmatmul34()
Multiplies two dMatrix3 matrices together resulting in a third matrix.

dMatrix3 a and dMatrix3 b are input
dMatrix3 c is the resultant matrix
====================================================================== */
void KM_dmatmul34( dMatrix3 a, dMatrix3 b, dMatrix3 c )
{
	c[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8];
	c[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9];
	c[2] = a[0]*b[2] + a[1]*b[6] + a[2]*b[10];
	c[3] = 0;
	c[4] = a[4]*b[0] + a[5]*b[4] + a[6]*b[8];
	c[5] = a[4]*b[1] + a[5]*b[5] + a[6]*b[9];
	c[6] = a[4]*b[2] + a[5]*b[6] + a[6]*b[10];
	c[7] = 0;
	c[8] = a[8]*b[0] + a[9]*b[4] + a[10]*b[8];
	c[9] = a[8]*b[1] + a[9]*b[5] + a[10]*b[9];
	c[10] = a[8]*b[2] + a[9]*b[6] + a[10]*b[10];
	c[11] = 0;
}


/*
======================================================================
KM_dtransform()
Converts a dMatrix3 to a LWDMatrix3, then multiplies a LWDVector by the
new LWDMatrix3 resulting in a rotated LWDVector.
====================================================================== */
void KM_dtransform( LWDVector a, dMatrix3 m, LWDVector b )
{
   int i;
   LWDMatrix3 n;

   LWMAT_dinitm3( n, m[0],m[8],m[4],
				     m[2],m[10],m[6],
					 m[1],m[9],m[5]);

   for ( i = 0; i < 3; i++ )
      b[ i ] = a[ 0 ] * n[ 0 ][ i ]
             + a[ 1 ] * n[ 1 ][ i ]
             + a[ 2 ] * n[ 2 ][ i ];
}


/*
======================================================================
Copied blatantly from lat_transform.c

Create a scale-only matrix from item parameters.  Calls itself
recursively to account for parenting.

@param iteminfo		LWItemInfo pointer
@param id			Item
@param time			Evaluation time
@param M			Result in Matrix form
====================================================================== */
void KM_LWMAT_getScales(LWItemInfo *iteminfo, LWItemID id, LWTime time, LWDMatrix4 M)
{
  LWDMatrix4 M2;
  LWDVector scale;
  LWItemID parent;

  iteminfo->param(id, LWIP_SCALING, time, scale);

  LWMAT_didentity4( M2 );
  M2[ 0 ][ 0 ] = ( double ) scale[ 0 ];
  M2[ 1 ][ 1 ] = ( double ) scale[ 1 ];
  M2[ 2 ][ 2 ] = ( double ) scale[ 2 ];
  LWMAT_dmatmul4( M, M2, M );

  if (parent = iteminfo->parent(id))
     KM_LWMAT_getScales(iteminfo, parent, time, M );
}


/*
======================================================================
CalcABVector()
Calculates the AB vector between any two bodies. Rotation is not
accounted for.
This function could use some error checking.
====================================================================== */
int CalcABVector (LWItemID id_a, LWItemID id_b, LWDVector delta )
{
	LWDVector objPosA;
	LWDVector objPosB;

	///////////////////////////////////////////////////////
	// Get the objects pivot position at the start frame 0
	///////////////////////////////////////////////////////
	iteminfo->param( id_a, LWIP_W_POSITION, 0, objPosA );
	iteminfo->param( id_b, LWIP_W_POSITION, 0, objPosB );

	///////////////////////////////////////
	// Get the relative position of B to A
	///////////////////////////////////////
	VSUB3(delta, objPosA, objPosB);

	return 0;
}


/*
======================================================================
CalcBoundingBox()
Calculate the bounding box and bounding sphere of an object.
This function could use some error checking.
====================================================================== */
int CalcBoundingBox ( LWItemID item )
{
	LWMeshInfoID mesh;
	ObjectData* test;
	int i,j;

	// get the current object
	test = FindObject(item);

	// initialize the first to vectors in the array
	for (i=0;i<2;i++)
		for (j=0;j<3;j++)
			test->bounds->bbox[i][j] = 0.0;

	// initialize the radius
	test->radius = 0.0;

	test->bounds->id = item; // should delete from bbox

	iteminfo->param( item, LWIP_RIGHT, 0, test->bounds->bbox[2]);
	iteminfo->param( item, LWIP_UP, 0, test->bounds->bbox[3]);
	iteminfo->param( item, LWIP_FORWARD, 0, test->bounds->bbox[4]);
	iteminfo->param( item, LWIP_W_RIGHT, 0, test->bounds->bbox[5]);
	iteminfo->param( item, LWIP_W_UP, 0, test->bounds->bbox[6]);
	iteminfo->param( item, LWIP_W_FORWARD, 0, test->bounds->bbox[7]);
	iteminfo->param( item, LWIP_W_POSITION, 0, test->bounds->bbox[8]);
	iteminfo->param( item, LWIP_PIVOT, 0, test->bounds->bbox[9]);
	iteminfo->param( item, LWIP_POSITION, 0, test->bounds->bbox[10]);

	// get the mesh info
	mesh = objinfo->meshInfo(item, 1);

	// scan all points
	// skip nulls
	mesh->scanPoints( mesh, (LWPntScanFunc*)pntScanBoundingBox, test );

	return 0;
}

/*
======================================================================
pntScanBoundingBox()

Point scan callback for Bounding Boxes and Radius.  Bounds is an array of two
LWDVectors.  It will contain two points which define the bounding box.
====================================================================== */
static int pntScanBoundingBox( ObjectData* test, LWPntID id )
{
	LWMeshInfo *mesh;
	LWFVector p,q;
	double radius;
	double x,y,z;
	int i;

	// get the point position
	mesh = objinfo->meshInfo(test->item, 1);
	mesh->pntOtherPos(mesh, id, q);

	// transform to item coordinates
	for ( i = 0; i < 3; i++ )
		p[ i ] = ( q[0] - test->bounds->bbox[8][0] ) * test->bounds->bbox[5][i]
			   + ( q[1] - test->bounds->bbox[8][1] ) * test->bounds->bbox[6][i]
			   + ( q[2] - test->bounds->bbox[8][2] ) * test->bounds->bbox[7][i]
			   + test->bounds->bbox[9][ i ];

	// test for min/max bound
	for ( i = 0; i < 3; i++ )
	{
		if (p[i] < test->bounds->bbox[0][i])
			test->bounds->bbox[0][i] = p[i];

		if (p[i] > test->bounds->bbox[1][i])
			test->bounds->bbox[1][i] = p[i];
	}

	x = p[0];
	y = p[1];
	z = p[2];


	radius = sqrt((x*x) + (y*y) + (z*z));

	// find the largest radius. Not nescessarily the bounds
	if (test->radius < radius) test->radius = radius;

   return 0;
}


/*
======================================================================
findRootParents()

Recursive function to find the root parent of each object in the
objMatrixData Linked List.  Not interested in hierarchy of
parent/child relationships, just which objects are the 'root' objects
and which are children, independent of how many levels down.
Just being a rootParent doesn't necessarily mean the object has
anything parented to it.
====================================================================== */

LWItemID findRootParents(ObjectData* test)
{
	LWItemID rootP = NULL;

	//////////////////////////////////////////////////////////////////
	// Check each object for parent data
	// This can be simplified nicely later
	//////////////////////////////////////////////////////////////////

	if (test->parent == NULL) return test->item; //a root parent is found

	if (test->rootParent != NULL) // node already tested
	{
		return test->rootParent;
	}
	else
	{
		rootP = findRootParents(FindObject(test->parent));
	}

	return rootP;
}


/*
======================================================================
sortList()

Sorting function to place root parents at the front of the ObjectData
linked list.

The idea is simple, iterate over the list once. As root parents are
found, move them to the front of the list.

No error checking implemented.
====================================================================== */

int sortList()
{

	ObjectData* test = objMatrixData; //start at the second object
	ObjectData* next = NULL;
	ObjectData* last = NULL;

	if (test == NULL || test->next == NULL) //one object
	{
		return 0;
	}

	while (test != NULL) //two or more
	{
		if (test == objMatrixData)//skip first object
		{
			last = test;
			test = test->next;
			continue;
		}

		if (test->parent == NULL) //test for root parent
		{
			next = test->next;
			last->next = test->next;
			test->next = objMatrixData;
			objMatrixData = test;		
		}
		else
		{
			last = test;
		}

		test = last->next;
	}

	return 0;
}


/*
======================================================================
findEnv()

From the Lightwave 9 SDK

Our findEnv function simply loops through the channels in a channel
group searching for a given channel name. If a match is found, it
returns the envelope ID for the channel.

====================================================================== */
LWEnvelopeID findEnv( LWChanGroupID group, char *name )
{
	LWChannelID chan;

	chan = chinfo->nextChannel( group, NULL );
	while ( chan ) {
		if ( !strcmp( chinfo->channelName( chan ), name ))
			return chinfo->channelEnvelope( chan );
		chan = chinfo->nextChannel( group, chan );
	}
	return NULL;
}


/*
======================================================================
CalcKeyData()
Determine if an ODE objet is keyframed and if true fill the 
KeyData data structure

typedef struct st_KeyData {
	LWTime	lastKeyTime;
	double	lastKeyDelta[3];
	double	lastKeyPosition[3];
	double	lastKeyVelocity[3];
} KeyData;

====================================================================== */
int CalcKeyData ( ObjectData* test )
{
	LWEnvKeyframeID currentKey[3], nextKey[3], previousKey[3];
	LWEnvelopeID env[3];
	LWChanGroupID group;
	LWTime keyTime[2];
	int keyResult;
	double keyValLast[3], keyVal2ndLast[3];

	group = iteminfo->chanGroup( test->item );

	env[0] = findEnv( group, "Position.X" );
	env[1] = findEnv( group, "Position.Y" );
	env[2] = findEnv( group, "Position.Z" );

	//Get the keys for each object for frame 0 
	currentKey[0] = envf->findKey( env[0], 0.0 );
	currentKey[1] = envf->findKey( env[1], 0.0 );
	currentKey[2] = envf->findKey( env[2], 0.0 );

	//initialize the previous keys
	previousKey[0] = currentKey[0];
	previousKey[1] = currentKey[1];
	previousKey[2] = currentKey[2];

	nextKey[0] = envf->nextKey( env[0], currentKey[0] );
	nextKey[1] = envf->nextKey( env[1], currentKey[1] );
	nextKey[2] = envf->nextKey( env[2], currentKey[2] );

	while (nextKey[0] != NULL)
	{
		previousKey[0] = currentKey[0];
		previousKey[1] = currentKey[1];
		previousKey[2] = currentKey[2];

		currentKey[0] = nextKey[0];
		currentKey[1] = nextKey[1];
		currentKey[2] = nextKey[2];

		nextKey[0] = envf->nextKey( env[0], currentKey[0] );
		nextKey[1] = envf->nextKey( env[1], currentKey[1] );
		nextKey[2] = envf->nextKey( env[2], currentKey[2] );
	}

	keyResult = envf->keyGet( env[0], currentKey[0], LWKEY_TIME, &keyTime[0] );
	keyResult = envf->keyGet( env[0], previousKey[0], LWKEY_TIME, &keyTime[1] );

	keyResult = envf->keyGet( env[0], currentKey[0], LWKEY_VALUE, &keyValLast[0] );
	keyResult = envf->keyGet( env[1], currentKey[1], LWKEY_VALUE, &keyValLast[1] );
	keyResult = envf->keyGet( env[2], currentKey[2], LWKEY_VALUE, &keyValLast[2] );

	keyResult = envf->keyGet( env[0], previousKey[0], LWKEY_VALUE, &keyVal2ndLast[0] );
	keyResult = envf->keyGet( env[1], previousKey[1], LWKEY_VALUE, &keyVal2ndLast[1] );
	keyResult = envf->keyGet( env[2], previousKey[2], LWKEY_VALUE, &keyVal2ndLast[2] );

	//Now fill in the KeyData for for the item
	if (previousKey[0] == currentKey[0])
	{
		test->objKey->lastKeyTime = 0.0;

		test->objKey->lastKeyDelta[0] = 0.0;
		test->objKey->lastKeyDelta[1] = 0.0;
		test->objKey->lastKeyDelta[2] = 0.0;

		test->objKey->lastKeyVelocity[0] = 0.0;
		test->objKey->lastKeyVelocity[1] = 0.0;
		test->objKey->lastKeyVelocity[2] = 0.0;
	}
	else
	{
		test->objKey->lastKeyTime = keyTime[0];
		
		test->objKey->lastKeyPosition[0] = keyValLast[0];
		test->objKey->lastKeyPosition[1] = keyValLast[1];
		test->objKey->lastKeyPosition[2] = keyValLast[2];

		test->objKey->lastKeyDelta[0] = keyValLast[0] - keyVal2ndLast[0];
		test->objKey->lastKeyDelta[1] = keyValLast[1] - keyVal2ndLast[1];
		test->objKey->lastKeyDelta[2] = keyValLast[2] - keyVal2ndLast[2];

		test->objKey->lastKeyVelocity[0] = (test->objKey->lastKeyDelta[0] / (keyTime[0] - keyTime[1]));
		test->objKey->lastKeyVelocity[1] = (test->objKey->lastKeyDelta[1] / (keyTime[0] - keyTime[1]));
		test->objKey->lastKeyVelocity[2] = (test->objKey->lastKeyDelta[2] / (keyTime[0] - keyTime[1]));
	}

	return 1;
}


/*
======================================================================
Start Displacement here
Base code from Inertia.c. Thanks Ernie and Newtek
======================================================================

======================================================================
popCnt_VMAP()

Return the number of weight maps, plus 1 for "(none)".  An xpanel
callback for the vmap popup list, also used by Load().
====================================================================== */

XCALL_( static int )
popCnt_VMAP( void *data )
{
   return 1 + objf->numVMaps( LWVMAP_WGHT );
}


/*
======================================================================
popName_VMAP()

Return the name of a vmap, given an index.  An xpanel callback for the
vmap popup list, also used by Load() and Save().
====================================================================== */

XCALL_( static const char * )
popName_VMAP( void *data, int idx )
{
   if ( idx == 0 ) return "(none)";

   return objf->vmapName( LWVMAP_WGHT, idx - 1 );
}


/*
======================================================================
popCnt_Item()

Return the number of objects, plus 1 for "(none)".  An xpanel
callback for the pivot object popup list.
====================================================================== */

XCALL_( static int )
popCnt_Item( void *data )
{
   int n = 1;
   LWItemID id;

   id = iteminfo->first( LWI_OBJECT, NULL );

   while ( id )
   {
      n++;
      id = iteminfo->next( id );
   }

   return n;
}


/*
======================================================================
popName_Item()

Return the name of an object, given an index.  An xpanel callback for
the pivot object popup list.
====================================================================== */

XCALL_( static const char * )
popName_Item( void *data, int idx )
{
   const char *a;
   int i = 0;
   LWItemID id;

   if ( idx == 0 ) return "(none)";

   idx--;

   if ( id = iteminfo->first( LWI_OBJECT, NULL ) )
	  a = iteminfo->name( id );

   while ( id && i < idx )
   {
      i++;
      if ( id = iteminfo->next( id ))
         a = iteminfo->name( id );
   }

   return a;
}


/*
======================================================================
popItemID()

Return the item ID of an object, given an index.
====================================================================== */

static LWItemID popItemID( int idx )
{
   LWItemID id;
   int i = 0;

   if ( idx <= 0 ) return NULL;
   idx--;

   id = iteminfo->first( LWI_OBJECT, NULL );
   while ( id && i < idx )
   {
      i++;
      id = iteminfo->next( id );
   }

   return id;
}


/*
======================================================================
AddObject()

Add an object to the end of the ODE Linked List.

This is where we would determine if an object had a parent.
====================================================================== */

XCALL_( static LWError )
AddObject(LWItemID id, ODEDisplaceData *dat)
{

	ObjectData* test = objMatrixData;
	LWDVector objPos;
	LWItemID parentID;


	//////////////////////////////////
	// Find the end of the LinkedList
	//////////////////////////////////

	while (test != NULL && test->next != NULL)
	{
		test=test->next;
	}

	///////////////////////////////////////////////////////
	// Get the objects pivot position at the start frame 0
	///////////////////////////////////////////////////////
	iteminfo->param( dat->item, LWIP_W_POSITION, 0, objPos );

	///////////////////////////////////////////////////////
	// Get the parent object, if any, otherwise null
	///////////////////////////////////////////////////////
	parentID = iteminfo->parent(id);

	//////////////////////////////////////////////////////////////
	// Create and add the new object to the end of the linked list
	// Make sure these values match those of Create() below
	//////////////////////////////////////////////////////////////
	
	if (objMatrixData == NULL)
	{
		objMatrixData = (ObjectData*)malloc(sizeof(ObjectData));
		test = objMatrixData;
	}
	else
	{
		test->next = (ObjectData*)malloc(sizeof(ObjectData));
		test=test->next;
	}

	// initialize structs
	test->bounds = (BBox*)malloc(sizeof(BBox));
	test->objKey = (KeyData*)malloc(sizeof(KeyData));

	test->m = NULL;
	test->item = id;
	test->parent = parentID;
	test->rootParent = NULL;
	test->numChildren = 0;
	test->odeNum = 0;
	test->next = NULL;
	test->objType = SPHERE;  // default
	test->objStatic = NONSTATIC;
	test->objGravity = ENABLED;
	test->objBounciness = 0.5;
	test->objBounceVel = 0.25;
	test->objDensity = 5.0;
	test->objCFM = 0.01;
	test->objERP = 0.2;
	test->objMU = 1.0;
	test->objUseMU = 0;
	test->xpos = objPos[0];
	test->ypos = objPos[1];
	test->zpos = objPos[2];

	return AFUNC_OK;
}


/*
======================================================================
RemoveObject()

Remove an object from the ODE Linked List
====================================================================== */

void RemoveObject(LWItemID id)
{

	ObjectData* test = objMatrixData;
	ObjectData* last = objMatrixData;

	//////////////////////////////////////////////////
	// Trying to remove a Linked node when none exist
	//////////////////////////////////////////////////
	if (objMatrixData == NULL)
		return;

	// Only one object
	if (objMatrixData->item == id && objMatrixData->next == NULL)
	{

		free (objMatrixData->m);
		objMatrixData->m = NULL;

		free (objMatrixData->bounds);
		objMatrixData->bounds = NULL;

		free (objMatrixData);
		objMatrixData = NULL;

		return;
	}

	// Only two objects
	if (objMatrixData->item == id)
	{
		test = objMatrixData->next;

		free (objMatrixData->m);
		objMatrixData->m = NULL;

		free (objMatrixData->bounds);
		objMatrixData->bounds = NULL;

		free (objMatrixData);
		objMatrixData = test;

		return;
	}

	// Find the object 
	while (test->item != id)
	{
		last = test;
		test = test->next;
	}

	// Determine if it is last or not
	if (test->next != NULL)
	{
		last->next = test->next;
	}
	else
	{
		last->next = NULL;
	}

	free (test->m);
	test->m = NULL;

	free (test->bounds);
	test->bounds = NULL;

	free (test);
	test = NULL;

}


/*
======================================================================
ObjectData* FindObject()

Find an object in the ODE Linked List
====================================================================== */

ObjectData* FindObject( LWItemID id)
{

	ObjectData* test = objMatrixData;

	/////////////////////
	// Empty Linked List
	/////////////////////
	if (test == NULL) return NULL;


	//////////////////////////////////////////////////
	// Find the object with the requested LWItemID
	//////////////////////////////////////////////////
	while (test->item != id)
	{
		test = test->next;
		if (test == NULL) return NULL;
	}

	return test;
}


/*
======================================================================
Create()

Handler callback.  Allocate and initialize Displacement instance data.
====================================================================== */

XCALL_( static LWInstance )
Create( void *priv, LWItemID item, LWError *err )
{
   ODEDisplaceData *dat;

   //set default values
   if ( dat = (struct st_ODEDisplaceData *)calloc( 1, sizeof( ODEDisplaceData ))) {
	   memset( dat, 0, sizeof(ODEDisplaceData) );
	   dat->objType = 0;
	   dat->objStatic = 0;
	   dat->objGravity = ENABLED;
	   dat->objBounciness = 0.5;
	   dat->objBounceVel = 0.25;
	   dat->objDensity = 5.0;
	   dat->objCFM = 0.01;
	   dat->objERP = 0.2;
	   dat->objMU = 1.0;
	   dat->objUseMU = 0;
	   dat->item = item;
   }

   ////////////////////////////////////////////////
   // Add the object to the ODE object linked list
   ////////////////////////////////////////////////
   AddObject(item, dat);

   return dat;
}


/*
======================================================================
Destroy()

Handler callback.  Free resources allocated by Create().
====================================================================== */

XCALL_( static void )
Destroy( ODEDisplaceData *dat )
{

	/////////////////////////////////////////////////////
	// Clear an object from the ODE object linked list
	/////////////////////////////////////////////////////

	RemoveObject(dat->item);
	
	if( dat ) {
		free( dat );
		dat = NULL;
	}
}


/*
======================================================================
Copy()

Handler callback.  Copy instance data.  If your instance data contains
allocated resources, note that a simple *to = *from is insufficient.

Although the plug-in works correctly with clones, we're really just
copying data between data structures.  I think it might be better to
have a pointer from the ObjectData structs directly to the corresponding
ODEDisplaceData struct.

====================================================================== */

XCALL_( static LWError )
Copy( ODEDisplaceData *to, ODEDisplaceData *from )
{
   LWItemID id;
   ObjectData* fromObjectData;
   ObjectData* toObjectData;

   //Copy data to the ODEDisplaceData struct
   fromObjectData = FindObject(from->item);

   id = to->item;
   *to = *from;
   to->item = id;
   to->objType = fromObjectData->objType;
   to->objStatic = fromObjectData->objStatic;
   to->objGravity = fromObjectData->objGravity;
   to->objBounciness = fromObjectData->objBounciness;
   to->objBounceVel = fromObjectData->objBounceVel;
   to->objDensity = fromObjectData->objDensity;
   to->objCFM = fromObjectData->objCFM;
   to->objERP = fromObjectData->objERP;
   to->objMU = fromObjectData->objMU;

   //Copy data to the ObjectData struct
   toObjectData = FindObject(to->item);

   toObjectData->objType = fromObjectData->objType;
   toObjectData->objStatic = fromObjectData->objStatic;
   toObjectData->objGravity = fromObjectData->objGravity;
   toObjectData->objBounciness = fromObjectData->objBounciness;
   toObjectData->objBounceVel = fromObjectData->objBounceVel;
   toObjectData->objDensity = fromObjectData->objDensity;
   toObjectData->objCFM = fromObjectData->objCFM;
   toObjectData->objERP = fromObjectData->objERP;
   toObjectData->objMU = fromObjectData->objMU;
   
   return AFUNC_OK;
}


/*
======================================================================
Load()

Handler callback.  Read instance data.
====================================================================== */

XCALL_( static LWError )
Load( ODEDisplaceData *dat, const LWLoadState *ls )
{
   ObjectData* item;
   short ver;

   item = FindObject( dat->item );

   LWLOAD_I2( ls, &ver, 1 );
   LWLOAD_I4( ls, (long*)&dat->objType, 1 );
   LWLOAD_I4( ls, (long*)&dat->objStatic, 1 );
   LWLOAD_FP( ls, (float*)&dat->objBounciness, 3 );
   LWLOAD_FP( ls, (float*)&dat->objBounceVel, 3 );
   LWLOAD_FP( ls, (float*)&dat->objDensity, 3 );
   LWLOAD_FP( ls, (float*)&dat->objCFM, 3 );
   LWLOAD_FP( ls, (float*)&dat->objERP, 3 );   
   LWLOAD_FP( ls, (float*)&dat->objMU, 3 );
   LWLOAD_FP( ls, (float*)&gravity, 3 );
   LWLOAD_I4( ls, (long*)&stepSizeMultiplier, 2 );
   LWLOAD_I4( ls, (long*)&dat->objUseMU, 1 );
   LWLOAD_I4( ls, (long*)&dat->objGravity, 1 );
   LWLOAD_I4( ls, (long*)&groundPlane, 1 );
   
   // Need to fix objType to be consistant
   item->objType = dat->objType;
   item->objStatic = dat->objStatic;
   item->objBounciness = dat->objBounciness;
   item->objBounceVel = dat->objBounceVel;
   item->objDensity = dat->objDensity;
   item->objCFM = dat->objCFM;
   item->objERP = dat->objERP;
   item->objMU = dat->objMU;
   item->objUseMU = dat->objUseMU;
   item->objGravity = dat->objGravity;

   return AFUNC_OK;
}


/*
======================================================================
Save()

Handler callback.  Write instance data.
====================================================================== */

XCALL_( static LWError )
Save( ODEDisplaceData *dat, const LWSaveState *ss )
{
   short ver = ODEDISPLACE_VERSION;

   LWSAVE_I2( ss, &ver, 1 );
   LWSAVE_I4( ss, (long*)&dat->objType, 1 );
   LWSAVE_I4( ss, (long*)&dat->objStatic, 1 );
   LWSAVE_FP( ss, (float*)&dat->objBounciness, 3 );
   LWSAVE_FP( ss, (float*)&dat->objBounceVel, 3 );
   LWSAVE_FP( ss, (float*)&dat->objDensity, 3 );
   LWSAVE_FP( ss, (float*)&dat->objCFM, 3 );
   LWSAVE_FP( ss, (float*)&dat->objERP, 3 );   
   LWSAVE_FP( ss, (float*)&dat->objMU, 3 );
   LWSAVE_FP( ss, (float*)&gravity, 3 ); //hack to save gravity for scene
   LWSAVE_I4( ss, (long*)&stepSizeMultiplier, 2 );
   LWSAVE_I4( ss, (long*)&dat->objUseMU, 1 );
   LWSAVE_I4( ss, (long*)&dat->objGravity, 1 );
   LWSAVE_I4( ss, (long*)&groundPlane, 1 );
	
   return AFUNC_OK;
}


/*
======================================================================

Describe()

Handler callback.  Write a short, human-readable string describing
the instance data.
====================================================================== */

XCALL_( static const char * )
Describe( ODEDisplaceData *dat )
{
	sprintf( dat->desc, " ODEfL v0.4.3 BETA", NULL );
   return dat->desc;
}


/*
======================================================================
UseItems()

Handler callback.  Return an array of items we depend on.
====================================================================== */

XCALL_( static const LWItemID * )
UseItems( ODEDisplaceData *dat )
{
	return NULL;
}


/*
======================================================================
ChangeID()

Handler callback.  ID numbers for items in the scene aren't constant.
They can change when items are added or deleted.  If the ID of our
pivot object has changed, we update our instance data.
====================================================================== */

XCALL_( static void )
ChangeID( ODEDisplaceData *dat, const LWItemID *ids )
{
   return;
}


/*
======================================================================
Init()

Handler callback, called at the start of rendering.
====================================================================== */

XCALL_( static LWError )
Init( ODEDisplaceData *dat, int mode )
{
   return NULL;
}


/*
======================================================================
Cleanup()

Handler callback, called at the end of rendering.
====================================================================== */

XCALL_( static void )
Cleanup( ODEDisplaceData *dat )
{
   return;
}


/*
======================================================================
NewTime()

Handler callback, called at the start of each sampling pass.
====================================================================== */

XCALL_( static LWError )
NewTime( ODEDisplaceData *dat, LWFrame fr, LWTime t )
{
   dat->frame = fr;
   dat->time = t;

   return NULL;
}


/*
======================================================================
Flags()

Handler callback.
====================================================================== */

XCALL_( static int )
Flags( ODEDisplaceData *dat )
{
   return 0;
}


/*
======================================================================
Evaluate()

Handler callback.  This is called for each vertex of the object the
instance is associated with.  The vertex is moved by setting the
members of the source[] array in the displacement access structure.
====================================================================== */

XCALL_( static void )
Evaluate( ODEDisplaceData *dat, LWDisplacementAccess *da )
{
	LWDMatrix4 objRot, objRotT;
	LWDMatrix4 mScale, mScaleT;
	LWDVector vertexPos;
	ObjectData* test;
	int cf;  // Current Frame - are fractional frames a concern here?

	////////////////////////////////////////////////////////////////////////
	// All that needs to be done is to get the correct transfomation matrix
	// array for the current object and multiply it against the vertices
	////////////////////////////////////////////////////////////////////////

	// Initialize the matices
	LWMAT_didentity4(objRot);
	LWMAT_didentity4(objRotT);
	LWMAT_didentity4(mScale);
	LWMAT_didentity4(mScaleT);

	// Get the current frame
	cf = dat->frame;

	// Get the current object structure
	test = FindObject(dat->item);

	// Quit the eval function if this is a new object with no ODE transform data
	if (test == NULL || test->m == NULL) return;

	KM_LWMAT_getScales(iteminfo, dat->item, cf*scninfo->framesPerSecond, mScale);
	
	// Matrix to rotate from item to world coordinates
	objRot[0][0] = test->bounds->bbox[5][0];
	objRot[0][1] = test->bounds->bbox[5][1];
	objRot[0][2] = test->bounds->bbox[5][2];
	objRot[0][3] = 0;
	objRot[1][0] = test->bounds->bbox[6][0];
	objRot[1][1] = test->bounds->bbox[6][1];
	objRot[1][2] = test->bounds->bbox[6][2];
	objRot[1][3] = 0;
	objRot[2][0] = test->bounds->bbox[7][0];
	objRot[2][1] = test->bounds->bbox[7][1];
	objRot[2][2] = test->bounds->bbox[7][2];
	objRot[2][3] = 0;
	objRot[3][0] = 0;
	objRot[3][1] = 0;
	objRot[3][2] = 0;
	objRot[3][3] = 1;

	// Put the coordinates for the current vertex into a LWDVector
	vertexPos[0] = da->source[ 0 ];
	vertexPos[1] = da->source[ 1 ];
	vertexPos[2] = da->source[ 2 ];

	// Multiply the coordinate by the matrix for the correct object at the current frame
	// This version screws up when scaled
	LWMAT_dmatmul4 (test->m[cf], objRot, objRotT); //transforms to the objects local coord frame (includes scaling)
	LWMAT_dtransformp( vertexPos, objRotT, da->source);

	// Done tranformation
}

/*
======================================================================
ODEDisplace()

Handler activation function.  Check the version, get some globals, and
fill in the callback fields of the handler structure.
====================================================================== */

XCALL_( int )
ODEDisplace( long version, GlobalFunc *global, LWDisplacementHandler *local, void *serverData)
{
   if ( version != LWDISPLACEMENT_VERSION )
      return AFUNC_BADVERSION;

   objf     = (struct st_LWObjectFuncs *)global( LWOBJECTFUNCS_GLOBAL, GFUSE_TRANSIENT );
   envf     = (struct st_LWEnvelopeFuncs *)global( LWENVELOPEFUNCS_GLOBAL, GFUSE_TRANSIENT );
   chanf    = (struct st_LWChannelInfo *)global( LWCHANNELINFO_GLOBAL, GFUSE_TRANSIENT );
   iteminfo = (struct st_LWItemInfo *)global( LWITEMINFO_GLOBAL, GFUSE_TRANSIENT);
   scninfo  = (struct st_LWSceneInfo *)global( LWSCENEINFO_GLOBAL, GFUSE_TRANSIENT );


   if ( !objf || !envf || !chanf || !iteminfo || !scninfo )
      return AFUNC_BADGLOBAL;

   local->inst->create   = Create;
   local->inst->destroy  = (void (*)(void *))Destroy;
   local->inst->load     = (const char *(*)(void *,const struct st_LWLoadState *))Load;
   local->inst->save     = (const char *(*)(void *,const struct st_LWSaveState *))Save;
   local->inst->copy     = (const char *(__cdecl *)(void *,void *))Copy;
   local->inst->descln   = (const char *(__cdecl *)(void *))Describe;

   local->item->useItems = (void *const *(__cdecl *)(void *))UseItems;
   local->item->changeID = (void (__cdecl *)(void *,void *const * ))ChangeID;

   local->rend->init     = (const char *(__cdecl *)(void *,int))Init;
   local->rend->cleanup  = (void (__cdecl *)(void *))Cleanup;
   local->rend->newTime  = (const char *(__cdecl *)(void *,int,double))NewTime;

   local->evaluate       = (void (__cdecl *)(void *,struct st_LWDisplacementAccess *))Evaluate;
   local->flags          = (unsigned int (__cdecl *)(void *))Flags;

   return AFUNC_OK;
}


/*
======================================================================
ODEDisplaceData_get()

XPanel callback.  XPanels calls this when it wants to retrieve the
value of a control from your instance data.  Returns a pointer to the
field containing the requested value.
====================================================================== */
XCALL_( static void * )
ODEDisplaceData_get( ODEDisplaceData *dat, unsigned long vid )
{
   void *result = NULL;
   ObjectData* object;
   object = FindObject(dat->item);

   if ( dat )
      switch ( vid ) {
         case CH_TYPE:			return &dat->objType;
         case CH_STATIC:		return &dat->objStatic;
         case CH_GRAVITY:		return &dat->objGravity;
		 case CH_BOUNCINESS:	return &dat->objBounciness;
		 case CH_BOUNCEVEL:		return &dat->objBounceVel;
		 case CH_DENSITY:		return &dat->objDensity;
		 case CH_CFM:			return &dat->objCFM;
		 case CH_ERP:			return &dat->objERP;
		 case CH_MU_ENABLE:		return &dat->objUseMU;
		 case CH_MU:
			 dat->objMU = object->objMU;
			 return &dat->objMU;
      }
  return NULL;
}

/*
======================================================================
ODEDisplaceData_set()

XPanel callback.  XPanels calls this when it wants to store the value
of a control in your instance data.  Returns 1 if the value could be
stored, otherwise 0.
====================================================================== */
XCALL_( static int )
ODEDisplaceData_set( ODEDisplaceData *dat, unsigned long vid, void *value )
{
   ObjectData* object;
   object = FindObject(dat->item);

   if ( dat )
      switch ( vid ) {
        case CH_TYPE:
			 dat->objType = *(( int * ) value );
			 	if (dat->objType == 0)
					object->objType = SPHERE;
				else if (dat->objType == 1)
					object->objType = BOX;
				else if (dat->objType == 2)
					object->objType = CYLINDER;
				else if (dat->objType == 3)
					object->objType = CAPSULE;
				else if (dat->objType == 4)
					object->objType = TRIMESH;
			 break;

        case CH_STATIC:
			 dat->objStatic = *(( int * ) value );
			 	if (dat->objStatic == 0)
					object->objStatic = NONSTATIC;
				else if (dat->objStatic == 1)
					object->objStatic = STATIC;
			 break;

        case CH_GRAVITY:
			 dat->objGravity = *(( int * ) value );
			 	if (dat->objGravity == 0)
					//object->objGravity = ENABLED;
					object->objGravity = DISABLED;
				else if (dat->objGravity == 1)
					//object->objGravity = DISABLED;
					object->objGravity = ENABLED;
				else if (dat->objGravity == 2)
					object->objGravity = DISABLED_UNTIL_COLLISION;
			 break;

        case CH_BOUNCINESS:
			 object->objBounciness = *(( double * ) value );
			 dat->objBounciness = object->objBounciness;
			 break;

		case CH_BOUNCEVEL:
			 object->objBounceVel = *(( double * ) value );
			 dat->objBounceVel = object->objBounceVel;
			 break;

		case CH_DENSITY:
			 object->objDensity = *(( double * ) value );
			 dat->objDensity = object->objDensity;
			 if (object->objDensity <= 0.0){ //clamp denisty to a positive non-zero value
				object->objDensity = 0.0001;
				dat->objDensity = object->objDensity;
			 }
			 break;

		case CH_CFM:
			 object->objCFM = *(( double * ) value );
			 dat->objCFM = object->objCFM;
			 break;
			 
		case CH_ERP:
			 object->objERP = *(( double * ) value );
			 dat->objERP = object->objERP;
			 break;
			 
		case CH_MU_ENABLE:
			 object->objUseMU = *(( int * ) value );
			 dat->objUseMU = object->objUseMU;
			 if (dat->objUseMU == 0)
				dat->objMU = dInfinity;
			 else if (dat->objUseMU == 1)
				dat->objMU = object->objMU;
			 break;

		case CH_MU:
			 object->objMU = *(( double * ) value );
			 dat->objMU = object->objMU;
			 break;

		default:
			 return 0;
      }

   return 1;
}

/*
======================================================================
ChangeNotify()

Xpanel callback.  XPanels calls this when an event occurs that affects
the value of one of your controls.  We use the instance update global
to tell Layout that our instance data has changed.
====================================================================== */
XCALL_( static void )
ChangeNotify( LWXPanelID panID, unsigned long cid, unsigned long vid, int event_code )
{
   void *dat;

   if ( event_code == LWXPEVENT_VALUE )
      if ( dat = xpanf->getData( panID, 0 ))
         lwupdate( LWDISPLACEMENT_HCLASS, dat );
}

/*
======================================================================
DrawXpLogo()

Xpanel callback.  XPanels calls this when an event occurs that affects
the value of one of your controls.  We use the instance update global
to tell Layout that our instance data has changed.
====================================================================== */
XCALL_( static void )
DrawXpLogo(LWXPanelID pan, unsigned long cid, LWXPDrAreaID reg, int w, int h)
{
  int x=0;
  int y=0;
  int i=0;
  int j=0;
  int index=0;

  LWXPDrawFuncs *drawf = xpanf->drawf;

  /////////////////////////////////////////////////////////////////////
  //Depending on which app created the image used to generate image1.h,
  //we may need to flip the order in which we write y.
  /////////////////////////////////////////////////////////////////////
  //y = IMAGE1HEIGHT;
  y = 0;
  for(j=0;j<IMAGE1HEIGHT;j++)
  {
	  x = (w-IMAGE1WIDTH)/2;
	  for(i=0;i<IMAGE1WIDTH;i++){
		  (*drawf->drawRGBPixel)(reg,image1_data[index],image1_data[index+1],image1_data[index+2],x++,y);
		  index+=3;
	  }
      //y--;
	  y++;
   }
   return;
}

/*
======================================================================
get_xpanel()

Create and initialize an xpanel.
====================================================================== */

static LWXPanelID get_xpanel( GlobalFunc *global, ODEDisplaceData *dat )
{
	LWXPanelID panID = NULL;

	static LWXPanelControl ctrl_list[] = {
	{ CH_LOGO, "Logo",  "dThumbnail" },
	{ CH_TYPE, STR_TYPE_TEXT,  "iChoice" },
	{ CH_STATIC, "Static",  "iBoolean" },
	{ CH_GRAVITY, STR_GRAVITY_TEXT,  "iChoice" },
	{ CH_BOUNCINESS, "Bounciness",  "float" },
	{ CH_BOUNCEVEL, "BounceVel",  "float" },
	{ CH_DENSITY, "Density",  "float" },
	{ CH_CFM, "Soft_CFM",  "float" },
	{ CH_ERP, "Soft_ERP",  "float" },
	{ CH_MU_ENABLE, "Enable Infinite Friction",  "iBoolean" },
	{ CH_MU, "Friction",  "float" },
	{ 0 }
	};

	static LWXPanelDataDesc data_descrip[] = {
	{ CH_LOGO, "Logo",  "integer" },
	{ CH_TYPE, STR_TYPE_TEXT,  "integer" },
	{ CH_STATIC, "Static",  "integer" },
	{ CH_GRAVITY, STR_GRAVITY_TEXT,  "integer" },
	{ CH_BOUNCINESS, "Bounciness",  "float" },
	{ CH_BOUNCEVEL, "BounceVel",  "float" },
	{ CH_DENSITY, "Density",  "float" },
	{ CH_CFM, "Soft_CFM",  "float" },
	{ CH_ERP, "Soft_ERP",  "float" },
	{ CH_MU_ENABLE, "Use_IF",  "integer" },
	{ CH_MU, "Friction",  "float" },   
	{ 0 }
	};

	const char *objType_strlist[] = {
	"Sphere",
	"Box",
	"Cylinder",
	"Capsule",
	"Mesh",
	NULL
	};	

	const char *objGravity_strlist[] = {
	"Disable",
	"Enable",
	"Disable until Collision",
	NULL
	};	

	static int EnableMap[] = {1};

	static LWXPanelHint hint[] = {
		XpLABEL( 0x8000, "ODEfL Displace" ),
		XpCTRLCFG ( CH_LOGO, THUM_LRG | THUM_FULL ),
		XpDRAWCBFUNC ( CH_LOGO, DrawXpLogo ),
		XpCHGNOTIFY( ChangeNotify ),
		XpSTRLIST( CH_TYPE, objType_strlist),
		XpORIENT( CH_TYPE, 1 ),
		XpSTRLIST( CH_GRAVITY, objGravity_strlist),
		XpORIENT( CH_GRAVITY, 1 ),
		XpENABLEMSG_MAP_(CH_MU_ENABLE,EnableMap,"Not enabled"),
		XpH(CH_MU),
		XpEND,
		XpEND
	};
	   
	xpanf = (struct st_LWXPanelFuncs *)global( LWXPANELFUNCS_GLOBAL, GFUSE_TRANSIENT );
	if ( xpanf ) {
		panID = xpanf->create( LWXP_VIEW, ctrl_list );
		if ( panID ) {
			xpanf->hint( panID, 0, hint );
			xpanf->describe(
				panID,
				data_descrip,
				(void *(__cdecl *)(void *,unsigned long))ODEDisplaceData_get,
				(enum en_LWXPRefreshCodes (__cdecl *)(void *,unsigned long,void *))ODEDisplaceData_set);
			xpanf->viewInst( panID, (void *(__cdecl *)(void *,unsigned long))dat );
			xpanf->setData( panID, 0, dat );
			xpanf->setData( panID, 2, dat );
		}
	}
	return panID;
}

/*
======================================================================
ODEDisplace_UI()

Interface activation function.  Get a global, create an xpanel, and
fill in the fields of the LWInterface structure.
====================================================================== */

XCALL_( int )
ODEDisplace_UI( long version, GlobalFunc *global, LWInterface *local, void *serverData )
{
   if ( version != LWINTERFACE_VERSION )
      return AFUNC_BADVERSION;

   lwupdate = (void (__cdecl *)(const char *,void *))global( LWINSTUPDATE_GLOBAL, GFUSE_TRANSIENT );
   if ( !lwupdate )
      return AFUNC_BADGLOBAL;

   local->panel = get_xpanel( global, (struct st_ODEDisplaceData *)local->inst );
   if ( !local->panel )
      return AFUNC_BADGLOBAL;

   local->options = NULL;
   local->command = NULL;

   return AFUNC_OK;
}


/*
======================================================================
This is the server description.  LightWave looks at this first to
determine what plug-ins the file contains.  It lists each plug-in's
class and internal name, along with a pointer to the activation
function.  You can optionally add a user name, or more than one in
different languages, if you like.
====================================================================== */
extern "C" {
	ServerRecord ServerDesc[] = {
	{ LWLAYOUTGENERIC_CLASS, "ODE_Generic",
		(int (*)(long,void * (*)(const char *,int),void *,void *))ODEGeneric },
	{ LWDISPLACEMENT_HCLASS, "ODE_Displace",
		(int (*)(long,void * (*)(const char *,int),void *,void *))ODEDisplace },
	{ LWDISPLACEMENT_ICLASS, "ODE_Displace",
		(int (*)(long,void * (*)(const char *,int),void *,void *))ODEDisplace_UI },
	{ NULL }
	};
}
