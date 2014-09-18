/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * VR Juggler is (C) Copyright 1998-2007 by Iowa State University
 *
 * Original Authors:
 *   Allen Bierbaum, Christopher Just,
 *   Patrick Hartling, Kevin Meinert,
 *   Carolina Cruz-Neira, Albert Baker
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#ifndef _OS_APP_H_
#define _OS_APP_H_

#include <stdlib.h>
//#include <fmod.h>
//#include <fmod_errors.h>

#if defined(__APPLE__)
#  include <OpenGL/gl.h>
#  include <OpenGL/glu.h>
#else
#  include <GL/gl.h>
#  include <GL/glu.h>
#endif

#include <gmtl/Point.h>
#include <gmtl/Sphere.h>
#include <gmtl/AABox.h>

#include <gadget/Type/PositionInterface.h>
#include <gadget/Type/DigitalInterface.h>
#include <vrj/Draw/OGL/GlApp.h>

#include <vector>

#include <btBulletDynamicsCommon.h>

//#include <vrj/Test/TestRunner.h>

#include "OBJlib.h"

class OsApp : public vrj::GlApp
{
public:
  OsApp(vrj::Kernel* kern = NULL)
    : vrj::GlApp(kern)
    , mSphereQuad(NULL)
    , mSphereIsect(false)
    , mSphereSelected(false)
    , mFramesToSleep(25)
    //, mTestRunner(NULL)
  {
    initShapes();
  }

  virtual ~OsApp()
  {
    gluDeleteQuadric(mSphereQuad);

    delete femur8;
    delete table1;
    delete table2;
    delete tableinst;
    delete moteur;
    delete meche;
    delete platte;
    delete scow;
    delete scow1;
    delete scow2;
    delete scow3;
    delete cube1;
    delete cube2;
    delete verb;
  }

  virtual void init();

  virtual void contextInit();

  virtual void preFrame();

  virtual void bufferPreDraw();

  virtual void draw();

  virtual void reset();

  void dumpState();

  /** Initialize the test runner and the associated tests.
  * This also sets up the runner to start processing tests
  */
  //void initTesting();

  //friend class SphereTestCase;     // Make friends with our test case so it can poke our internals

protected:
  /** Update any selection and grabbing information */
  void updateGrabbing();

  /** Update the state of the current navigation */
  void updateNavigation();

  /** Update any logger playback that is being used */
  void updateLogger();

private:
  void detIntersection();

  void initShapes();

  void initGLState();

  void drawSphere(const gmtl::Spheref& sphere, const bool& intersected,
                 const bool& selected);

  //void initFmod();
  //void updateListener(gmtl::Point3f position,gmtl::Point3f velocity,gmtl::Point3f forward,gmtl::Point3f up);
  //void loadSoundFmodMoteur(char *path);
  //void VolumeMoteur(gmtl::Point3f pos);
  //void SoundProp( FMOD_CHANNEL * pChannel,gmtl::Point3f centerOB,gmtl::Point3f P0);
  //void updateSound(gmtl::Point3f position,gmtl:: Point3f velocity);

  //void RenderScene();

  //void StepBulletPhysics();
  //void InitBulletPhysics();
  //void ExitBulletPhysics();

  std::vector<MeshObj*> selectable;

  gadget::PositionInterface mHead;
  gadget::PositionInterface mWand;
  gadget::DigitalInterface  mForwardButton;       /**< Button to go forward */
  gadget::DigitalInterface  mRotateButtonX;        /**< Button to rotate */
  gadget::DigitalInterface  mRotateButtonY;        /**< Button to rotate */
  gadget::DigitalInterface  mRotateButtonZ;        /**< Button to rotate */
  gadget::DigitalInterface  mGrabButton;          /**< Button to grab an object */
  gadget::DigitalInterface  mDumpStateButton;     /**< Button to dump the current state */
    
  // Sphere information
  gmtl::Spheref mSphere;                          /**< State of the sphere */
  GLUquadric*   mSphereQuad;                      /**< GL Quadric for drawing the sphere */
  bool          mSphereIsect;                     /**< If true, the wand is isecting with the sphere */
  bool          mSphereSelected;                  /**< If true, then the user is grabbing the sphere */

  gmtl::Matrix44f mNavMatrix;                   /**< Navigation matrix: world_M_virtualworld */

  gadget::DigitalInterface mLoggerPlayButton;   // Playback log file when pressed

  unsigned int mFramesToSleep;         /**< Number of frames to sleep at start up */

  MeshObj *femur8;
  MeshObj *table1;
  MeshObj *table2;
  MeshObj *tableinst;
  MeshObj *moteur;
  MeshObj *meche;
  MeshObj *platte;
  MeshObj *scow;
  MeshObj *scow1;
  MeshObj *scow2;
  MeshObj *scow3;
  MeshObj *cube1;
  MeshObj *cube2;
  MeshObj *verb;

  MeshObj *selected;

  float theta;

  //Bullet Physics
  btBroadphaseInterface *broadphase;
  btDefaultCollisionConfiguration *collisionConfiguration;
  btCollisionDispatcher *dispatcher;
  btSequentialImpulseConstraintSolver *solver;
  btDiscreteDynamicsWorld *dynamicsWorld;
  btCollisionShape *groundShape;
  btCollisionShape *fallShape;
  btRigidBody* fallRigidBody;
  btCollisionShape *tableInstShape;
  btRigidBody* tableInstRigidBody;
  std::vector<btRigidBody*> bodies;

  void addBody(btRigidBody *body)
  {
    dynamicsWorld->addRigidBody(body);
    bodies.push_back(body);
  }

  //vrj::test::TestRunner*  mTestRunner;

  /**< Test runner for this appliation */
  /* char * path;
  char * path1;
  CHAR * path2;
  FMOD_SYSTEM *system1;
  FMOD_SOUND *sound;
  FMOD_CHANNEL *channel,*channel1;
  FMOD_CHANNEL *channeln[2];
  FMOD_VECTOR velocity ;
  FMOD_CHANNELGROUP *chGroup1;
  FMOD_RESULT result;
  gmtl::Point3f posM;*/

  //FMOD_SYSTEM     *system1;
  //FMOD_SOUND      *soundC;
  //FMOD_CHANNEL    *channelC;
  //FMOD_VECTOR listenerVelocity,listenerPos, soundShperePos, soundShpereVelo,soundCubePos, soundCubeVelo,listenerUp,listenerForward;
  //char * pathC;
  //char * pathS;
  //gmtl::Point3f up,forward,position,velocity;

  //gmtl::Point3f P0c ;
};

#endif /* _OS_APP_H_ */
