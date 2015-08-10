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
#include <fmod.hpp>
#include <fmod_errors.h>

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

#include <vhtBase.h>
#include <vhtCore.h>

#include <btBulletDynamicsCommon.h>

//#include <vrj/Test/TestRunner.h>

#include "OBJlib.h"

#include <cd_wavefront.h>

#include <string>

class OsApp : public vrj::GlApp
{
public:
  OsApp(vrj::Kernel* kern = NULL)
    : vrj::GlApp(kern)
    , mSphereQuad(NULL)
    , mSphereIsect(false)
    , mSphereSelected(false)
    , mFramesToSleep(25)
    , x_axis(gmtl::Vec3f(1.f,0.f,0.f))
    , y_axis(gmtl::Vec3f(0.f,1.f,0.f))
    , z_axis(gmtl::Vec3f(0.f,0.f,1.f))
    , m_chemin("C:/Users/Cave/Desktop/hand model files/")
    , m_scaleMain(0.025f)
    , m_scaleMotion(1.f)
  {
    initShapes();
  }

  virtual ~OsApp()
  {
    gluDeleteQuadric(mSphereQuad);

    exitCGS();

    selectable.clear();
    fallRigidBodies.clear();

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
    delete tournevis;
    delete cube1;
    delete cube2;
    delete verb;

    for(int i=dynamicsWorld->getNumConstraints()-1; i>=0; --i)
    {
      btTypedConstraint *contr = dynamicsWorld->getConstraint(i);
      dynamicsWorld->removeConstraint(contr);
      delete contr;
    }

    for(unsigned int i=0; i<bodies.size(); ++i)
    {
      dynamicsWorld->removeCollisionObject(bodies[i]);
      btMotionState *motionState = bodies[i]->getMotionState();
      btCollisionShape *shape = bodies[i]->getCollisionShape();
      delete bodies[i];
      delete shape;
      delete motionState;
    }

    bodies.clear();

    //à ajouter : libération de la mémoire des maillages (trimesh)
    delete trimesh;
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfiguration;
    delete broadphase;
  }

  virtual void init();

  virtual void contextInit();

  virtual void preFrame();

  virtual void bufferPreDraw();

  virtual void draw();

  virtual void reset();

  void dumpState();

  static bool callbackFunc(btManifoldPoint &cp, const btCollisionObjectWrapper *obj1, int id1, int index1, const btCollisionObjectWrapper *obj2, int id2, int index2);
  bool mcallbackFunc(btManifoldPoint &cp, const btCollisionObjectWrapper *obj1, int id1, int index1, const btCollisionObjectWrapper *obj2, int id2, int index2);

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
  static OsApp *thisApp;

  void detIntersection(const btVector3&);

  void initShapes();

  void initGLState();

  void drawSphere(const gmtl::Spheref& sphere, const bool& intersected,
                 const bool& selected);

  void initFmod();
  //void updateListener(gmtl::Point3f position,gmtl::Point3f velocity,gmtl::Point3f forward,gmtl::Point3f up);
  //void loadSoundFmodMoteur(char *path);
  //void VolumeMoteur(gmtl::Point3f pos);
  //void SoundProp( FMOD_CHANNEL * pChannel,gmtl::Point3f centerOB,gmtl::Point3f P0);
  //void updateSound(gmtl::Point3f position,gmtl:: Point3f velocity);

  //void RenderScene();

  //void StepBulletPhysics();
  //void InitBulletPhysics();
  //void ExitBulletPhysics();

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

  float theta;

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
  MeshObj *tournevis;
  MeshObj *cube1;
  MeshObj *cube2;
  MeshObj *verb;

  MeshObj *selected;

  std::vector<MeshObj*> selectable;

  //Bullet Physics
  btBroadphaseInterface *broadphase;
  btDefaultCollisionConfiguration *collisionConfiguration;
  btCollisionDispatcher *dispatcher;
  btSequentialImpulseConstraintSolver *solver;
  btDiscreteDynamicsWorld *dynamicsWorld;

  btTypedConstraint *m_pickConstraint;

  std::vector<btRigidBody*> bodies;
  std::vector<btRigidBody*> fallRigidBodies;

  void addBody(btRigidBody *body)
  {
    dynamicsWorld->addRigidBody(body);
    bodies.push_back(body);
  }

  btVector3 gmtl2btVector(const gmtl::Point3f& p) const
  {
    return btVector3(btScalar(p.mData[0]), btScalar(p.mData[1]), btScalar(p.mData[2]));
  }

  gmtl::Point3f btVector2gmtlPoint(const btVector3& v) const
  {
    return gmtl::Point3f(v.x(),v.y(),v.z());
  }

  //btMatrix3x3 gmtl44rot2bt33rot(const gmtl::Matrix44f& m) const
  //{
  //  return btMatrix3x3(m(0,0), m(0,1), m(0,2),
  //                      m(1,0), m(1,1), m(1,2),
  //                      m(2,0), m(2,1), m(2,2));
  //}

  void initPhysics(); //initialisation des variables Bullet
  void initCGS();      //initialisation du CyberGloveSystem
  void exitCGS();

  void addGround();

  void addFemur();
  void addTable1();
  void addTable2();
  void addTableInst();
  //void addMoteur();
  //void addMeche();
  void addPerceuse();
  void addPlaque();
  void addVis();
  void addVis1();
  void addVis2();
  void addVis3();
  void addTournevis();
  void addCube1();
  void addCube2();
  void addVerb();

  btTriangleMesh *trimesh;
  void buildTrimesh(const ConvexDecomposition::WavefrontObj &wo, btTriangleMesh *trimesh, const btVector3 &centre, const btVector3 &scaling);

  const gmtl::Vec3f x_axis;
  const gmtl::Vec3f y_axis;
  const gmtl::Vec3f z_axis;

  FMOD::System *fmodSystem;
  FMOD::Sound *sonPerceuse;
  FMOD::Channel *fmodCanal;

  bool perceuseOn;

  //variables relatives au CyberGloveSystem
  vhtCFHumanHand *m_cgsMainDroite;
  vhtCyberGlove  *m_cgsGantDroit;
  vhtTracker     *m_cgsForceDroite;
  vhtCyberGrasp  *m_cgsGraspDroit;
  vht6DofDevice  *m_cgsRcvrDroit;
  vhtHandMaster  *m_cgsMasterDroit;

  enum doigts {pau = 0, po1, po2, po3, in1, in2, in3, ma1, ma2, ma3, an1, an2, an3, au1, au2, au3, MAX_DOIGTS};
  std::string    m_chemin; //chemin des fichiers .obj des parties de la main
  std::string    m_fichier[MAX_DOIGTS];
  float          m_scaleMain;
  float          m_scaleMotion;
  float          m_penet[MAX_DOIGTS];
  MeshObj       *m_mainDroiteMeshObj[MAX_DOIGTS];
  btTriangleMesh m_trimesh[MAX_DOIGTS];
  btRigidBody   *m_mainDroiteRB[MAX_DOIGTS];

  btGeneric6DofConstraint *m_dof6[MAX_DOIGTS];
  btConeTwistConstraint   *m_pouce;
  btHingeConstraint       *m_hinge[MAX_DOIGTS];

  void addMainDroite();
  void addPhalange(enum doigts phal, bool mainGauche = false);
  void drawMainDroite();
  void drawPhalange(enum doigts phal, bool mainGauche = false);
  void addContraintes();
  void addHingeContrainte(enum doigts, enum doigts, const btVector3&, const btVector3&);

  void updateForces();
  void setHingeLimit(enum doigts phal, double angle, bool mainGauche = false)
  {
    m_hinge[phal]->setLimit(angle,angle);
  }
};

#endif /* _OS_APP_H_ */
