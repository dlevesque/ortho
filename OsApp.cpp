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

#include <gmtl/Point.h>
#include <gmtl/Matrix.h>
#include <gmtl/Containment.h>
#include <gmtl/Generate.h>
#include <gmtl/Output.h>
#include <gmtl/Xforms.h>
#include <gmtl/Ray.h>

#include <gadget/InputManager.h>
#include <gadget/InputLogger.h>

#include <string>
#include <iostream>

#include "OsApp.h"
//#include <btBulletDynamicsCommon.h>
//#include <TestCases/FailureTestCase.h>
//#include <TestCases/SphereTestCase.h>

#include <gmtl/Intersection.h>

void OsApp::init()
{
  vrj::GlApp::init();

  mHead.init("VJHead");
  mWand.init("VJWand");

  /*mForwardButton.init("VJButton0");
  mRotateButtonX.init("VJButton1");
  mRotateButtonY.init("VJButton4");
  mRotateButtonZ.init("VJButton3");*/
  mGrabButton.init("VJButton2");

  femur8 = new MeshObj("femur8.obj");
  table1 = new MeshObj("table1.obj");
  table2 = new MeshObj("table2.obj");
  tableinst = new MeshObj("tableinst.obj");
  moteur = new MeshObj("moteur.obj");
  meche = new MeshObj("meche.obj");
  platte = new MeshObj("platte.obj");
  scow = new MeshObj("scow.obj");
  scow1 = new MeshObj("scow.obj");
  scow2 = new MeshObj("scow.obj");
  scow3 = new MeshObj("scow.obj");
  cube1 = new MeshObj("cube1.obj");
  cube2 = new MeshObj("cube1.obj");
  verb = new MeshObj("verb.obj");

  selected = 0;

  theta = 0.f;

  // Uncomment these lines when setting up logger tests
  //mDumpStateButton.init("VJButton2");
  //mLoggerPlayButton.init("LoggerPlayButton");
  meche->make_bbox();
  std::cout << meche->center[0] << ' ' << meche->center[1] << ' ' << meche->center[2] << std::endl;
  moteur->make_bbox();
  selectable.push_back(moteur);
  moteur->print();
  std::cout << moteur->center[0] << ' ' << moteur->center[1] << ' ' << moteur->center[2] << std::endl;
  platte->make_bbox();
  selectable.push_back(platte);
  scow->make_bbox();
  selectable.push_back(scow);
  scow3->make_bbox();
  selectable.push_back(scow3);
  scow1->make_bbox();
  selectable.push_back(scow1);
  scow2->make_bbox();
  selectable.push_back(scow2);
  verb->make_bbox();
  selectable.push_back(verb);
  //ahla->make_bbox();

  const gmtl::Vec3f x_axis( 1.0f, 0.0f, 0.0f );
  const gmtl::Vec3f y_axis( 0.0f, 1.0f, 0.0f );
  const gmtl::Vec3f z_axis( 0.0f, 0.0f, 1.0f );

  table1->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(230.f), y_axis)));
  table1->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-5.0f, 0.0f, 0.0f)));
  table1->addTransf(gmtl::makeScale<gmtl::Matrix44f,float>(0.06f));

  table2->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(230.f), y_axis)));
  table2->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-2.5f, 20.0f, 3.0f)));
  table2->addTransf(gmtl::makeScale<gmtl::Matrix44f,float>(0.05f));

  tableinst->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(82.f), y_axis)));
  tableinst->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(300.0f, -10.0f, 744.0f)));
  tableinst->addTransf(gmtl::makeScale<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(0.02f, 0.03f, 0.02f)));

  platte->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(79.0f), x_axis )));
  platte->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(5.0f), y_axis )));
  platte->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(30790.0f,26999.0f,-5500.0f)));
  platte->addTransf(gmtl::makeScale<gmtl::Matrix44f,float>(0.0001f));

  scow->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.f), z_axis)));
  scow->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(33000.0f,32209.0f,0.0f)));
  scow->addTransf(gmtl::makeScale<gmtl::Matrix44f, float>(0.00009f));

  scow1->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.f), z_axis)));
  scow1->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(33000.0f,32209.0f,1000.0f)));
  scow1->addTransf(gmtl::makeScale<gmtl::Matrix44f, float>(0.00009f));

  scow2->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.f), z_axis)));
  scow2->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(33000.0f,32209.0f,2000.0f)));
  scow2->addTransf(gmtl::makeScale<gmtl::Matrix44f, float>(0.00009f));

  scow3->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.f), z_axis)));
  scow3->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(33000.0f,32209.0f,-900.0f)));
  scow3->addTransf(gmtl::makeScale<gmtl::Matrix44f, float>(0.00009f));

  cube1->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(7.f), y_axis)));
  cube1->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(510.0f,750.7f,580.0f)));
  cube1->addTransf(gmtl::makeScale<gmtl::Matrix44f, float>(0.003f));

  cube2->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(7.f), y_axis)));
  cube2->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(770.0f,750.7f,560.0f)));
  cube2->addTransf(gmtl::makeScale<gmtl::Matrix44f, float>(0.003f));

  moteur->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(-10.0f), y_axis )));
  moteur->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(150.0f), z_axis )));
  moteur->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(80.0f), x_axis )));
  moteur->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(2000.9f,1750.0f,250.0f)));
  moteur->addTransf(gmtl::makeScale<gmtl::Matrix44f,float>(0.0017f));

  meche->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(-180.0f), x_axis )));
  meche->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(-35.0f), z_axis )));
  meche->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(80.0f), x_axis )));
  meche->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(131.8f, 126.f, -4.3f)));
  meche->addTransf(gmtl::makeScale<gmtl::Matrix44f,float>(0.025f));

  femur8->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(4.0f), y_axis )));
  femur8->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.0f), z_axis )));
  femur8->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(-78.0f), y_axis )));
  femur8->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(280.0f,1230.0f,2600.0f)));
  femur8->addTransf(gmtl::makeScale<gmtl::Matrix44f,float>(0.0023f));

  verb->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(55.0f), x_axis )));
  verb->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(60000.0f,48990.0f,-16000.0f)));
  verb->addTransf(gmtl::makeScale<gmtl::Matrix44f,float>(0.000057f));
}

void OsApp::contextInit()
{
  initGLState();
  /*soundFmod(path,posM);*/
}

void OsApp::preFrame()
{
  // Update the grabbing state
  updateGrabbing();
  // Update the state of navigation
  updateNavigation();

  // Update the logger state information
  updateLogger();

  //bool grab2 = (mGrabButton->getData() == gadget::Digital::ON);
  //
  //if ((sel2 && grab2)==true)
  //{
  //  loadSoundFmodMoteur(pathC);
  //}
  //else
  //{
  //  FMOD_Channel_Stop(channelC);
  //}
}

void OsApp::detIntersection()
{
  for(int i=0; i<selectable.size(); ++i)
  {
    if(gmtl::intersect(selectable[i]->getWaabox(), mSphere))
    {
      selected=selectable[i];
      return;
    }
  }
}

void OsApp::updateGrabbing()
{
  // Get wand matrix in the virtual world coordinate system
  // wand_vw = vw_M_w * wand_w
  gmtl::Matrix44f vw_M_w;
  gmtl::invert(vw_M_w, mNavMatrix);      // Nav matrix is: w_M_vw.  So invert it
  gmtl::Matrix44f wand_matrix = vw_M_w * mWand->getData();

  // Get the point in space where the wand is located
  gmtl::Point3f wand_pt = gmtl::makeTrans<gmtl::Point3f>(wand_matrix);

  mSphere.setCenter(wand_pt);
  mSphere.setRadius(0.1f); //0.02f

  gadget::Digital::State grabState = mGrabButton->getData();

  // Si on n'appuie pas sur le bouton et que rien n'�tait s�lectionn�, rien � faire
  // (pas vraiment n�cessaire �tant donn� le if qui suit...)
  //if(!grab && !selected)
  //{
  //  mSphereSelected = false;
  //  return;
  //}

  // Si on n'appuie pas sur le bouton, il faut d�selectionner et sortir
  if(grabState == gadget::Digital::OFF || grabState == gadget::Digital::TOGGLE_OFF)
  {
    //if(selected==moteur) // il faut arr�ter la rotation de la m�che
    //{
    //  theta = 0.f;
    //}
    mSphereSelected = false;
    selected = 0;
    return;
  }

  mSphereSelected = true;

  if(!selected)
  {
    //if(grabState == gadget::Digital::TOGGLE_ON)
    //{
    //  std::cout << "TOGGLE_ON ";
    //}
    if(grabState != gadget::Digital::TOGGLE_ON)
    {
      return;
    }
    // D�tecter quel objet on veut saisir
    detIntersection();
    if(!selected)
    {
      return;
    }
  }

  selected->addPostTransf(wand_matrix);

  if(selected==moteur)
  {
    theta+=100.f;
  }
}

void OsApp::updateNavigation()
{
  gmtl::Matrix44f wand_matrix = mWand->getData();      // Get the wand matrix

  // Update navigation
  // - Find forward direction of wand
  // - Translate along that direction
  float velocity(0.07f);

  if(mForwardButton->getData())
  {
    gmtl::Vec3f z_dir = gmtl::Vec3f(0.0f, 0.0f, velocity);
    gmtl::Vec3f dir(wand_matrix * z_dir);
    gmtl::preMult(mNavMatrix, gmtl::makeTrans<gmtl::Matrix44f>(dir));
  }   

  if(mRotateButtonX->getData())
  {
    const float rot_scale(0.01f);
    float x_rot = gmtl::makeXRot<float, 4, 4>(wand_matrix);
    float rotationX = -1.0f * x_rot * rot_scale;
    gmtl::preMult(mNavMatrix,
      gmtl::makeRot<gmtl::Matrix44f>(gmtl::EulerAngleXYZf(rotationX,0.0,0.0)));
  }
  if(mRotateButtonY->getData())
  {
    const float rot_scale(0.01f);
    float y_rot = gmtl::makeYRot<float, 4, 4>(wand_matrix);
    float rotationY = -1.0f * y_rot * rot_scale;
    gmtl::preMult(mNavMatrix,
      gmtl::makeRot<gmtl::Matrix44f>(gmtl::EulerAngleXYZf(0.0,rotationY,0.0)));
  }
  if(mRotateButtonZ->getData())
  {
    const float rot_scale(0.01f);
    float z_rot = gmtl::makeZRot<float, 4, 4>(wand_matrix);
    float rotationZ = -1.0f * z_rot * rot_scale;
    gmtl::preMult(mNavMatrix,
      gmtl::makeRot<gmtl::Matrix44f>(gmtl::EulerAngleXYZf(0.0,0.0,rotationZ)));
  }

  // ---- RESET ---- //
  // If the reset button is pressed, reset the state of the application.
  // This button takes precedence over all others.
  if ( mForwardButton->getData() && mRotateButtonY->getData())
  {
    this->reset();
  }
}

void OsApp::updateLogger()
{
   // If the dump state button has just been pressed, dump the current state
   // information.
   if ( mDumpStateButton->getData() == gadget::Digital::TOGGLE_ON )
   {
      dumpState();
   }

   // Playing of logs
   // Check logger play button
   if(mLoggerPlayButton->getData() == gadget::Digital::TOGGLE_ON)
   {
      std::cout << "\n\n------ Log Play Button hit ----\n" << std::flush;
      gadget::InputManager* input_mgr = gadget::InputManager::instance();
      gadget::InputLoggerPtr logger = input_mgr->getInputLogger();

      std::string log_filename;
      std::cout << "Enter filename to load:" << std::flush;
      std::cin >> log_filename;
      std::cout << std::endl << "Using file:" << log_filename << std::endl;

      vprASSERT(logger.get() != NULL);
      logger->load(log_filename);
      logger->play();
   }

   // Sleep several frames before starting up the logger
   if(mFramesToSleep != 0)
   {
      mFramesToSleep--;
      std::cout << "mFramesToSleep: " << mFramesToSleep << std::endl;
   }
   //else if((NULL != mTestRunner) && (mTestRunner->getState() == vrj::test::TestRunner::Processing) )
   //{
   //   mTestRunner->processTests();
   //}
}

void OsApp::bufferPreDraw()
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT);
}

//btAlignedObjectArray<btCollisionShape*>	m_collisionShapes; //keep the collision shapes, for deletion/cleanup
//btBroadphaseInterface*					m_broadphase;
//btCollisionDispatcher*					m_dispatcher;
//btConstraintSolver*						m_solver;
//btDefaultCollisionConfiguration*		m_collisionConfiguration;
//btDynamicsWorld*						m_dynamicsWorld; //this is the most important class
//
//void OsApp::StepBulletPhysics()
//{
//	if(m_dynamicsWorld)//step the simulation
//		m_dynamicsWorld->stepSimulation(1.0f/2000.0f);
//}
//
//void OsApp::InitBulletPhysics()
//{
//
//	m_collisionConfiguration = new btDefaultCollisionConfiguration(); //collision configuration contains default setup for memory, collision setup
//	m_dispatcher			 = new btCollisionDispatcher(m_collisionConfiguration); //use the default collision dispatcher 
//	m_broadphase		  	 = new btDbvtBroadphase();
//	m_solver			 	 = new btSequentialImpulseConstraintSolver; //the default constraint solver
//	m_dynamicsWorld			 = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
//	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
//
//	
//	
//	//Creating a static shape which will act as ground
//	{
//		btCollisionShape* groundShape = new btBoxShape(btVector3(50,50,50));
//		// btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
//		m_collisionShapes.push_back(groundShape);
//		
//		btScalar mass = 0; //rigidbody is static if mass is zero, otherwise dynamic
//		btVector3 localInertia(0,0,0);
//
//		groundShape->calculateLocalInertia(mass,localInertia);
//
//		btTransform groundTransform;
//		groundTransform.setIdentity();
//		groundTransform.setOrigin(btVector3(0,-45,0));
//
//		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform); //motionstate provides interpolation capabilities, and only synchronizes 'active' objects
//		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
//		btRigidBody* body = new btRigidBody(rbInfo);
//		
//		m_dynamicsWorld->addRigidBody(body); //add the body to the dynamics world
//	}
//
//
//	//Creating some dynamic boxes
//	{
//		btCollisionShape* boxShape = new btBoxShape(btVector3(0.5,0.5,0.5));
//		m_collisionShapes.push_back(boxShape);
//
//		btScalar mass = 1.0f;
//		btVector3 localInertia(0,0,0);
//
//		boxShape->calculateLocalInertia(mass,localInertia);
//
//		btTransform startTransform;
//		startTransform.setIdentity();
//		startTransform.setOrigin(btVector3(5,20,0));
//
//		
//		for(int j=0; j<1; j++)
//		{
//			startTransform.setOrigin(btVector3(9,(j*10)+12 ,0));
//
//			btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
//			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,boxShape,localInertia);
//			btRigidBody* body = new btRigidBody(rbInfo);
//
//			m_dynamicsWorld->addRigidBody(body);
//		}
//
//	}
//
//}
//
//void OsApp::ExitBulletPhysics()
//{
//	//cleanup in the reverse order of creation/initialization
//
//	for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0; i--) //remove the rigidbodies from the dynamics world and delete them
//	{
//		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
//		btRigidBody* body = btRigidBody::upcast(obj);
//		if(body && body->getMotionState())
//			delete body->getMotionState();
//		
//		m_dynamicsWorld->removeCollisionObject(obj);
//		delete obj;
//	}
//
//	
//	for (int j=0; j<m_collisionShapes.size(); j++) //delete collision shapes
//	{
//		btCollisionShape* shape = m_collisionShapes[j];
//		delete shape;
//	}
//	
//	m_collisionShapes.clear();
//	delete m_dynamicsWorld;
//	delete m_solver;
//	delete m_broadphase;
//	delete m_dispatcher;
//	delete m_collisionConfiguration;
//}

void OsApp::draw()
{
  glClear(GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  if (mFramesToSleep > 0) return;

  glPushMatrix();
    glMultMatrixf(table1->getTrans().mData);
    table1->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(table2->getTrans().mData);
    table2->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(tableinst->getTrans().mData);
    tableinst->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(platte->getPostTransf().mData);
    glMultMatrixf(platte->getTrans().mData);
    platte->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(scow->getPostTransf().mData);
    glMultMatrixf(scow->getTrans().mData);
    scow->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(scow3->getPostTransf().mData);
    glMultMatrixf(scow3->getTrans().mData);
    scow3->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(scow1->getPostTransf().mData);
    glMultMatrixf(scow1->getTrans().mData);
    scow1->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(scow2->getPostTransf().mData);
    glMultMatrixf(scow2->getTrans().mData);
    scow2->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(cube1->getTrans().mData);
    cube1->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(cube2->getTrans().mData);
    cube2->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(moteur->getPostTransf().mData);
    glPushMatrix();
      glMultMatrixf(moteur->getTrans().mData);
      moteur->draw_model();
    glPopMatrix();
    glPushMatrix();
      glMultMatrixf(meche->getTrans().mData);
      glRotatef(theta,0.f,1.f,0.f);
      meche->draw_model();
    glPopMatrix();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(femur8->getTrans().mData);
    femur8->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(verb->getPostTransf().mData);
    glMultMatrixf(verb->getTrans().mData);
    verb->draw_model(); 
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(mNavMatrix.mData);
    drawSphere(mSphere, mSphereIsect, mSphereSelected);
  glPopMatrix();

  //moteur->draw_waabox();
}

//void OsApp::initFmod()
//{ unsigned int length;
//	int t=0;
//  
//	FMOD_System_Create(&system1);
////nombre de haut parleurs utilises 
// // FMOD_System_SetSoftwareFormat(system1,48000,FMOD_SOUND_FORMAT_PCMFLOAT,8,2,FMOD_DSP_RESAMPLER_LINEAR);
//
//	//initialisation du systeme
//    FMOD_System_Init(system1, 100, FMOD_INIT_3D_RIGHTHANDED, NULL);
//	
//	//position initiale du listener
//	
//	gmtl::Matrix44f h;
//   gmtl::invert(h, mNavMatrix);      // Nav matrix is: w_M_vw.  So invert it
//   gmtl::Matrix44f head_matrix = h * mHead->getData();
//
//   // Get the point in space where the Listener is located.
//   P0c = gmtl::makeTrans<gmtl::Point3f>(head_matrix)-mCube.getMax();
//	//creation du systeme 
//	
//	FMOD_System_Set3DSettings(system1, 10.0f, 10.0f, 10.0f);
//  FMOD_System_SetSpeakerMode (system1,FMOD_SPEAKERMODE_RAW);
//	//creation du son du cube
//	FMOD_System_CreateSound(system1,pathC,FMOD_3D|FMOD_LOOP_NORMAL|FMOD_CREATESAMPLE,0, &soundC);
//	FMOD_Sound_Set3DMinMaxDistance(soundC, 1.f, 5000.0f);
//	FMOD_Sound_GetLength(soundC,&length, FMOD_TIMEUNIT_PCM);
//FMOD_Channel_SetSpeakerMix(channelC,0,0,0,0,0,0,0,0);
//float levels[2] ={0,1.0f};
//FMOD_Channel_SetSpeakerMix(channelC,1.0f,1.0f,0,0,0,0,0,0);
//
//}
//
//
//void OsApp::VolumeMoteur(gmtl::Point3f pos)
//{	float	volC;
//	float distanceLC=0;
//	float distMax =17.2f;	
//	volC=1;
//	FMOD_VECTOR posC = { pos[0], pos[1],pos[2] };
//    FMOD_VECTOR velC = {  0.0f, 0.0f, 0.0f };
//	FMOD_Channel_Set3DAttributes(channelC, &posC, &velC);
//
//	// Get wand matrix in the virtual world coordinate system
//   // wand_vw = vw_M_w * wand_w
//   gmtl::Matrix44f h;
//   gmtl::invert(h, mNavMatrix);      // Nav matrix is: w_M_vw.  So invert it
//   gmtl::Matrix44f head_matrix = h * mHead->getData();
//
//   // Get the point in space where the Listener is located.
//   gmtl::Point3f head_pt = gmtl::makeTrans<gmtl::Point3f>(head_matrix);
//  //distance entre Listener et la sphere : 
//
//   distanceLC= sqrt(pow((head_pt[0]- pos[0]),2) + pow((head_pt[1]- pos[1]),2)+ pow((head_pt[2]- pos[2]),2));
//   //std::cout<<distanceLC;
//   //std::cout<<"**";
//   volC= 1-(distanceLC/distMax);
//
//FMOD_Channel_SetVolume(channelC,volC);
//}
//
//void OsApp::SoundProp( FMOD_CHANNEL * pChannel,gmtl::Point3f centerOB,gmtl::Point3f P0)
//{ float distance,vl,beta,DF;
//	float fDE;
//	float f= 450;
//	float c= 2999729458;
//	float v;
//	v= 343,3;
//	   gmtl::Matrix44f h;
//	   gmtl::invert(h, mNavMatrix);      // Nav matrix is: w_M_vw.  So invert it
//	   gmtl::Matrix44f head_matrix = h * mHead->getData();
//   // Get the point in space where the Listener is located.
//	   gmtl::Point3f P1 = gmtl::makeTrans<gmtl::Point3f>(head_matrix)-centerOB;
//	//****************
//	   
//
//   // Calcul de la distance = relative a l'objet entre 2 instance diff (deplacement)
//   distance= sqrt(pow((P1[0]- P0[0]),2) + pow((P1[1]- P0[1]),2)+ pow((P1[2]- P0[2]),2));
//	vl=distance*60;
//	//std::cout<<distance;
//	//std::cout<<"*****";
//	beta=vl/c;
//	DF= sqrt((1+beta)/(1-beta));
//	fDE =DF * f * ((v-vl)/v);
//  
//   FMOD_Channel_Set3DDopplerLevel(pChannel,fDE);
//    P0=P1;
//}
//
//
//void OsApp::loadSoundFmodMoteur(char *path)
//{ 
//	float freqC;
//	//Cube sound
//	FMOD_System_PlaySound(system1, FMOD_CHANNEL_FREE, soundC, FALSE, &channelC);
//	FMOD_Channel_GetFrequency (channelC,&freqC);
// // FMOD_Channel_SetSpeakerLevels(channelC,FMOD_SPEAKER_FRONT_LEFT,levels,2);
//}
//
//
//
//
//void OsApp::updateListener(gmtl::Point3f position,gmtl::Point3f velocity,gmtl::Point3f forward,gmtl::Point3f up)
//{
//	listenerVelocity.x = velocity[0];
//	listenerVelocity.y =velocity[1];
//	listenerVelocity.z = velocity[2];
//
//	listenerPos.x = position[0];
//	listenerPos.y = position[1];
//	listenerPos.z = position[2];
//	
//	listenerUp.x= up[0];
//	listenerUp.y= up[1];
//	listenerUp.z= up[2];
//
//	listenerForward.x=forward[0];
//	listenerForward.y=forward[1];
//	listenerForward.z=forward[2];
//	
//	FMOD_System_Set3DListenerAttributes(system1, 0, &listenerPos, &listenerVelocity,&listenerForward,&listenerUp);
//}
//
//void OsApp::updateSound( gmtl::Point3f position, gmtl::Point3f velocity )
//{
//	soundShperePos.x = position[0];
//    soundShperePos.y = position[1];
//    soundShperePos.z = position[2];
//
//    soundShpereVelo.x = velocity[0];
//    soundShpereVelo.y = velocity[1];
//    soundShpereVelo.z = velocity[2];
//
//
//}

void OsApp::reset()
{
   // Initialize the sphere.  This creates a sphere with radius 1 and center
   // (2.5, 3, -2.5).
   gmtl::Point3f sphere_center = gmtl::Point3f(2.5f, 3.0f, -2.5f);
   mSphere.setCenter(sphere_center);
   mSphere.setRadius(1.0f);

   //// Initialize the cube.  This creates a cube with sides of length 2 centered
   //// at (-2.5, 3, -2.5).
   //mCubeEdgeLength = 0.75f;
   //gmtl::Point3f cube_center(-2.5, 3, -2.5);
   //gmtl::Point3f corner_offset(mCubeEdgeLength, mCubeEdgeLength, mCubeEdgeLength);
   //mCube.setMin(cube_center - corner_offset);   // Bottom rear left corner
   //mCube.setMax(cube_center + corner_offset);   // Top front right corner
   //mCube.setEmpty(false);
   
   mSphereSelected = false;
   //mCubeSelected   = false;

   gmtl::identity(mNavMatrix);      // Set Nav matrix to identity
}

void OsApp::dumpState()
{
   vprDEBUG(vprDBG_ALL, vprDBG_CRITICAL_LVL)
      << "Sphere -- center: " << mSphere.getCenter() << " ## radius: "
      << mSphere.getRadius() << std::endl << vprDEBUG_FLUSH;
   //vprDEBUG(vprDBG_ALL, vprDBG_CRITICAL_LVL)
   //   << "Cube -- min: " << mCube.getMin() << " ## max: " << mCube.getMax()
   //   << std::endl << vprDEBUG_FLUSH;
}

/** Initialize the test runner and the associated tests.
* This also sets up the runner to start processing tests
*/
//void OsApp::initTesting()
//{
//   mTestRunner = new vrj::test::TestRunner;
//
//   //mTestRunner->addTest(new SphereTestCase);
//   //mTestRunner->addTest(new FailureTestCase);
//      
//   mTestRunner->initialize(this);
//   //ExitBulletPhysics();
//}

void OsApp::initShapes()
{
   // Set to initial shape positions
   reset();

   // Allocate a new quadric that will be used to render the sphere.
   mSphereQuad = gluNewQuadric();
}

void OsApp::initGLState()
{
  //pathC= "C:\\Users\\demo\\Desktop\\cabana\\RecordedSound\\M.mp3";
  //pathS="";

  //initFmod();
  //InitBulletPhysics();

  GLfloat light0_ambient[]  = { 0.1f,  0.1f,  0.1f, 1.0f };
  GLfloat light0_diffuse[]  = { 0.8f,  0.8f,  0.8f, 1.0f };
  GLfloat light0_specular[] = { 1.0f,  1.0f,  1.0f, 1.0f };
  GLfloat light0_position[] = { 0.0f, 0.75f, 0.75f, 0.0f };

  GLfloat mat_ambient[]   = { 0.7f, 0.7f, 0.7f, 1.0f };
  GLfloat mat_diffuse[]   = { 1.0f, 0.5f, 0.8f, 1.0f };
  GLfloat mat_specular[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat mat_shininess[] = { 50.0f };
  //GLfloat mat_emission[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat no_mat[]        = { 0.0f, 0.0f, 0.0f, 1.0f };

  glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

  glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
  glMaterialfv(GL_FRONT, GL_EMISSION, no_mat);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_COLOR_MATERIAL);
  glShadeModel(GL_SMOOTH);
}

void OsApp::drawSphere(const gmtl::Spheref& sphere,
                            const bool& intersected, const bool& selected)
{
   gmtl::Point3f sphere_center = sphere.getCenter();

   glPushMatrix();
      // Set the rendering color based on the state of the sphere with respect
      // to the wand.
      if ( selected )
      {
         glColor3f(1.0f, 1.0f, 0.0f);
      }
      else if ( intersected )
      {
         glColor3f(1.0f, 0.0f, 0.5f);
      }
      else
      {
         glColor3f(1.0f, 0.0f, 0.0f);
      }

      // Rener the sphere.
      glTranslatef(sphere_center[0], sphere_center[1], sphere_center[2]);
      gluSphere(mSphereQuad, sphere.getRadius(), 15, 15);
   glPopMatrix();
}
