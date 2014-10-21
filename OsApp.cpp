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

#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

class GlDrawcallback : public btTriangleCallback
{

public:

	bool	m_wireframe;

	GlDrawcallback()
		:m_wireframe(false)
	{
	}

	virtual void processTriangle(btVector3* triangle,int partId, int triangleIndex)
	{

		(void)triangleIndex;
		(void)partId;


		if (m_wireframe)
		{
			glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 1, 0);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 0, 1);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glEnd();
		} else
		{
			glBegin(GL_TRIANGLES);
			//glColor3f(1, 1, 1);
			
			
			//glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			//glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			//glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());

			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glEnd();
		}
	}
};

OsApp* OsApp::thisApp = 0;

void OsApp::init()
{
  vrj::GlApp::init();

  thisApp = this;

  mHead.init("VJHead");
  mWand.init("VJWand");

  mForwardButton.init("VJButton0");
  mRotateButtonX.init("VJButton1");
  mRotateButtonY.init("VJButton4");
  mRotateButtonZ.init("VJButton3");
  mGrabButton.init("VJButton2");

  initPhysics();
  addGround();

  addFemur();
  addTable1();
  addTable2();
  addTableInst();
  addPerceuse(); // moteur et mèche
  addPlaque();
  addVis();
  addVis1();
  addVis2();
  addVis3();
  addCube1();
  addCube2();
  addVerb();

  btCollisionDispatcher *disp = static_cast<btCollisionDispatcher*>(dynamicsWorld->getDispatcher());
  btGImpactCollisionAlgorithm::registerAlgorithm(disp);

  selected = 0;

  theta = 0.f;

  // Uncomment these lines when setting up logger tests
  //mDumpStateButton.init("VJButton2");
  //mLoggerPlayButton.init("LoggerPlayButton");

  //ahla->make_bbox();

  //// Qu'il neige !
  //btCollisionShape *fallSphereShape = new btSphereShape(0.02);
  //btScalar sphereMass = 0.1f;
  //btVector3 fallSphereInertia(0, 0, 0);
  //fallSphereShape->calculateLocalInertia(sphereMass, fallSphereInertia);
  //for(float x = -2.f; x<2.1f; x+=0.1)
  //{
  //  for(float z = -2.f; z<2.1f; z+=0.1)
  //  {
  //    btDefaultMotionState* fallMotionState =
  //                  new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(x, 10.f, z)));
  //    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(sphereMass, fallMotionState, fallSphereShape, fallSphereInertia);
  //    btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  //    fallRigidBodies.push_back(fallRigidBody);
  //    addBody(fallRigidBody);
  //  }
  //}
}

void OsApp::contextInit()
{
  initGLState();
  /*soundFmod(path,posM);*/
}

bool OsApp::callbackFunc(btManifoldPoint &cp, const btCollisionObjectWrapper *obj1, int id1, int index1, const btCollisionObjectWrapper *obj2, int id2, int index2)
{
  return thisApp->mcallbackFunc(cp,obj1,id1,index1,obj2,id2,index2);
}

bool OsApp::mcallbackFunc(btManifoldPoint &cp, const btCollisionObjectWrapper *obj1, int id1, int index1, const btCollisionObjectWrapper *obj2, int id2, int index2)
{
  const btCollisionObject *o1 = obj1->getCollisionObject();
  const btCollisionObject *o2 = obj2->getCollisionObject();
  
  if(o1 == femur8->getBody())
    std::cout << "o1 == femur " << id1 << " " << index1 << std::endl;
  //if(o1 == platte->getBody())
  //  std::cout << "o1 == plaque" << std::endl;
  if(o2 == femur8->getBody())
    std::cout << "o2 == femur" << id1 << " " << index1 << std::endl;
  //if(o2 == platte->getBody())
  //  std::cout << "o2 == plaque" << std::endl;
  return false;
}

void OsApp::preFrame()
{
  dynamicsWorld->stepSimulation(1./60.,10,1./600.);
  //dynamicsWorld->stepSimulation(1./60.);

  //std::cout << mForwardButton->getData() << ' ';
  //std::cout << mRotateButtonX->getData() << ' ';
  //std::cout << mGrabButton->getData() << ' ';
  //std::cout << mRotateButtonZ->getData() << ' ';
  //std::cout << mRotateButtonY->getData() << ' ';
  //std::cout << std::endl;

  if(mRotateButtonX->getData() == gadget::Digital::TOGGLE_OFF)
  {
    for(int i=0; i<selectable.size(); ++i)
    {
      if(selectable[i]->getBody())
      {
        selectable[i]->getBody()->setActivationState(DISABLE_DEACTIVATION);
        selectable[i]->resetBody();
        selectable[i]->getBody()->setAngularVelocity(btVector3(0,0,0));
        selectable[i]->getBody()->setLinearVelocity(btVector3(0,0,0));
        selectable[i]->getBody()->forceActivationState(ACTIVE_TAG);
        selectable[i]->getBody()->setDeactivationTime(0.f);
      }
    }
  }

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

void OsApp::detIntersection(const btVector3& pos)
{
  btVector3 aabbMin, aabbMax;
  for(int i=0; i<selectable.size(); ++i)
  {
    bool intersect;
    if(selectable[i]->getBody())
    {
      selectable[i]->getBody()->getAabb(aabbMin,aabbMax);
      intersect = gmtl::intersect(gmtl::AABoxf(btVector2gmtlPoint(aabbMin),btVector2gmtlPoint(aabbMax)), mSphere);
    }
    else
    {
      intersect = gmtl::intersect(selectable[i]->getWaabox(), mSphere);
    }
    if(intersect)
    {
      std::cout << "intersection ";
      selected=selectable[i];
      if(selected->getBody())
      {
        std::cout << "constraint ";
        selected->getBody()->setActivationState(DISABLE_DEACTIVATION);
        btVector3 pivot = selected->getBody()->getCenterOfMassTransform().inverse() * pos;
        //btPoint2PointConstraint *p2p = new btPoint2PointConstraint(*(selected->getBody()), pivot);
        //dynamicsWorld->addConstraint(p2p,true);
        //m_pickConstraint = p2p;
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(pivot);
        btGeneric6DofConstraint *dof6 = new btGeneric6DofConstraint(*(selected->getBody()),tr,false);
        dof6->setLinearLowerLimit(btVector3(0,0,0));
        dof6->setLinearUpperLimit(btVector3(0,0,0));
        dof6->setAngularLowerLimit(btVector3(0,0,0));
        dof6->setAngularUpperLimit(btVector3(0,0,0));
        dynamicsWorld->addConstraint(dof6,true);
        m_pickConstraint = dof6;
      }
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
  gmtl::Quat<float> rot = gmtl::makeRot<gmtl::Quat<float> >(wand_matrix);
  static gmtl::Point3f old_wand_pt;
  static gmtl::Quat<float> init_rot;

  mSphere.setCenter(wand_pt);
  mSphere.setRadius(0.1f); //0.02f

  gadget::Digital::State grabState = mGrabButton->getData();

  // Si on n'appuie pas sur le bouton, il faut déselectionner et sortir
  if(grabState == gadget::Digital::OFF || grabState == gadget::Digital::TOGGLE_OFF)
  {
    //if(selected==moteur) // il faut arrêter la rotation de la mèche
    //{
    //  theta = 0.f;
    //}
    mSphereSelected = false;
    if(m_pickConstraint)
    {
      dynamicsWorld->removeConstraint(m_pickConstraint);
      delete m_pickConstraint;
      m_pickConstraint = 0;
      std::cout << "drop ";
      selected->getBody()->forceActivationState(ACTIVE_TAG);
      selected->getBody()->setDeactivationTime(0.f);
    }
    selected = 0;
    return;
  }

  mSphereSelected = true;

  if(!selected)
  {
    if(grabState != gadget::Digital::TOGGLE_ON)
    {
      return;
    }
    // Détecter quel objet on veut saisir
    detIntersection(gmtl2btVector(wand_pt));
    if(!selected)
    {
      return;
    }
    old_wand_pt = wand_pt;
    if(m_pickConstraint)
    {
      btGeneric6DofConstraint *pc = static_cast<btGeneric6DofConstraint*>(m_pickConstraint);
      btQuaternion q = pc->getFrameOffsetA().getRotation();
      btQuaternion r(rot[0],rot[1],rot[2],rot[3]);
      btQuaternion init = r.inverse() * q;
      init_rot = gmtl::Quat<float>(init.x(),init.y(),init.z(),init.w());
    }
  }

  if(m_pickConstraint)
  {
//#if 1
//    btPoint2PointConstraint *pickCon = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
//    //btVector3 pivot = selected->getBody()->getCenterOfMassTransform().inverse() * gmtl2btVector(wand_pt);
//    btVector3 pivot = gmtl2btVector(wand_pt);
//    pickCon->setPivotB(pivot);
//#else
//    btVector3 oldOrg = selected->getBody()->getWorldTransform().getOrigin();
//    btVector3 newOrg = oldOrg + gmtl2btVector(wand_pt - old_wand_pt);
//    selected->getBody()->getWorldTransform().setOrigin(newOrg);
//    old_wand_pt = wand_pt;
//#endif
    btGeneric6DofConstraint *pickCon = static_cast<btGeneric6DofConstraint*>(m_pickConstraint);
    btVector3 pivot = gmtl2btVector(wand_pt);
    pickCon->getFrameOffsetA().setOrigin(pivot);
    gmtl::Quat<float> drot = rot * init_rot;
    pickCon->getFrameOffsetA().setRotation(btQuaternion(drot[0],drot[1],drot[2],drot[3]));
    //pickCon->getFrameOffsetA().setRotation(btQuaternion(0,0,0,1));
  }
  else
  {
    selected->addPostTransf(wand_matrix);
  }

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

void OsApp::draw()
{
  glClear(GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  if (mFramesToSleep > 0) return;

  float mat[16];
  btTransform t;

  glPushMatrix();

  glMultMatrixf(mNavMatrix.mData);

  // plancher
  glPushMatrix();
    glColor3f(0.1f, 0.1f, 0.1f);
    glBegin(GL_QUADS);
      glVertex3f(-100.f, 0.f, -100.f);
      glVertex3f(-100.f, 0.f,  100.f);
      glVertex3f( 100.f, 0.f,  100.f);
      glVertex3f( 100.f, 0.f, -100.f);
    glEnd();
  glPopMatrix();

  //glPushMatrix();
  //  glColor3f(1.0f, 0.0f, 0.0f);
  //  gluSphere(mSphereQuad, 1.0f, 15, 15);
  //glPopMatrix();

  //glPushMatrix();
  //  glColor3f(0.0f, 1.0f, 0.0f);
  //  glTranslatef(10.0f, 0.0f, 0.0f);
  //  gluSphere(mSphereQuad, 1.0f, 15, 15);
  //glPopMatrix();

  //glPushMatrix();
  //  glColor3f(0.0f, 0.0f, 1.0f);
  //  glTranslatef(10.0f, 10.0f, 0.0f);
  //  gluSphere(mSphereQuad, 1.0f, 15, 15);
  //glPopMatrix();

  //glPushMatrix();
  //  glColor3f(1.0f, 1.0f, 0.0f);
  //  glTranslatef(0.0f, 10.0f, 0.0f);
  //  gluSphere(mSphereQuad, 1.0f, 15, 15);
  //glPopMatrix();

  //glPushMatrix();
  //  glColor3f(0.01f, 0.01f, 0.01f);
  //  glTranslatef(0.0f, 0.0f, 10.0f);
  //  gluSphere(mSphereQuad, 1.0f, 115, 115);
  //glPopMatrix();

  //glPushMatrix();
  //  glColor3f(0.2f, 0.2f, 0.2f);
  //  glTranslatef(0.0f, 10.0f, 10.0f);
  //  gluSphere(mSphereQuad, 1.0f, 15, 15);
  //glPopMatrix();

  //glPushMatrix();
  //  glColor3f(0.5f, 0.5f, 0.5f);
  //  glTranslatef(10.0f, 10.0f, 10.0f);
  //  gluSphere(mSphereQuad, 1.0f, 15, 15);
  //glPopMatrix();

  //glPushMatrix();
  //  glColor3f(1.0f, 1.0f, 1.0f);
  //  glTranslatef(10.0f, 0.0f, 10.0f);
  //  gluSphere(mSphereQuad, 1.0f, 15, 15);
  //glPopMatrix();

  glPushMatrix();
    glMultMatrixf(table1->getTrans().mData);
    table1->draw_model();
  glPopMatrix();

  //glPushMatrix();
  //  glMultMatrixf(table2->getTrans().mData);
  //  table2->draw_model();
  //glPopMatrix();

  glPushMatrix();
    glMultMatrixf(tableinst->getTrans().mData);
    tableinst->draw_model();
  glPopMatrix();

  //glPushMatrix();
  //  glMultMatrixf(platte->getPostTransf().mData);
  //  glMultMatrixf(platte->getTrans().mData);
  //  platte->draw_model();
  //glPopMatrix();

  //glPushMatrix();
  //  glMultMatrixf(scow->getPostTransf().mData);
  //  glMultMatrixf(scow->getTrans().mData);
  //  scow->draw_model();
  //glPopMatrix();

  for(int i=0; i<selectable.size(); ++i)
  {
    btRigidBody *body = selectable[i]->getBody();
    if(body)
    {
      glPushMatrix();
        body->getMotionState()->getWorldTransform(t);
        t.getOpenGLMatrix(mat);
        glMultMatrixf(mat);
        glScalef(selectable[i]->getScale()[0],selectable[i]->getScale()[1],selectable[i]->getScale()[2]);
        glMultMatrixf(selectable[i]->getInitTransf().mData);
        selectable[i]->draw_model();
      glPopMatrix();
      if(selectable[i]==moteur)
      {
        glPushMatrix();
          glMultMatrixf(mat);
          const btCompoundShape *cpSh = static_cast<const btCompoundShape*>(body->getCollisionShape());
          cpSh->getChildTransform(1).getOpenGLMatrix(mat);
          glMultMatrixf(mat);
          glScalef(meche->getScale()[0],meche->getScale()[1],meche->getScale()[2]);
          glMultMatrixf(meche->getInitTransf().mData);
          glRotatef(theta,0.f,1.f,0.f);
          meche->draw_model();
        glPopMatrix();
      }
    }
  }

  //scow->drawBody();
  //scow1->drawBody();
  //scow2->drawBody();
  //scow3->drawBody();
  //cube1->drawBody();
  //cube2->drawBody();
  //table1->drawBody();
  //platte->drawBody();
  //tableinst->drawBody();
  //moteur->drawBody();

  //glPushMatrix();
  //  glMultMatrixf(scow1->getPostTransf().mData);
  //  glMultMatrixf(scow1->getTrans().mData);
  //  scow1->draw_model();
  //glPopMatrix();

  //glPushMatrix();
  //  glMultMatrixf(scow2->getPostTransf().mData);
  //  glMultMatrixf(scow2->getTrans().mData);
  //  scow2->draw_model();
  //glPopMatrix();

  //glPushMatrix();
  //  glMultMatrixf(scow3->getPostTransf().mData);
  //  glMultMatrixf(scow3->getTrans().mData);
  //  scow3->draw_model();
  //glPopMatrix();

  glPushMatrix();
    glMultMatrixf(cube1->getTrans().mData);
    cube1->draw_model();
  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(cube2->getTrans().mData);
    cube2->draw_model();
  glPopMatrix();

  //glPushMatrix();
  //  glMultMatrixf(moteur->getPostTransf().mData);
  //  glPushMatrix();
  //    glMultMatrixf(moteur->getTrans().mData);
  //    moteur->draw_model();
  //  glPopMatrix();
  //  glPushMatrix();
  //    glMultMatrixf(meche->getTrans().mData);
  //    glRotatef(theta,0.f,1.f,0.f);
  //    meche->draw_model();
  //  glPopMatrix();
  //glPopMatrix();

  //glPushMatrix();
  //  femur8->getBody()->getMotionState()->getWorldTransform(t);
  //  t.getOpenGLMatrix(mat);
  //  glMultMatrixf(mat);
  //  //glMultMatrixf(femur8->getTrans().mData);
  //  glScalef(femur8->getScale()[0],femur8->getScale()[1],femur8->getScale()[2]);
  //  glMultMatrixf(femur8->getInitTransf().mData);
  //  femur8->draw_model();
  //glPopMatrix();

  glPushMatrix();
    femur8->getBody()->getMotionState()->getWorldTransform(t);
    t.getOpenGLMatrix(mat);
    glMultMatrixf(mat);
    glColor3f(22.0f,22.0f,22.0f);
    btConcaveShape *concaveMesh = (btConcaveShape*) femur8->getBody()->getCollisionShape();
    GlDrawcallback drawCallback;
    //drawCallback.m_wireframe = true;
    concaveMesh->processAllTriangles(&drawCallback,btVector3(-100,-100,-100),btVector3(100,100,100));
  glPopMatrix();
  //glPushMatrix();
  //  glMultMatrixf(verb->getPostTransf().mData);
  //  glMultMatrixf(verb->getTrans().mData);
  //  verb->draw_model(); 
  //glPopMatrix();

  for(int i=0; i<fallRigidBodies.size(); ++i)
  {
    glPushMatrix();
      glColor3f(1.0f,0.0f,0.0f);
      fallRigidBodies[i]->getMotionState()->getWorldTransform(t);
      t.getOpenGLMatrix(mat);
      glMultMatrixf(mat);
      gluSphere(mSphereQuad, 0.02, 10, 10);
    glPopMatrix();
  }

  //// cylindre pour la partie centrale du fémur
  //glPushMatrix();
  //  femur8->getBody()->getMotionState()->getWorldTransform(t);
  //  t.getOpenGLMatrix(mat);
  //  glMultMatrixf(mat);
  //  glColor3f(0.8f, 0.8f, 0.5f);
  //  btVector3 aabbMin, aabbMax;
  //  femur8->getBody()->getCollisionShape()->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);
  //  glRotatef(90.f, 1.f, 0.f, 0.f);
  //  glTranslatef(0.f,0.f,aabbMin[1]);
  //  gluCylinder(mSphereQuad, aabbMax[0], aabbMax[0], 2.f*aabbMax[1], 10, 10);
  //glPopMatrix();

  glPopMatrix();

  glPushMatrix();
    glMultMatrixf(mNavMatrix.mData);
    drawSphere(mSphere, mSphereIsect, mSphereSelected);
  glPopMatrix();


  //moteur->draw_waabox();
  //scow->draw_waabox();
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

void OsApp::buildTrimesh(const ConvexDecomposition::WavefrontObj &wo, btTriangleMesh *trimesh, const btVector3 &centre, const btVector3 &scaling)
{
  for(int i = 0; i < wo.mTriCount; ++i)
  {
    int index0 = wo.mIndices[i*3];
    int index1 = wo.mIndices[i*3+1];
    int index2 = wo.mIndices[i*3+2];

    btVector3 vertex0(wo.mVertices[index0*3], wo.mVertices[index0*3+1], wo.mVertices[index0*3+2]);
    btVector3 vertex1(wo.mVertices[index1*3], wo.mVertices[index1*3+1], wo.mVertices[index1*3+2]);
    btVector3 vertex2(wo.mVertices[index2*3], wo.mVertices[index2*3+1], wo.mVertices[index2*3+2]);

    vertex0 -= centre;
    vertex1 -= centre;
    vertex2 -= centre;

    vertex0 *= scaling;
    vertex1 *= scaling;
    vertex2 *= scaling;

    trimesh->addTriangle(vertex0,vertex1,vertex2);
  }
}

void OsApp::addFemur()
{
  femur8 = new MeshObj("femur8.obj");
  femur8->make_bbox();
  femur8->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-femur8->center[0],-femur8->center[1],-femur8->center[2])));
  femur8->setScale(0.0023f);
  femur8->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(4.0f), y_axis )));
  femur8->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.0f), z_axis )));
  femur8->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(-78.0f), y_axis )));
  //btMatrix3x3 btRot33 = gmtl44rot2bt33rot(femur8->getTrans());
  femur8->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(280.0f*0.0023,1230.0f*0.0023,2600.0f*0.0023)));
  //femur8->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(280.0f*0.0023,1230.0f*0.0023,0.0f*0.0023)));
#if 1
  ConvexDecomposition::WavefrontObj wo;
  wo.loadObj("femur8.obj");
  btTriangleMesh *trimesh = new btTriangleMesh;
  btVector3 localScaling(0.0023f,0.0023f,0.0023f);
  btVector3 centre(femur8->center[0],femur8->center[1],femur8->center[2]);
  buildTrimesh(wo,trimesh,centre,localScaling);
  //btBvhTriangleMeshShape* concaveShape = new btBvhTriangleMeshShape(trimesh,true);
  btGImpactMeshShape* concaveShape = new btGImpactMeshShape(trimesh);
  concaveShape->updateBound();
  btQuaternion rot = btQuaternion(gmtl::Math::deg2Rad(-78.0f),0,0) * btQuaternion(0,0,gmtl::Math::deg2Rad(90.0f)) * btQuaternion(gmtl::Math::deg2Rad(4.0f),0,0);
  btDefaultMotionState *state =
    new btDefaultMotionState(btTransform(rot, btVector3(femur8->trCentre()[0],femur8->trCentre()[1],femur8->trCentre()[2])));
  //btDefaultMotionState *state =
  //  new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0.f,0.f,0.f)));
  btRigidBody::btRigidBodyConstructionInfo rbCI(0.f, state, concaveShape, btVector3(0.f, 0.f, 0.f));
#else
  btCollisionShape *shape = new btCylinderShape(btVector3(0.06f,0.7f,1.f));
  btDefaultMotionState *state =
    new btDefaultMotionState(btTransform(btQuaternion(gmtl::Math::deg2Rad(5.f),0.f,gmtl::Math::deg2Rad(91.f)), btVector3(0.5f,2.6f,0.21f)));
  btRigidBody::btRigidBodyConstructionInfo rbCI(0.f, state, shape, btVector3(0.f, 0.f, 0.f));
#endif
  btRigidBody *rb = new btRigidBody(rbCI);
  rb->setFriction(5.f);
  addBody(rb);
  femur8->setBody(rb);
  //std::cout << "femur est concave ? " << femur8->getBody()->getCollisionShape()->isConcave() << std::endl;
}

void OsApp::addTable1()
{
  table1 = new MeshObj("table1.obj");
  table1->setScale(0.06f);
  table1->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(230.f), y_axis)));
  table1->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-5.0f*0.06, 0.0f, 0.0f)));
  //table1->make_bbox();
  //float dx, dy, dz;
  //dx = table1->aabox.mMax[0] - table1->aabox.mMin[0];
  //dy = table1->aabox.mMax[1] - table1->aabox.mMin[1];
  //dz = table1->aabox.mMax[2] - table1->aabox.mMin[2];
  //btCollisionShape *tableShape = new btBoxShape(btVector3(dx*0.06/2, dy*0.06/2, dz*0.06/2));
  //btDefaultMotionState* tableMotionState =
  //              new btDefaultMotionState(btTransform(btQuaternion(gmtl::Math::deg2Rad(230.f),0,0), btVector3(table1->trCentre()[0],table1->trCentre()[1],table1->trCentre()[2])));
  btCollisionShape *tableShape = new btBoxShape(btVector3(109.f*0.06/2, 10.f*0.06/2, 30.f*0.06/2));
  btDefaultMotionState* tableMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(gmtl::Math::deg2Rad(185.f),0,0), btVector3(-0.5f,33.8f*0.06,0.f)));
  btRigidBody::btRigidBodyConstructionInfo tableRigidBodyCI(0, tableMotionState, tableShape, btVector3(0,0,0));
  btRigidBody *tableRigidBody = new btRigidBody(tableRigidBodyCI);
  addBody(tableRigidBody);
  table1->setBody(tableRigidBody);
}

void OsApp::addTable2()
{
  table2 = new MeshObj("table2.obj");
  table2->addTransf(gmtl::makeScale<gmtl::Matrix44f,float>(0.05f));
  table2->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(230.f), y_axis)));
  table2->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-2.5f*0.05, 20.0f*0.05, 3.0f*0.05)));
}

void OsApp::addTableInst()
{
  tableinst = new MeshObj("tableinst.obj");
  tableinst->setScale(gmtl::Vec3f(0.02f, 0.03f, 0.02f));
  tableinst->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(82.f), y_axis)));
  tableinst->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(300.0f*0.02, -10.0f*0.03, 744.0f*0.02)));
  tableinst->make_bbox();
  //std::cout << "tableinst " << tableinst->aabox.mMin[0] << ' ' << tableinst->aabox.mMin[1] << ' ' << tableinst->aabox.mMin[2] << ' ' <<
  //  tableinst->aabox.mMax[0] << ' ' << tableinst->aabox.mMax[1] << ' ' << tableinst->aabox.mMax[2] << ' ' << std::endl;
  //std::cout << "tableinst " << tableinst->getWaabox().mMin[0] << ' ' << tableinst->getWaabox().mMin[1] << ' ' << tableinst->getWaabox().mMin[2] << ' ' <<
  //  tableinst->getWaabox().mMax[0] << ' ' << tableinst->getWaabox().mMax[1] << ' ' << tableinst->getWaabox().mMax[2] << ' ' << std::endl;
  //float dx, dy, dz;
  //dx = tableinst->aabox.mMax[0] - tableinst->aabox.mMin[0];
  //dy = tableinst->aabox.mMax[1] - tableinst->aabox.mMin[1];
  //dz = tableinst->aabox.mMax[2] - tableinst->aabox.mMin[2];
  //btCollisionShape *tableInstShape = new btBoxShape(btVector3(dx*0.02/2, dy*0.03/2, dz*0.02/2));
  btCollisionShape *tableInstShape = new btBoxShape(btVector3(1.f, 1.5f, 0.7f));
  //MeshObj dummy("tableinst.obj");
  //dummy.addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(82.f), y_axis)));
  //dummy.addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(300.0f*0.02, -10.0f*0.03, 744.0f*0.02)));
  //gmtl::Matrix44f tableInstT = tableinst->getTrans();
  //btMatrix3x3 tableInstRot;
  //btVector3 tableInstTransl;
  //std::cout << "centre ";
  //gmtl::Point3f trCentre = tableInstT * gmtl::Point3f(tableinst->center[0],tableinst->center[1],tableinst->center[2]);
  //for(int i=0; i<3; ++i)
  //{
  //  for(int j=0; j<3; ++j)
  //  {
  //    tableInstRot[i][j] = tableInstT[i][j];
  //  }
  //  tableInstTransl[i] = trCentre[i]; //tableInstT[i][3] + tableinst->center[i]*(i==1 ? 0.03 : 0.02);
  //  std::cout << tableInstTransl[i] << ' ';
  //}
  //std::cout << std::endl;
  //std::cout << "centre transformé ";
  //for(int i=0; i<3; ++i) std::cout << trCentre[i] << ' ';
  //std::cout << std::endl;
  //btDefaultMotionState* tableInstMotionState =
  //              new btDefaultMotionState(btTransform(tableInstRot, tableInstTransl));
  btDefaultMotionState* tableInstMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(gmtl::Math::deg2Rad(95.f),0,0), btVector3(tableinst->trCentre()[0],tableinst->trCentre()[1],tableinst->trCentre()[2])));
  //btDefaultMotionState* tableInstMotionState =
  //              new btDefaultMotionState(btTransform(btQuaternion(gmtl::Math::deg2Rad(82.f),0,0), btVector3(tableinst->trCentre()[0],tableinst->trCentre()[1],tableinst->trCentre()[2])));
  btRigidBody::btRigidBodyConstructionInfo tableInstRigidBodyCI(0, tableInstMotionState, tableInstShape, btVector3(0,0,0));
  btRigidBody *tableInstRigidBody = new btRigidBody(tableInstRigidBodyCI);
  addBody(tableInstRigidBody);
  tableinst->setBody(tableInstRigidBody);
  //float mat[16];
  //btTransform t;
  //tableInstRigidBody->getMotionState()->getWorldTransform(t);
  //t.getOpenGLMatrix(mat);
  //for(int i=0; i<16; ++i) std::cout << mat[i] << ' ';
  //std::cout << std::endl;
  //for(int i=0; i<16; ++i) std::cout << tableinst->getTrans().mData[i] << ' ';
  //std::cout << std::endl;
}

void OsApp::addPerceuse()
{
  moteur = new MeshObj("moteur.obj");
  moteur->make_bbox();
  selectable.push_back(moteur);
  moteur->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-moteur->center[0],-moteur->center[1],-moteur->center[2])));
  //moteur->print();
  //std::cout << moteur->center[0] << ' ' << moteur->center[1] << ' ' << moteur->center[2] << std::endl;
  moteur->setScale(0.0017f);
  //moteur->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(-10.0f), y_axis )));
  //moteur->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(150.0f), z_axis )));
  //moteur->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(80.0f), x_axis )));
  //moteur->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(2000.9f*0.0017,1750.0f*0.0017,250.0f*0.0017)));
  float dx, dy, dz;
  dx = moteur->aabox.mMax[0] - moteur->aabox.mMin[0];
  dy = moteur->aabox.mMax[1] - moteur->aabox.mMin[1];
  dz = moteur->aabox.mMax[2] - moteur->aabox.mMin[2];
  btCollisionShape *moteurShape = new btBoxShape(btVector3(dx*0.0017/2, dy*0.0017/2, dz*0.0017/2));

  meche = new MeshObj("meche.obj");
  meche->make_bbox();
  meche->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-meche->center[0],-meche->center[1],-meche->center[2])));
  //std::cout << meche->center[0] << ' ' << meche->center[1] << ' ' << meche->center[2] << std::endl;
  meche->setScale(0.025f);
  //meche->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(-180.0f), x_axis )));
  //meche->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(-35.0f), z_axis )));
  //meche->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(80.0f), x_axis )));
  //meche->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(131.8f*0.025, 126.f*0.025, -4.3f*0.025)));
  dx = meche->aabox.mMax[0] - meche->aabox.mMin[0];
  dy = meche->aabox.mMax[1] - meche->aabox.mMin[1];
  dz = meche->aabox.mMax[2] - meche->aabox.mMin[2];
  btCollisionShape *mecheShape = new btBoxShape(btVector3(dx*0.025/2, dy*0.025/2, dz*0.025/2));

  btCompoundShape *compound = new btCompoundShape();
  btTransform mecheTr;
  mecheTr.setIdentity();
  mecheTr.setOrigin(btVector3(0.f,0.3,-0.22f));
  //mecheTr.setRotation(btQuaternion(0.f,gmtl::Math::deg2Rad(90.f),0.f));
  btTransform moteurTr;
  moteurTr.setIdentity();
  compound->addChildShape(moteurTr, moteurShape);
  compound->addChildShape(mecheTr, mecheShape);

  btTransform tr(btQuaternion(0.f,gmtl::Math::deg2Rad(90.f),0.f), btVector3(3.4f,5.075f,0.425f));
  moteur->setInitBodyTr(tr);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(tr);
  btScalar mass = 30.f;
  btVector3 fallInertia(0, 0, 0);
  compound->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, compound, fallInertia);
  btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  addBody(fallRigidBody);
  moteur->setBody(fallRigidBody);

  //const btCompoundShape *cpSh = static_cast<const btCompoundShape*>(fallRigidBody->getCollisionShape());
  //for(int i=0; i<cpSh->getNumChildShapes(); ++i)
  //{
  //  btTransform tr = cpSh->getChildTransform(i);
  //  std::cout << "ch " << tr.getOrigin()[0] << ' ' << tr.getOrigin()[1] << ' ' << tr.getOrigin()[2] << std::endl;
  //}
}

void OsApp::addPlaque()
{
  platte = new MeshObj("platte.obj");
  platte->make_bbox();
  selectable.push_back(platte);
  platte->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-platte->center[0],-platte->center[1],-platte->center[2])));
  platte->setScale(0.0001f);
  platte->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(79.0f), x_axis )));
  platte->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(5.0f), y_axis )));
  platte->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(30790.0f*0.0001,76999.0f*0.0001,-5500.0f*0.0001)));
  //platte->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(0.5f,2.7f,0.1f)));
#if 1
  ConvexDecomposition::WavefrontObj wo;
  wo.loadObj("platte.obj");
  btTriangleMesh *trimesh = new btTriangleMesh;
  btVector3 localScaling(0.0001f,0.0001f,0.0001f);
  btVector3 centre(platte->center[0],platte->center[1],platte->center[2]);
  buildTrimesh(wo,trimesh,centre,localScaling);
  btGImpactMeshShape *fallShape = new btGImpactMeshShape(trimesh);
  fallShape->updateBound();
#else
  float dx, dy, dz;
  dx = platte->aabox.mMax[0] - platte->aabox.mMin[0];
  dy = platte->aabox.mMax[1] - platte->aabox.mMin[1];
  dz = platte->aabox.mMax[2] - platte->aabox.mMin[2];
  btCollisionShape *fallShape = new btBoxShape(btVector3(dx*0.0001/2, dy*0.0001/2, dz*0.0001/2));
#endif
  btTransform tr(btQuaternion(0.f,gmtl::Math::deg2Rad(185.f),0.f), btVector3(platte->trCentre()[0],platte->trCentre()[1],platte->trCentre()[2]));
  platte->setInitBodyTr(tr);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(tr);
  btScalar mass = 0.3f;
  btVector3 fallInertia(0, 0, 0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
  btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  fallRigidBody->setCollisionFlags(fallRigidBody->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
  addBody(fallRigidBody);
  platte->setBody(fallRigidBody);
}

void OsApp::addVis()
{
  scow = new MeshObj("scow.obj");
  scow->make_bbox();
  selectable.push_back(scow);
  //std::cout << "scow ";
  //scow->print();
  scow->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-scow->center[0],-scow->center[1],-scow->center[2])));
  scow->setScale(0.00009f);
  scow->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.f), z_axis)));
  scow->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(33000.0f*0.00009,32209.0f*0.00009,0.0f*0.00009)));
  float dx, dy, dz;
  dx = scow->aabox.mMax[0] - scow->aabox.mMin[0];
  dy = scow->aabox.mMax[1] - scow->aabox.mMin[1];
  dz = scow->aabox.mMax[2] - scow->aabox.mMin[2];
  btCollisionShape *fallShape = new btBoxShape(btVector3(dx*0.00009/2, dy*0.00009/2, dz*0.00009/2));
  //gmtl::Matrix44f scowt = scow->getTrans();
  //btMatrix3x3 rot;
  //btVector3 transl;
  //for(int i=0; i<3; ++i)
  //{
  //  for(int j=0; j<3; ++j)
  //  {
  //    rot[i][j] = scowt[i][j];
  //  }
  //  transl[i] = scowt[i][3];
  //}
  //btDefaultMotionState* fallMotionState =
  //              new btDefaultMotionState(btTransform(rot, transl));
  btTransform tr(btQuaternion(0,0,gmtl::Math::deg2Rad(-90.f)), btVector3(scow->trCentre()[0],scow->trCentre()[1],scow->trCentre()[2]));
  scow->setInitBodyTr(tr);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(tr);
  btScalar mass = 0.1f;
  btVector3 fallInertia(0, 0, 0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
  btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  addBody(fallRigidBody);
  scow->setBody(fallRigidBody);
}

void OsApp::addVis1()
{
  scow1 = new MeshObj("scow.obj");
  scow1->make_bbox();
  selectable.push_back(scow1);
  scow1->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-scow1->center[0],-scow1->center[1],-scow1->center[2])));
  scow1->setScale(0.00009f);
  scow1->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.f), z_axis)));
  scow1->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(33000.0f*0.00009,32209.0f*0.00009,1000.0f*0.00009)));
  float dx, dy, dz;
  dx = scow1->aabox.mMax[0] - scow1->aabox.mMin[0];
  dy = scow1->aabox.mMax[1] - scow1->aabox.mMin[1];
  dz = scow1->aabox.mMax[2] - scow1->aabox.mMin[2];
  btCollisionShape *fallShape = new btBoxShape(btVector3(dx*0.00009/2, dy*0.00009/2, dz*0.00009/2));
  btTransform tr(btQuaternion(0,0,gmtl::Math::deg2Rad(90.f)), btVector3(scow1->trCentre()[0],scow1->trCentre()[1],scow1->trCentre()[2]));
  scow1->setInitBodyTr(tr);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(tr);
  btScalar mass = 0.1f;
  btVector3 fallInertia(0, 0, 0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
  btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  addBody(fallRigidBody);
  scow1->setBody(fallRigidBody);
}

void OsApp::addVis2()
{
  scow2 = new MeshObj("scow.obj");
  scow2->make_bbox();
  selectable.push_back(scow2);
  scow2->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-scow2->center[0],-scow2->center[1],-scow2->center[2])));
  scow2->setScale(0.00009f);
  scow2->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.f), z_axis)));
  scow2->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(33000.0f*0.00009,32209.0f*0.00009,2000.0f*0.00009)));
  float dx, dy, dz;
  dx = scow2->aabox.mMax[0] - scow2->aabox.mMin[0];
  dy = scow2->aabox.mMax[1] - scow2->aabox.mMin[1];
  dz = scow2->aabox.mMax[2] - scow2->aabox.mMin[2];
  btCollisionShape *fallShape = new btBoxShape(btVector3(dx*0.00009/2, dy*0.00009/2, dz*0.00009/2));
  btTransform tr(btQuaternion(0,0,gmtl::Math::deg2Rad(90.f)), btVector3(scow2->trCentre()[0],scow2->trCentre()[1],scow2->trCentre()[2]));
  scow2->setInitBodyTr(tr);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(tr);
  btScalar mass = 0.1f;
  btVector3 fallInertia(0, 0, 0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
  btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  addBody(fallRigidBody);
  scow2->setBody(fallRigidBody);
}

void OsApp::addVis3()
{
  scow3 = new MeshObj("scow.obj");
  scow3->make_bbox();
  selectable.push_back(scow3);
  scow3->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-scow3->center[0],-scow3->center[1],-scow3->center[2])));
  scow3->setScale(0.00009f);
  scow3->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(90.f), z_axis)));
  scow3->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(33000.0f*0.00009,32209.0f*0.00009,-900.0f*0.00009)));
  float dx, dy, dz;
  dx = scow3->aabox.mMax[0] - scow3->aabox.mMin[0];
  dy = scow3->aabox.mMax[1] - scow3->aabox.mMin[1];
  dz = scow3->aabox.mMax[2] - scow3->aabox.mMin[2];
  btCollisionShape *fallShape = new btBoxShape(btVector3(dx*0.00009/2, dy*0.00009/2, dz*0.00009/2));
  btTransform tr(btQuaternion(0,0,gmtl::Math::deg2Rad(90.f)), btVector3(scow3->trCentre()[0],scow3->trCentre()[1],scow3->trCentre()[2]));
  scow3->setInitBodyTr(tr);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(tr);
  btScalar mass = 0.1f;
  btVector3 fallInertia(0, 0, 0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
  btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  addBody(fallRigidBody);
  scow3->setBody(fallRigidBody);
}

void OsApp::addCube1()
{
  cube1 = new MeshObj("cube1.obj");
  cube1->setScale(0.003f);
  cube1->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(7.f), y_axis)));
  cube1->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(510.0f*0.003,750.7f*0.003,580.0f*0.003)));
  cube1->make_bbox();
  float dx, dy, dz;
  dx = cube1->aabox.mMax[0] - cube1->aabox.mMin[0];
  dy = cube1->aabox.mMax[1] - cube1->aabox.mMin[1];
  dz = cube1->aabox.mMax[2] - cube1->aabox.mMin[2];
  btCollisionShape *cubeShape = new btBoxShape(btVector3(dx*0.003/2, dy*0.003/2, dz*0.003/2));
  btDefaultMotionState* cubeMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(gmtl::Math::deg2Rad(7.f),0,0), btVector3(cube1->trCentre()[0],cube1->trCentre()[1],cube1->trCentre()[2])));
  btRigidBody::btRigidBodyConstructionInfo cubeRigidBodyCI(0, cubeMotionState, cubeShape, btVector3(0,0,0));
  btRigidBody *cubeRigidBody = new btRigidBody(cubeRigidBodyCI);
  addBody(cubeRigidBody);
  cube1->setBody(cubeRigidBody);
}

void OsApp::addCube2()
{
  cube2 = new MeshObj("cube1.obj");
  cube2->setScale(0.003f);
  cube2->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(7.f), y_axis)));
  cube2->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(770.0f*0.003,750.7f*0.003,560.0f*0.003)));
  cube2->make_bbox();
  float dx, dy, dz;
  dx = cube2->aabox.mMax[0] - cube2->aabox.mMin[0];
  dy = cube2->aabox.mMax[1] - cube2->aabox.mMin[1];
  dz = cube2->aabox.mMax[2] - cube2->aabox.mMin[2];
  btCollisionShape *cubeShape = new btBoxShape(btVector3(dx*0.003/2, dy*0.003/2, dz*0.003/2));
  btDefaultMotionState* cubeMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(gmtl::Math::deg2Rad(7.f),0,0), btVector3(cube2->trCentre()[0],cube2->trCentre()[1],cube2->trCentre()[2])));
  btRigidBody::btRigidBodyConstructionInfo cubeRigidBodyCI(0, cubeMotionState, cubeShape, btVector3(0,0,0));
  btRigidBody *cubeRigidBody = new btRigidBody(cubeRigidBodyCI);
  addBody(cubeRigidBody);
  cube2->setBody(cubeRigidBody);
}

void OsApp::addVerb()
{
  verb = new MeshObj("verb.obj");
  verb->make_bbox();
  selectable.push_back(verb);
  verb->initTr(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(-verb->center[0],-verb->center[1],-verb->center[2])));
  verb->setScale(0.000057f);
  verb->addTransf(gmtl::makeRot<gmtl::Matrix44f>(gmtl::AxisAnglef(gmtl::Math::deg2Rad(55.0f), x_axis )));
  verb->addTransf(gmtl::makeTrans<gmtl::Matrix44f, gmtl::Vec3f>(gmtl::Vec3f(60000.0f*0.000057,48990.0f*0.000057,-16000.0f*0.000057)));
  float dx, dy, dz;
  dx = verb->aabox.mMax[0] - verb->aabox.mMin[0];
  dy = verb->aabox.mMax[1] - verb->aabox.mMin[1];
  dz = verb->aabox.mMax[2] - verb->aabox.mMin[2];
  btCollisionShape *fallShape = new btBoxShape(btVector3(dx*0.000057/2, dy*0.000057/2, dz*0.000057/2));
  btTransform tr(btQuaternion(0,0,gmtl::Math::deg2Rad(55.f)), btVector3(verb->trCentre()[0],verb->trCentre()[1],verb->trCentre()[2]));
  verb->setInitBodyTr(tr);
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(tr);
  btScalar mass = 0.3f;
  btVector3 fallInertia(0, 0, 0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
  btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  addBody(fallRigidBody);
  verb->setBody(fallRigidBody);
}

void OsApp::initPhysics()
{
  //Bullet Physics
  broadphase = new btDbvtBroadphase();
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  solver = new btSequentialImpulseConstraintSolver();
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, -9.8, 0));
  //dynamicsWorld->setGravity(btVector3(0, 0, 0));

  gContactAddedCallback = callbackFunc;

  m_pickConstraint = 0;
}

void OsApp::addGround()
{
  btCollisionShape *groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
  btDefaultMotionState* groundMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
  btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  addBody(groundRigidBody);
}