#include "NonDeterminismExample.h"
#include "btBulletDynamicsCommon.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"

struct NonDeterminismExample : public CommonRigidBodyBase
{
    btRigidBody* m_carChassis[2];
    btCollisionShape* m_wheelShape;
    btHinge2Constraint* m_hingeFL[2];
    btHinge2Constraint* m_hingeFR[2];
    btHinge2Constraint* m_hingeRL[2];
    btHinge2Constraint* m_hingeRR[2];
    
    NonDeterminismExample(struct GUIHelperInterface* helper)
    :CommonRigidBodyBase(helper)
    {
    }
    virtual ~NonDeterminismExample(){}
    virtual void initPhysics();
    virtual bool keyboardCallback(int key, int state);
    virtual void stepSimulation(float deltaTime);
    void resetCamera()
    {
        float dist = 4;
        float pitch = -35;
        float yaw = 52;
        float targetPos[3]={0,0,0};
        m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
    }
};

void NonDeterminismExample::initPhysics()
{
    m_guiHelper->setUpAxis(1);
    
    createEmptyDynamicsWorld();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    
    if (m_dynamicsWorld->getDebugDrawer()) {
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);
    }
    
#if 1
    btBulletWorldImporter importer(m_dynamicsWorld);
    
    
    const char* prefix[]={"","./","./data/","../data/","../../data/","../../../data/","../../../../data/"};
    int numPrefixes = sizeof(prefix)/sizeof(const char*);
    char relativeFileName[1024];
    FILE* f=0;
    
    for (int i=0;!f && i<numPrefixes;i++)
    {
        sprintf(relativeFileName,"%s%s",prefix[i],"plane.bullet");
        f = fopen(relativeFileName,"rb");
        if (f)
        {
            break;
        }
    }
    if (f)
    {
        fclose(f);
    }
    
    
    importer.loadFile(relativeFileName);
    
    m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
    
#else
    btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.),btScalar(10.),btScalar(50.)));
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
    
    //m_collisionShapes.push_back(groundShape);
    
    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0,-10,0));
    btRigidBody* groundBody = createRigidBody(0, groundTransform, groundShape, btVector4(0,0,1,1));
    groundBody->setFriction(0.62);
#endif
    
    int group = 4;
    int mask = btBroadphaseProxy::AllFilter^group;
    
    btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
    m_collisionShapes.push_back(chassisShape);
    
    btCompoundShape* compound = new btCompoundShape();
    m_collisionShapes.push_back(compound);
    btTransform localTrans;
    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0,1,0));
    
    compound->addChildShape(localTrans,chassisShape);
    
    {
        btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f,0.1f,0.5f));
        btTransform suppLocalTrans;
        suppLocalTrans.setIdentity();
        //localTrans effectively shifts the center of mass with respect to the chassis
        suppLocalTrans.setOrigin(btVector3(0,1.0,0));
        compound->addChildShape(suppLocalTrans, suppShape);
    }
    
    btTransform chassisTransform;
    chassisTransform.setIdentity();
    chassisTransform.setOrigin(btVector3(0,5,0));
    
    btScalar chassisMass = 300;
    
    m_wheelShape = new btCylinderShapeX(btVector3(0.4,0.5,0.5));
    
    btVector3 wheelPos[4] = {
        btVector3(btScalar(-1.), btScalar(-0.25+5), btScalar(1.25)),
        btVector3(btScalar(1.), btScalar(-0.25+5), btScalar(1.25)),
        btVector3(btScalar(1.), btScalar(-0.25+5), btScalar(-1.25)),
        btVector3(btScalar(-1.), btScalar(-0.25+5), btScalar(-1.25))
    };
    
    for (int j=0;j<2;j++) {
        m_carChassis[j] = createRigidBody(chassisMass,chassisTransform,compound,group,mask);

        for (int i=0;i<4;i++)
        {
            btRigidBody* pBodyA = this->m_carChassis[j];
            // dynamic bodyB (child) below it :
            btTransform tr;
            tr.setIdentity();
            tr.setOrigin(wheelPos[i]);
            
            btRigidBody* pBodyB = createRigidBody(30.0, tr, m_wheelShape, group, mask);
            pBodyB->setFriction(0.8);
            pBodyB->setRollingFriction(0.1);
            // add some data to build constraint frames
            btVector3 parentAxis(0.f, 1.f, 0.f);
            btVector3 childAxis(1.f, 0.f, 0.f);
            btVector3 anchor = tr.getOrigin();
            btHinge2Constraint* pHinge2 = new btHinge2Constraint(*pBodyA, *pBodyB, anchor, parentAxis, childAxis);
            
            pHinge2->setLowerLimit(0);
            pHinge2->setUpperLimit(0);
            
            pHinge2->setLinearLowerLimit(btVector3(0.f, 0.f, -0.1f));
            pHinge2->setLinearUpperLimit(btVector3(0.f, 0.f,  0.9f));
            
            // add constraint to world
            m_dynamicsWorld->addConstraint(pHinge2, true);
            // draw constraint frames and limits for debugging
            
            {
                int motorAxis = 5;
                pHinge2->enableMotor(motorAxis,true);
                pHinge2->setMaxMotorForce(motorAxis,1000);
                pHinge2->setTargetVelocity(motorAxis,0);
            }
            
            pHinge2->setAngularLowerLimit(btVector3(1,0,0));
            pHinge2->setAngularUpperLimit(btVector3(-1,0,0));
            
            pHinge2->setDbgDrawSize(btScalar(5.f));
            
            if (i == 0) m_hingeFR[j] = pHinge2;
            if (i == 1) m_hingeFL[j] = pHinge2;
            if (i == 2) m_hingeRR[j] = pHinge2;
            if (i == 3) m_hingeRL[j] = pHinge2;
        }
    }
    
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    
}

void NonDeterminismExample::stepSimulation(float deltaTime)
{
    CommonRigidBodyBase::stepSimulation(deltaTime);
}

bool NonDeterminismExample::keyboardCallback(int key, int state)
{
    bool handled = false;
    
    for (int j=0;j<2;j++) {
        switch (key) {
            case B3G_LEFT_ARROW : {
                handled = true;
                break;
            }
            case B3G_RIGHT_ARROW : {
                handled = true;
                break;
            }
            case B3G_UP_ARROW : {
                m_carChassis[j]->activate(true);
                m_hingeRL[j]->enableMotor(3, state);
                m_hingeRR[j]->enableMotor(3, state);
                m_hingeRL[j]->setMaxMotorForce(3,3000);
                m_hingeRR[j]->setMaxMotorForce(3,3000);
                m_hingeRL[j]->setTargetVelocity(3, state ? 50 : 0);
                m_hingeRR[j]->setTargetVelocity(3, state ? 50 : 0);
                handled = true;
                break;
            }
            case B3G_DOWN_ARROW : {
                m_carChassis[j]->activate(true);
                m_hingeRL[j]->enableMotor(3, state);
                m_hingeRR[j]->enableMotor(3, state);
                m_hingeRL[j]->setMaxMotorForce(3,500);
                m_hingeRR[j]->setMaxMotorForce(3,500);
                m_hingeRL[j]->setTargetVelocity(3, state ? -100 : 0);
                m_hingeRR[j]->setTargetVelocity(3, state ? -100 : 0);
                handled = true;
                break;
            }
        }
    }
    
    return handled;
}

CommonExampleInterface* NonDeterminismExampleCreateFunc(CommonExampleOptions& options)
{
    return new NonDeterminismExample(options.m_guiHelper);
    
}
