/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btSphereSegmentTriangleCollisionAlgorithm.h"
#include "BulletCollision/CollisionShapes/btSphereSegmentShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "SphereTriangleDetector.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"

btSphereSegmentTriangleCollisionAlgorithm::btSphereSegmentTriangleCollisionAlgorithm(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,bool swapped,btConvexPenetrationDepthSolver* pdSolver)
: btActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
m_ownManifold(false),
m_manifoldPtr(mf),
m_swapped(swapped),
m_pdSolver(pdSolver)
{
	if (!m_manifoldPtr)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}

btSphereSegmentTriangleCollisionAlgorithm::~btSphereSegmentTriangleCollisionAlgorithm()
{
	if (m_ownManifold && m_manifoldPtr)
        m_dispatcher->releaseManifold(m_manifoldPtr);
}

void btSphereSegmentTriangleCollisionAlgorithm::processCollision (const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}

    resultOut->setPersistentManifold(m_manifoldPtr);

    // Calculate closest points between sphere and triangle first
	const btCollisionObjectWrapper* sphereSegmentObjWrap = m_swapped? body1Wrap : body0Wrap;
	const btCollisionObjectWrapper* triObjWrap = m_swapped? body0Wrap : body1Wrap;

	btSphereSegmentShape* sphereSegment = (btSphereSegmentShape*)sphereSegmentObjWrap->getCollisionShape();
	btTriangleShape* triangle = (btTriangleShape*)triObjWrap->getCollisionShape();
	
	SphereTriangleDetector detector(sphereSegment->getRadius(),triangle, m_manifoldPtr->getContactBreakingThreshold()+ resultOut->m_closestPointDistanceThreshold);
	
	btDiscreteCollisionDetectorInterface::ClosestPointInput input;
	input.m_maximumDistanceSquared = btScalar(BT_LARGE_FLOAT);///@todo: tighter bounds
	input.m_transformA = sphereSegmentObjWrap->getWorldTransform();
	input.m_transformB = triObjWrap->getWorldTransform();

	bool swapResults = m_swapped;
	detector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw,swapResults);

    if (m_manifoldPtr->getNumContacts() > 0) 
    {
        int index = resultOut->getLastInsertIndex();
        btManifoldPoint& pt = m_manifoldPtr->getContactPoint(index);
        btVector3& localPointOnSphere = m_swapped? pt.m_localPointB : pt.m_localPointA;

        // If we've got a contact and it's outside the sphere-segment width, use GJK to calculate the closest point which should lie on the circular border
        if (btFabs(localPointOnSphere.x()) > sphereSegment->getWidth()*0.5) 
        {
            // Remove the point on sphere if it is reasonably away of the sphere-segment circular edge. 
            // Otherwise keep it and it should be merged with the GJK contact point and thus the appliedImpulse should be perpetuated.
            if (btFabs(localPointOnSphere.x()) - sphereSegment->getWidth()*0.5 > m_manifoldPtr->getContactCachingThreshold()*0.6) 
            {
                m_manifoldPtr->removeContactPoint(index);
            }

            const btConvexShape* min0 = static_cast<const btConvexShape*>(body0Wrap->getCollisionShape());
            const btConvexShape* min1 = static_cast<const btConvexShape*>(body1Wrap->getCollisionShape());

            btGjkPairDetector::ClosestPointInput input;
            btVoronoiSimplexSolver simplexSolver;
            btGjkPairDetector gjkPairDetector( min0, min1, &simplexSolver, m_pdSolver );
            gjkPairDetector.setMinkowskiA(min0);
            gjkPairDetector.setMinkowskiB(min1);

            input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactBreakingThreshold()+resultOut->m_closestPointDistanceThreshold;
            input.m_maximumDistanceSquared*= input.m_maximumDistanceSquared;

            input.m_transformA = body0Wrap->getWorldTransform();
            input.m_transformB = body1Wrap->getWorldTransform();

            gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);
        }
    }

	if (m_ownManifold)
    {
		resultOut->refreshContactPoints();
    }
}

btScalar btSphereSegmentTriangleCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* col0,btCollisionObject* col1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return btScalar(1.);
}