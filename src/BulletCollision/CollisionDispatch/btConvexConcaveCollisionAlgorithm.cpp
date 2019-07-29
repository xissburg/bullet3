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

#include "btConvexConcaveCollisionAlgorithm.h"
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/CollisionShapes/btSdfCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"

btConvexConcaveCollisionAlgorithm::btConvexConcaveCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, bool isSwapped)
	: btActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
	  m_btConvexTriangleCallback(ci.m_dispatcher1, body0Wrap, body1Wrap, isSwapped),
	  m_isSwapped(isSwapped)
{
}

btConvexConcaveCollisionAlgorithm::~btConvexConcaveCollisionAlgorithm()
{
}

void btConvexConcaveCollisionAlgorithm::getAllContactManifolds(btManifoldArray& manifoldArray)
{
	if (m_btConvexTriangleCallback.m_manifoldPtr)
	{
		manifoldArray.push_back(m_btConvexTriangleCallback.m_manifoldPtr);
	}
}

btConvexTriangleCallback::btConvexTriangleCallback(btDispatcher* dispatcher, const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, bool isSwapped) : m_dispatcher(dispatcher),
																																													 m_dispatchInfoPtr(0)
{
	m_convexBodyWrap = isSwapped ? body1Wrap : body0Wrap;
	m_triBodyWrap = isSwapped ? body0Wrap : body1Wrap;

	//
	// create the manifold from the dispatcher 'manifold pool'
	//
	m_manifoldPtr = m_dispatcher->getNewManifold(m_convexBodyWrap->getCollisionObject(), m_triBodyWrap->getCollisionObject());

	clearCache();
}

btConvexTriangleCallback::~btConvexTriangleCallback()
{
	clearCache();
	m_dispatcher->releaseManifold(m_manifoldPtr);
}

void btConvexTriangleCallback::clearCache()
{
	m_dispatcher->clearManifold(m_manifoldPtr);
}

void btConvexTriangleCallback::processTriangle(btVector3* triangle, int partId, int triangleIndex)
{
	BT_PROFILE("btConvexTriangleCallback::processTriangle");

	if (!TestTriangleAgainstAabb2(triangle, m_aabbMin, m_aabbMax))
	{
		return;
	}

	//just for debugging purposes
	//printf("triangle %d",m_triangleCount++);

	btCollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher1 = m_dispatcher;

#if 0	
	
	///debug drawing of the overlapping triangles
	if (m_dispatchInfoPtr && m_dispatchInfoPtr->m_debugDraw && (m_dispatchInfoPtr->m_debugDraw->getDebugMode() &btIDebugDraw::DBG_DrawWireframe ))
	{
		const btCollisionObject* ob = const_cast<btCollisionObject*>(m_triBodyWrap->getCollisionObject());
		btVector3 color(1,1,0);
		btTransform& tr = ob->getWorldTransform();
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(triangle[1]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(triangle[2]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(triangle[0]),color);
	}
#endif
	
	if (!m_convexBodyWrap->getCollisionShape()->isConvex())
	{
		return;
	}

	btVector3 localTranslation = -triangle[0];
	btVector3 translation = m_triBodyWrap->getWorldTransform().getBasis() * localTranslation;

	btTriangleShape tm(btVector3(0,0,0), triangle[1]+localTranslation, triangle[2]+localTranslation);	
	tm.setMargin(m_collisionMarginTriangle);

	btTransform convexWrapTransform = m_convexBodyWrap->getWorldTransform();
	convexWrapTransform.setOrigin(convexWrapTransform.getOrigin() + translation);

	const btTransform convexObjTransform = m_convexBodyWrap->getCollisionObject()->getWorldTransform();

	btCollisionObject* convexObj = const_cast<btCollisionObject*>(m_convexBodyWrap->getCollisionObject());
	btTransform t = convexObj->getWorldTransform();
	t.setOrigin(t.getOrigin() + translation);
	convexObj->setWorldTransform(t);

	bool isTireFr = convexObj->getUserIndex() == 99135 && m_convexBodyWrap->m_index == 0;

	if (isTireFr) {
	//	printf("@--- Triangle %d -------------\n", triangleIndex);
	}

	btCollisionObjectWrapper convexBodyWrap(m_convexBodyWrap->m_parent, m_convexBodyWrap->m_shape, m_convexBodyWrap->m_collisionObject, convexWrapTransform, m_convexBodyWrap->m_partId, m_convexBodyWrap->m_index);
	
	// Add points to a temporary manifold in the convex-triangle collision algorithm
	// and then only add points that lie on the triangle face or in the voronoi region
	// of the edges to the original manifold.
	btPersistentManifold* originalManifold = m_resultOut->getPersistentManifold();
	btPersistentManifold* tempManifold = m_dispatcher->getNewManifold(originalManifold->getBody0(),originalManifold->getBody1());
	m_resultOut->setPersistentManifold(tempManifold);
	
	btCollisionObjectWrapper triObWrap(m_triBodyWrap,&tm,m_triBodyWrap->getCollisionObject(),m_triBodyWrap->getWorldTransform(),partId,triangleIndex);//correct transform?
	btCollisionAlgorithm* colAlgo = m_dispatcher->findAlgorithm(&convexBodyWrap, &triObWrap, tempManifold, 
									m_resultOut->m_closestPointDistanceThreshold > 0 ? BT_CLOSEST_POINT_ALGORITHMS : BT_CONTACT_POINT_ALGORITHMS);
	
	const btCollisionObjectWrapper* tmpWrap = 0;
	const btCollisionObjectWrapper* tmpConvexWrap = 0;

	if (m_resultOut->getBody0Internal() == m_triBodyWrap->getCollisionObject())
	{
		tmpWrap = m_resultOut->getBody0Wrap();
		m_resultOut->setBody0Wrap(&triObWrap);
		m_resultOut->setShapeIdentifiersA(partId,triangleIndex);

		tmpConvexWrap = m_resultOut->getBody1Wrap();
		m_resultOut->setBody1Wrap(&convexBodyWrap);
	}
	else
	{
		tmpWrap = m_resultOut->getBody1Wrap();
		m_resultOut->setBody1Wrap(&triObWrap);
		m_resultOut->setShapeIdentifiersB(partId,triangleIndex);

		tmpConvexWrap = m_resultOut->getBody0Wrap();
		m_resultOut->setBody0Wrap(&convexBodyWrap);
	}

	colAlgo->processCollision(&convexBodyWrap,&triObWrap,*m_dispatchInfoPtr,m_resultOut);

	// reset back to original manifold and only add valid points to it
	m_resultOut->setPersistentManifold(originalManifold);
	convexObj->setWorldTransform(convexObjTransform);

	const btBvhTriangleMeshShape* trimeshShape = 0;
	const btCollisionShape* shape = m_triBodyWrap->getCollisionObject()->getCollisionShape();

	if (shape->getShapeType() == SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE) 
	{
		trimeshShape = static_cast<const btScaledBvhTriangleMeshShape*>(shape)->getChildShape();
	}
	else if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE) 
	{
		trimeshShape = static_cast<const btBvhTriangleMeshShape*>(shape);
	}
	
	int hash = btGetHash(partId, triangleIndex);
	const btTriangleInfo* info = trimeshShape->getTriangleInfoMap()->find(hash);

	// TODO: check if info is null and if it is just add all points, or maybe embed this into the triangle
	// mesh shape and calculate it on initialization.
	btAssert(info);
	
	btScalar edgeAngles[] = {info->m_edgeV0V1Angle, info->m_edgeV1V2Angle, info->m_edgeV2V0Angle};
	bool swapped = tempManifold->getBody0() != m_resultOut->getBody0Wrap()->getCollisionObject();

	btVector3 localNormal;
	tm.calcNormal(localNormal);
	if (!swapped) localNormal *= -1;
	const btMatrix3x3& triBasis = triObWrap.getWorldTransform().getBasis();
	btVector3 worldNormal = triBasis * localNormal;

	const btScalar distToVertex = 0.01;

	if (isTireFr) {
		if (tempManifold->getNumContacts() == 0) {
			//printf("No points.. o_o\n");
		}
		else if (tempManifold->getNumContacts() != 1) {
			//printf("More than one point.. :O\n");
		}
	}

	// tempManifold->getNumContacts() should be 1 since both objects are convex
	for (int i = 0; i < tempManifold->getNumContacts(); ++i)
	{
		bool pointOnFace = true;
		btContactPointType cpType = BT_CP_TYPE_NONE;
		btManifoldPoint& pt = tempManifold->getContactPoint(i);
		const btVector3& pA = swapped ? pt.m_localPointB : pt.m_localPointA;
		const btVector3& contactNormal = swapped ? pt.m_normalWorldOnB : -pt.m_normalWorldOnB;

		// Check if point lies in the voronoi region of edges or vertices
		for (int j = 0; j < 3; ++j) 
		{
			btVector3 e0, e1;
			tm.getEdge(j, e0, e1);
			btVector3 edge = e1 - e0;
			btVector3 p = pA - e0;
			
			// Calc distance from pA to edge (see SegmentSqrDistance in SphereTriangleDetector.cpp)
			btScalar t = p.dot(edge) / edge.dot(edge);
			p -= t * edge;
			btScalar distance = btSqrt(p.dot(p));

			// Check if the point on the triangle is near the edge 
			if (distance < m_collisionMarginTriangle) {
				btScalar edgeAngle = edgeAngles[j];
						
				if (isTireFr) {
					//printf("Point on Edge %d with angle %f\n", j, edgeAngle/SIMD_PI*180);
				}

				if (edgeAngle == SIMD_2_PI) // edge has no adjacent face
				{
					pointOnFace = false;

					if ((pA - e0).length2() < distToVertex*distToVertex || (pA - e1).length2() < distToVertex*distToVertex) 
					{
						cpType = BT_CP_TYPE_VERTEX;

						if (isTireFr) {
							//printf("Result: Point on vertex with no adjacent face\n");
						}
					}
					else 
					{
						cpType = BT_CP_TYPE_EDGE;

						if (isTireFr) {
							//printf("Result: Point on edge with no adjacent face\n");
						}
					}
				}
				else if (edgeAngle <= 0) // convex edge
				{ 
					pointOnFace = false;
					bool pointOnVertex0 = (pA - e0).length2() < distToVertex*distToVertex;
					bool pointOnVertex1 = (pA - e1).length2() < distToVertex*distToVertex;

					if (pointOnVertex0 || pointOnVertex1) 
					{
						// This point must be within the limits of the voronoi region of both edges which share a vertex						
						// Project contact normal onto plane orthogonal to edge
						btVector3 e0n = triBasis * edge.normalized();
						btVector3 n0 = contactNormal - e0n * contactNormal.dot(e0n);
						btScalar c0 = n0.dot(worldNormal);
						btScalar a0 = btAcos(c0);

						int k = pointOnVertex0 ? (j-1)%3 : (j+1)%3;
						btVector3 e10, e11;
						tm.getEdge(k, e10, e11);
						btVector3 edge1 = e11 - e10;
						btVector3 e1n = triBasis * edge1.normalized();
						btVector3 n1 = contactNormal - e1n * contactNormal.dot(e1n);
						btScalar c1 = n1.dot(worldNormal);
						btScalar a1 = btAcos(c1);

						btScalar edgeAngle1 = edgeAngles[k];

						if (a0 < -edgeAngle + 0.0174 && a1 < -edgeAngle1 + 0.0174)
						{
							cpType = BT_CP_TYPE_VERTEX;

							if (isTireFr) {
								//printf("Result: Point on Vertex %d\n", pointOnVertex0 ? 0 : 1);
							}
						}
						else if (isTireFr) {
							//printf("Result: Point is on vertex but normal is out of its Voronoi region\n");
						}
					}
					else 
					{
						btScalar c = contactNormal.dot(worldNormal);
						// TODO: store the sine of the angle instead in the triangle info map or smth and get rid of this btAcos
						btScalar a = btAcos(c);

						// Add it if it's in the edge's voronoi region. Tolerance might require tunning. Small values 
						// seem to deteriorate contact persistence.
						if (a < -edgeAngle + 0.0174) 
						{
							if (isTireFr) {
								//printf("Result: Point lies on edge with current angle %f\n", a/SIMD_PI*180);
							}

							cpType = BT_CP_TYPE_EDGE;
						}
						else if (isTireFr) {
							//printf("Result: Point is on edge but normal is out of its Voronoi region\n");
						}
					}
				}
				else { // concave edge
					pointOnFace = contactNormal.dot(worldNormal) > 0.9998;

					if (isTireFr) {
						//printf("Result: Point is on concave edge\n");
					}
				}

				break; // no need to process other edges if this already contains the point
			}
		}

		if (pointOnFace)
		{ 
			cpType = BT_CP_TYPE_FACE;

			if (isTireFr) {
				//printf("Result: Point on Face | margin %f\n", m_collisionMarginTriangle);
			}
		}

		if (cpType != BT_CP_TYPE_NONE)
		{
			pt.m_cpType = cpType;

			if (swapped) 
			{
				pt.m_localPointB -= localTranslation;
			} else 
			{
				pt.m_localPointA -= localTranslation;
			}

			pt.m_positionWorldOnA -= translation;
			pt.m_positionWorldOnB -= translation;

			int insertIndex = originalManifold->getCacheEntry(pt);

			if (insertIndex >= 0)
			{
				btManifoldPoint& ptCached = originalManifold->getContactPoint(insertIndex);
				
				// If this point is on a face, always replace. If this point is on an edge, only replace
				// the similar point if it is on an edge or vertex. If this point is on a vertex, only
				// replace it if it is on vertex as well. This is key to fix problems like first adding
				// a contact to an edge on one triangle and then replacing it by a contact on a vertex on
				// another triangle which will cause the object to penetrate the edge while standing on the
				// other triangle's vertex.
				if (ptCached.m_cpType == BT_CP_TYPE_NONE || 
					(cpType == BT_CP_TYPE_FACE) ||
					(cpType == BT_CP_TYPE_EDGE && ptCached.m_cpType == BT_CP_TYPE_EDGE) ||
					(cpType == BT_CP_TYPE_EDGE && ptCached.m_cpType == BT_CP_TYPE_VERTEX) ||
					(cpType == BT_CP_TYPE_VERTEX && ptCached.m_cpType == BT_CP_TYPE_VERTEX)) 
				{
					originalManifold->replaceContactPoint(pt,insertIndex);
				}
			} else
			{
				originalManifold->addManifoldPoint(pt);
			}
		}
		else if (isTireFr) {
			//printf("No points added (!!!)\n");
		}
	}
	
	m_dispatcher->releaseManifold(tempManifold);

	if (m_resultOut->getBody0Internal() == m_triBodyWrap->getCollisionObject())
	{
		m_resultOut->setBody0Wrap(tmpWrap);
		m_resultOut->setBody1Wrap(tmpConvexWrap);
	} else
	{
		m_resultOut->setBody1Wrap(tmpWrap);
		m_resultOut->setBody0Wrap(tmpConvexWrap);
	}

	colAlgo->~btCollisionAlgorithm();
	m_dispatcher->freeCollisionAlgorithm(colAlgo);

}

void btConvexTriangleCallback::setTimeStepAndCounters(btScalar collisionMarginTriangle, const btDispatcherInfo& dispatchInfo, const btCollisionObjectWrapper* convexBodyWrap, const btCollisionObjectWrapper* triBodyWrap, btManifoldResult* resultOut)
{
	m_convexBodyWrap = convexBodyWrap;
	m_triBodyWrap = triBodyWrap;

	m_dispatchInfoPtr = &dispatchInfo;
	m_collisionMarginTriangle = collisionMarginTriangle;
	m_resultOut = resultOut;

	//recalc aabbs
	btTransform convexInTriangleSpace;
	convexInTriangleSpace = m_triBodyWrap->getWorldTransform().inverse() * m_convexBodyWrap->getWorldTransform();
	const btCollisionShape* convexShape = static_cast<const btCollisionShape*>(m_convexBodyWrap->getCollisionShape());
	//CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
	convexShape->getAabb(convexInTriangleSpace, m_aabbMin, m_aabbMax);
	btScalar extraMargin = collisionMarginTriangle + resultOut->m_closestPointDistanceThreshold;

	btVector3 extra(extraMargin, extraMargin, extraMargin);

	m_aabbMax += extra;
	m_aabbMin -= extra;
}

void btConvexConcaveCollisionAlgorithm::clearCache()
{
	m_btConvexTriangleCallback.clearCache();
}

void btConvexConcaveCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0Wrap, const btCollisionObjectWrapper* body1Wrap, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	BT_PROFILE("btConvexConcaveCollisionAlgorithm::processCollision");

	const btCollisionObjectWrapper* convexBodyWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const btCollisionObjectWrapper* triBodyWrap = m_isSwapped ? body0Wrap : body1Wrap;

	if (triBodyWrap->getCollisionShape()->isConcave())
	{
		if (triBodyWrap->getCollisionShape()->getShapeType() == SDF_SHAPE_PROXYTYPE)
		{
			btSdfCollisionShape* sdfShape = (btSdfCollisionShape*)triBodyWrap->getCollisionShape();
			if (convexBodyWrap->getCollisionShape()->isConvex())
			{
				btConvexShape* convex = (btConvexShape*)convexBodyWrap->getCollisionShape();
				btAlignedObjectArray<btVector3> queryVertices;

				if (convex->isPolyhedral())
				{
					btPolyhedralConvexShape* poly = (btPolyhedralConvexShape*)convex;
					for (int v = 0; v < poly->getNumVertices(); v++)
					{
						btVector3 vtx;
						poly->getVertex(v, vtx);
						queryVertices.push_back(vtx);
					}
				}
				btScalar maxDist = SIMD_EPSILON;

				if (convex->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
				{
					queryVertices.push_back(btVector3(0, 0, 0));
					btSphereShape* sphere = (btSphereShape*)convex;
					maxDist = sphere->getRadius() + SIMD_EPSILON;
				}
				if (queryVertices.size())
				{
					resultOut->setPersistentManifold(m_btConvexTriangleCallback.m_manifoldPtr);
					//m_btConvexTriangleCallback.m_manifoldPtr->clearManifold();

					btPolyhedralConvexShape* poly = (btPolyhedralConvexShape*)convex;
					for (int v = 0; v < queryVertices.size(); v++)
					{
						const btVector3& vtx = queryVertices[v];
						btVector3 vtxWorldSpace = convexBodyWrap->getWorldTransform() * vtx;
						btVector3 vtxInSdf = triBodyWrap->getWorldTransform().invXform(vtxWorldSpace);

						btVector3 normalLocal;
						btScalar dist;
						if (sdfShape->queryPoint(vtxInSdf, dist, normalLocal))
						{
							if (dist <= maxDist)
							{
								normalLocal.safeNormalize();
								btVector3 normal = triBodyWrap->getWorldTransform().getBasis() * normalLocal;

								if (convex->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
								{
									btSphereShape* sphere = (btSphereShape*)convex;
									dist -= sphere->getRadius();
									vtxWorldSpace -= sphere->getRadius() * normal;
								}
								resultOut->addContactPoint(normal, vtxWorldSpace - normal * dist, dist);
							}
						}
					}
					resultOut->refreshContactPoints();
				}
			}
		}
		else
		{
			const btConcaveShape* concaveShape = static_cast<const btConcaveShape*>(triBodyWrap->getCollisionShape());

			if (convexBodyWrap->getCollisionShape()->isConvex())
			{
				btScalar collisionMarginTriangle = concaveShape->getMargin();

				bool isTireFR = convexBodyWrap->getCollisionObject()->getUserIndex() == 99135 && convexBodyWrap->m_index == 0;
				if (isTireFR) {
					//printf("=====================================================\n");
				}
				
				// Use a clean manifold to insert the current contact points
				btPersistentManifold* originalManifold = m_btConvexTriangleCallback.m_manifoldPtr;
				originalManifold->setBodies(convexBodyWrap->getCollisionObject(),triBodyWrap->getCollisionObject());

				btPersistentManifold* tempManifold = m_dispatcher->getNewManifold(originalManifold->getBody0(),originalManifold->getBody1());
				resultOut->setPersistentManifold(tempManifold);

				m_btConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle,dispatchInfo,convexBodyWrap,triBodyWrap,resultOut);

				concaveShape->processAllTriangles( &m_btConvexTriangleCallback,m_btConvexTriangleCallback.getAabbMin(),m_btConvexTriangleCallback.getAabbMax());
			
				// Reset to the original manifold and insert all points into it the same way it's done in btManifoldResult::addContactPoint
				resultOut->setPersistentManifold(originalManifold);

				for (int i = 0; i < tempManifold->getNumContacts(); ++i) 
				{
					btManifoldPoint& pt = tempManifold->getContactPoint(i);
					int insertIndex = originalManifold->getCacheEntry(pt);
					
					if (insertIndex >= 0)
					{
						originalManifold->replaceContactPoint(pt,insertIndex);
						if (isTireFR) {
						 //printf("*** Replaced ");
						}
					} else
					{
						originalManifold->addManifoldPoint(pt);
						if (isTireFR) {
							//printf("*** Added ");
						}
					}

					if (isTireFR) {
						//printf("Point %d ************\n", i);
						//printf("m_localPointA %f %f %f\n", pt.m_localPointA.x(), pt.m_localPointA.y(), pt.m_localPointA.z());
						//printf("m_localPointB %f %f %f\n", pt.m_localPointB.x(), pt.m_localPointB.y(), pt.m_localPointB.z());
					}
				}

				m_dispatcher->releaseManifold(tempManifold);

				resultOut->refreshContactPoints();

				m_btConvexTriangleCallback.clearWrapperData();
			}
		}
	}
}

btScalar btConvexConcaveCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1, const btDispatcherInfo& dispatchInfo, btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	btCollisionObject* convexbody = m_isSwapped ? body1 : body0;
	btCollisionObject* triBody = m_isSwapped ? body0 : body1;

	//quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

	//only perform CCD above a certain threshold, this prevents blocking on the long run
	//because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
	btScalar squareMot0 = (convexbody->getInterpolationWorldTransform().getOrigin() - convexbody->getWorldTransform().getOrigin()).length2();
	if (squareMot0 < convexbody->getCcdSquareMotionThreshold())
	{
		return btScalar(1.);
	}

	//const btVector3& from = convexbody->m_worldTransform.getOrigin();
	//btVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
	//todo: only do if the motion exceeds the 'radius'

	btTransform triInv = triBody->getWorldTransform().inverse();
	btTransform convexFromLocal = triInv * convexbody->getWorldTransform();
	btTransform convexToLocal = triInv * convexbody->getInterpolationWorldTransform();

	struct LocalTriangleSphereCastCallback : public btTriangleCallback
	{
		btTransform m_ccdSphereFromTrans;
		btTransform m_ccdSphereToTrans;
		btTransform m_meshTransform;

		btScalar m_ccdSphereRadius;
		btScalar m_hitFraction;

		LocalTriangleSphereCastCallback(const btTransform& from, const btTransform& to, btScalar ccdSphereRadius, btScalar hitFraction)
			: m_ccdSphereFromTrans(from),
			  m_ccdSphereToTrans(to),
			  m_ccdSphereRadius(ccdSphereRadius),
			  m_hitFraction(hitFraction)
		{
		}

		virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
		{
			BT_PROFILE("processTriangle");
			(void)partId;
			(void)triangleIndex;
			//do a swept sphere for now
			btTransform ident;
			ident.setIdentity();
			btConvexCast::CastResult castResult;
			castResult.m_fraction = m_hitFraction;
			btSphereShape pointShape(m_ccdSphereRadius);
			btTriangleShape triShape(triangle[0], triangle[1], triangle[2]);
			btVoronoiSimplexSolver simplexSolver;
			btSubsimplexConvexCast convexCaster(&pointShape, &triShape, &simplexSolver);
			//GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
			//ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
			//local space?

			if (convexCaster.calcTimeOfImpact(m_ccdSphereFromTrans, m_ccdSphereToTrans,
											  ident, ident, castResult))
			{
				if (m_hitFraction > castResult.m_fraction)
					m_hitFraction = castResult.m_fraction;
			}
		}
	};

	if (triBody->getCollisionShape()->isConcave())
	{
		btVector3 rayAabbMin = convexFromLocal.getOrigin();
		rayAabbMin.setMin(convexToLocal.getOrigin());
		btVector3 rayAabbMax = convexFromLocal.getOrigin();
		rayAabbMax.setMax(convexToLocal.getOrigin());
		btScalar ccdRadius0 = convexbody->getCcdSweptSphereRadius();
		rayAabbMin -= btVector3(ccdRadius0, ccdRadius0, ccdRadius0);
		rayAabbMax += btVector3(ccdRadius0, ccdRadius0, ccdRadius0);

		btScalar curHitFraction = btScalar(1.);  //is this available?
		LocalTriangleSphereCastCallback raycastCallback(convexFromLocal, convexToLocal,
														convexbody->getCcdSweptSphereRadius(), curHitFraction);

		raycastCallback.m_hitFraction = convexbody->getHitFraction();

		btCollisionObject* concavebody = triBody;

		btConcaveShape* triangleMesh = (btConcaveShape*)concavebody->getCollisionShape();

		if (triangleMesh)
		{
			triangleMesh->processAllTriangles(&raycastCallback, rayAabbMin, rayAabbMax);
		}

		if (raycastCallback.m_hitFraction < convexbody->getHitFraction())
		{
			convexbody->setHitFraction(raycastCallback.m_hitFraction);
			return raycastCallback.m_hitFraction;
		}
	}

	return btScalar(1.);
}
