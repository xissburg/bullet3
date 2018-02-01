/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SPHERE_SEGMENT_SHAPE
#define BT_SPHERE_SEGMENT_SHAPE

#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btVector3.h"

ATTRIBUTE_ALIGNED16(class) btSphereSegmentShape : public btConvexInternalShape
{
protected:
	int	m_upAxis;

public:
BT_DECLARE_ALIGNED_ALLOCATOR();

    btSphereSegmentShape(btScalar radius, btScalar halfWidth);

    btVector3 getHalfExtentsWithMargin() const
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();
		btVector3 margin(getMargin(),getMargin(),getMargin());
		halfExtents += margin;
		return halfExtents;
	}
	
	const btVector3& getHalfExtentsWithoutMargin() const
	{
		return m_implicitShapeDimensions;
	}

    void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

	virtual void calculateLocalInertia(btScalar mass,btVector3& inertia) const;

	virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

	virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;

	virtual void setMargin(btScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		btVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		
		btConvexInternalShape::setMargin(collisionMargin);
		btVector3 newMargin(getMargin(),getMargin(),getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
	}

	virtual btVector3 localGetSupportingVertex(const btVector3& vec) const
	{
		btVector3 supVertex;
		supVertex = localGetSupportingVertexWithoutMargin(vec);
		
		if (getMargin() != btScalar(0.))
		{
			btVector3 vecnorm = vec;
			if (vecnorm.length2() < (SIMD_EPSILON*SIMD_EPSILON))
			{
				vecnorm.setValue(btScalar(-1.),btScalar(-1.),btScalar(-1.));
			} 
			vecnorm.normalize();
			supVertex += getMargin() * vecnorm;
		}
		return supVertex;
	}

	int	getUpAxis() const
	{
		return m_upAxis;
	}

	virtual btVector3 getAnisotropicRollingFrictionDirection() const
	{
		btVector3 aniDir(0,0,0);
		aniDir[getUpAxis()]=1;
		return aniDir;
	}

	btScalar getRadius() const
	{
		return getHalfExtentsWithMargin().y();
	}

    void setRadius(btScalar r)
    {
        r -= getMargin();
        m_implicitShapeDimensions.setY(r);
        m_implicitShapeDimensions.setZ(r);
    }

    btScalar getWidth() const 
    {
        return getHalfExtentsWithMargin().x() * 2;
    }

    void setWidth(btScalar w) 
    {
        w -= getMargin()*2;
        m_implicitShapeDimensions.setX(w/2);
    }

	virtual void setLocalScaling(const btVector3& scaling)
	{
		btVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		btConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
	}

	//debugging
	virtual const char*	getName()const
	{
		return "SphereSegmentX";
	}
};

#endif // BT_SPHERE_SEGMENT_SHAPE