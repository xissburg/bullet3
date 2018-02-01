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

#include "btSphereSegmentShape.h"

btSphereSegmentShape::btSphereSegmentShape(btScalar radius, btScalar halfWidth)
:btConvexInternalShape(),
m_upAxis(0)
{
	btVector3 margin(getMargin(),getMargin(),getMargin());
    btVector3 halfExtents(halfWidth, radius, radius);
	m_implicitShapeDimensions = halfExtents * m_localScaling - margin;

	setSafeMargin(halfExtents);

	m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;
}

void btSphereSegmentShape::getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
{
	btTransformAabb(getHalfExtentsWithoutMargin(),getMargin(),t,aabbMin,aabbMax);
}

void btSphereSegmentShape::calculateLocalInertia(btScalar mass,btVector3& inertia) const
{
	/*
    Use inertia of a cylinder.
	Cylinder is defined as following:
	*
	* - principle axis aligned along y by default, radius in x, z-value not used
	* - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
	* - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used
	*
	*/

	btScalar radius2;	// square of cylinder radius
	btScalar height2;	// square of cylinder height
	btVector3 halfExtents = getHalfExtentsWithMargin();	// get cylinder dimension
	btScalar div12 = mass / 12.f;
	btScalar div4 = mass / 4.f;
	btScalar div2 = mass / 2.f;
	int idxRadius, idxHeight;

	switch (m_upAxis)	// get indices of radius and height of cylinder
	{
		case 0:		// cylinder is aligned along x
			idxRadius = 1;
			idxHeight = 0;
			break;
		case 2:		// cylinder is aligned along z
			idxRadius = 0;
			idxHeight = 2;
			break;
		default:	// cylinder is aligned along y
			idxRadius = 0;
			idxHeight = 1;
	}

	// calculate squares
	radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
	height2 = btScalar(4.) * halfExtents[idxHeight] * halfExtents[idxHeight];

	// calculate tensor terms
	btScalar t1 = div12 * height2 + div4 * radius2;
	btScalar t2 = div2 * radius2;

	switch (m_upAxis)	// set diagonal elements of inertia tensor
	{
		case 0:		// cylinder is aligned along x
			inertia.setValue(t2,t1,t1);
			break;
		case 2:		// cylinder is aligned along z
			inertia.setValue(t1,t1,t2);
			break;
		default:	// cylinder is aligned along y
			inertia.setValue(t1,t2,t1);
	}
}

SIMD_FORCE_INLINE btVector3 SphereSegmentLocalSupportX(const btVector3& halfExtents,const btVector3& v) 
{
    btScalar radius = halfExtents[1];
    btScalar vLenSq = v.length2();
    btVector3 p(0,0,0);

    if (vLenSq > SIMD_EPSILON) {
        p = v * (radius / btSqrt(vLenSq));
        btScalar halfWidth = halfExtents[0];

        if (btFabs(p.x()) > halfWidth) {
            p.setX(0);
            btScalar pLenSq = p.length2();

            btScalar sideRadius = btSqrt(radius*radius - halfWidth*halfWidth);

            if (pLenSq > SIMD_EPSILON) {
                p *= sideRadius / btSqrt(pLenSq);
                p.setX(halfWidth);
            }
            else {
                p.setX(v.x() < 0 ? -halfWidth : halfWidth);
                p.setY(sideRadius);
                p.setZ(0);
            }
        }
    }

    return p;
}

btVector3 btSphereSegmentShape::localGetSupportingVertexWithoutMargin(const btVector3& vec)const
{
	return SphereSegmentLocalSupportX(getHalfExtentsWithoutMargin(),vec);
}

void btSphereSegmentShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = SphereSegmentLocalSupportX(getHalfExtentsWithoutMargin(),vectors[i]);
	}
}