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

#ifndef BT_SOLVER_CONSTRAINT_H
#define BT_SOLVER_CONSTRAINT_H

class btRigidBody;
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "btJacobianEntry.h"
#include "LinearMath/btAlignedObjectArray.h"

//#define NO_FRICTION_TANGENTIALS 1
#include "btSolverBody.h"

enum btConstraintFlags
{
	BT_CONSTRAINT_USE_SPLIT_SPIN_BODY_A = 1,
	BT_CONSTRAINT_USE_SPLIT_SPIN_BODY_B = 2,
	BT_CONSTRAINT_USE_SPLIT_SPIN_BODY_C = 3,
};

///1D constraint along a normal axis between bodyA and bodyB. It can be combined to solve contact and friction constraints.
ATTRIBUTE_ALIGNED16(struct)
btSolverConstraint
{
	// Use an integer type that's the same size as btScalar because of the way btConstraintInfo2 works with its
	// `rowskip` thing. It's expected that the size of btConstraintInfo2 is a multiple of the size of btScalar.
#ifdef BT_USE_DOUBLE_PRECISION
	typedef int64_t IntegerType;
#else 
	typedef int32_t IntegerType;
#endif

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btVector3 m_relpos1CrossNormal;
	btVector3 m_contactNormal1;

	btVector3 m_relpos2CrossNormal;
	btVector3 m_contactNormal2;  //usually m_contactNormal2 == -m_contactNormal1, but not always

	btVector3 m_relpos3CrossNormal;
	btVector3 m_contactNormal3;

	btVector3 m_angularComponentA;
	btVector3 m_angularComponentB;
	btVector3 m_angularComponentC;

	mutable btSimdScalar m_appliedPushImpulse;
	mutable btSimdScalar m_appliedImpulse;

	btScalar m_friction;
	btScalar m_jacDiagABInv;
	btScalar m_rhs;
	btScalar m_cfm;

	btScalar m_lowerLimit;
	btScalar m_upperLimit;
	btScalar m_rhsPenetration;
	union {
		void* m_originalContactPoint;
		btScalar m_unusedPadding4;
		IntegerType m_numRowsForNonContactConstraint;
	};

	IntegerType m_overrideNumSolverIterations;
	IntegerType m_frictionIndex;
	IntegerType m_solverBodyIdA;
	IntegerType m_solverBodyIdB;
	IntegerType m_solverBodyIdC;

	enum btSolverConstraintType
	{
		BT_SOLVER_CONTACT_1D = 0,
		BT_SOLVER_FRICTION_1D
	};

	IntegerType m_flags;
};

typedef btAlignedObjectArray<btSolverConstraint> btConstraintArray;

#endif  //BT_SOLVER_CONSTRAINT_H
