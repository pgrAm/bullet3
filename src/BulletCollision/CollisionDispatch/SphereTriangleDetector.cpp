/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "LinearMath/btScalar.h"
#include "SphereTriangleDetector.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "LinearMath/btQuickprof.h"

SphereTriangleDetector::SphereTriangleDetector(btSphereShape* sphere, btTriangleShape* triangle, btScalar contactBreakingThreshold)
	: m_sphere(sphere),
	  m_triangle(triangle),
	  m_contactBreakingThreshold(contactBreakingThreshold)
{
}

void SphereTriangleDetector::getClosestPoints(const ClosestPointInput& input, Result& output, class btIDebugDraw* debugDraw, bool swapResults)
{
	(void)debugDraw;
	const btTransform& transformA = input.m_transformA;
	const btTransform& transformB = input.m_transformB;

	btVector3 point, normal;
	btScalar timeOfImpact = btScalar(1.);
	btScalar depth = btScalar(0.);
	//	output.m_distance = btScalar(BT_LARGE_FLOAT);
	//move sphere into triangle space
	btTransform sphereInTr = transformB.inverseTimes(transformA);

	if (collide(sphereInTr.getOrigin(), point, normal, depth, timeOfImpact, m_contactBreakingThreshold))
	{
		if (swapResults)
		{
			btVector3 normalOnB = transformB.getBasis() * normal;
			btVector3 normalOnA = -normalOnB;
			btVector3 pointOnA = transformB * point + normalOnB * depth;
			output.addContactPoint(normalOnA, pointOnA, depth);
		}
		else
		{
			output.addContactPoint(transformB.getBasis() * normal, transformB * point, depth);
		}
	}
}

// See also geometrictools.com
// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
btScalar SegmentSqrDistance(const btVector3& from, const btVector3& to, const btVector3& p, btVector3& nearest);

btScalar SegmentSqrDistance(const btVector3& from, const btVector3& to, const btVector3& p, btVector3& nearest)
{
	btVector3 diff = p - from;
	btVector3 v = to - from;
	btScalar t = v.dot(diff);

	if (t > 0)
	{
		btScalar dotVV = v.dot(v);
		if (t < dotVV)
		{
			t /= dotVV;
			diff -= t * v;
		}
		else
		{
			t = 1;
			diff -= v;
		}
	}
	else
		t = 0;

	nearest = from + t * v;
	return diff.dot(diff);
}

static btScalar SegmentSqrDistance_edge(btVector3 from, btVector3 diff, btVector3 v /*to - from*/, btVector3& nearest)
{
	btScalar t = v.dot(diff);

	if (t > 0)
	{
		btScalar dotVV = v.dot(v);
		if (t < dotVV)
		{
			t /= dotVV;
			diff -= t * v;
		}
		else
		{
			t = 1;
			diff -= v;
		}
	}
	else
		t = 0;

	nearest = from + t * v;
	return diff.dot(diff);
}

static bool SegmentSqrDistance_edge_check(btVector3 from, btVector3 diff, btVector3 v /*to - from*/, btVector3& nearest, btScalar& minSqrDist)
{
	btScalar t = v.dot(diff);

	if (t > 0)
	{
		btScalar dotVV = v.dot(v);
		if (t < dotVV)
		{
			t /= dotVV;
			diff -= t * v;
		}
		else
		{
			t = 1;
			diff -= v;
		}
	}
	else
		t = 0;

	btScalar sqrDist = diff.dot(diff);

	if (sqrDist < minSqrDist)
	{
		minSqrDist = sqrDist;
		nearest = from + t * v;
		return true;
	}

	return false;
}

bool SphereTriangleDetector::facecontains(const btVector3& p, const btVector3* vertices, btVector3& normal)
{
	btVector3 lp(p);
	btVector3 lnormal(normal);

	return pointInTriangle(vertices, lnormal, &lp);
}


inline bool pointInTriangle_edges(btVector3 edge1, btVector3 edge2, btVector3 edge3, btVector3 p1_to_p, btVector3 p2_to_p, btVector3 p3_to_p, btVector3 normal)
{
	btVector3 edge1_normal(edge1.cross(normal));
	btVector3 edge2_normal(edge2.cross(normal));
	btVector3 edge3_normal(edge3.cross(normal));

	btScalar r1, r2, r3;
	r1 = edge1_normal.dot(p1_to_p);
	r2 = edge2_normal.dot(p2_to_p);
	r3 = edge3_normal.dot(p3_to_p);
	if ((r1 > 0 && r2 > 0 && r3 > 0) ||
		(r1 <= 0 && r2 <= 0 && r3 <= 0))
		return true;
	return false;
}

bool SphereTriangleDetector::collide(const btVector3& sphereCenter, btVector3& point, btVector3& resultNormal, btScalar& depth, btScalar& timeOfImpact, btScalar contactBreakingThreshold)
{
	BT_PROFILE("SphereTriangleDetector::collide");

	const btVector3* vertices = &m_triangle->getVertexPtr(0);

	btVector3 normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);

	btScalar l2 = normal.length2();

	if (l2 < SIMD_EPSILON * SIMD_EPSILON)
		return false;

	bool hasContact = false;
	btVector3 contactPoint;

	normal /= btSqrt(l2);

	btVector3 p1ToCentre = sphereCenter - vertices[0];
	btScalar distanceFromPlane = p1ToCentre.dot(normal);

	if (distanceFromPlane < btScalar(0.))
	{
		//triangle facing the other way
		distanceFromPlane *= btScalar(-1.);
		normal *= btScalar(-1.);
	}

	btScalar radius = m_sphere->getRadius();
	btScalar radiusWithThreshold = radius + contactBreakingThreshold;

	bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;

	// Check for contact / intersection

	if (isInsideContactPlane)
	{
		const btVector3 p = sphereCenter;

		const btVector3* p1 = &vertices[0];
		const btVector3* p2 = &vertices[1];
		const btVector3* p3 = &vertices[2];

		btVector3 edge1(*p2 - *p1);
		btVector3 edge2(*p3 - *p2);
		btVector3 edge3(*p1 - *p3);

		btVector3 p1_to_p(p - *p1);
		btVector3 p2_to_p(p - *p2);
		btVector3 p3_to_p(p - *p3);

		if (pointInTriangle_edges(edge1, edge2, edge3, p1_to_p, p2_to_p, p3_to_p, normal))
		{
			// Inside the contact wedge - touches a point on the shell plane
			hasContact = true;
			contactPoint = sphereCenter - normal * distanceFromPlane;
		}
		else
		{
			// Could be inside one of the contact capsules
			btScalar contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold;
			btScalar minDistSqr = contactCapsuleRadiusSqr;

			#if 1
			hasContact = hasContact || SegmentSqrDistance_edge_check(*p1, p1_to_p, edge1, contactPoint, minDistSqr);
			hasContact = hasContact || SegmentSqrDistance_edge_check(*p2, p2_to_p, edge2, contactPoint, minDistSqr);
			hasContact = hasContact || SegmentSqrDistance_edge_check(*p3, p3_to_p, edge3, contactPoint, minDistSqr);
			#else
			btVector3 nearestOnEdge;

			if (btScalar distanceSqr = SegmentSqrDistance_edge(*p1, p1_to_p, edge1, nearestOnEdge); distanceSqr < minDistSqr)
			{
				// Yep, we're inside a capsule, and record the capsule with smallest distance
				minDistSqr = distanceSqr;
				hasContact = true;
				contactPoint = nearestOnEdge;
			}
			
			if (btScalar distanceSqr = SegmentSqrDistance_edge(*p2, p2_to_p, edge2, nearestOnEdge); distanceSqr < minDistSqr)
			{
				// Yep, we're inside a capsule, and record the capsule with smallest distance
				minDistSqr = distanceSqr;
				hasContact = true;
				contactPoint = nearestOnEdge;
			}
			
			if (btScalar distanceSqr = SegmentSqrDistance_edge(*p3, p3_to_p, edge3, nearestOnEdge); distanceSqr < minDistSqr)
			{
				// Yep, we're inside a capsule, and record the capsule with smallest distance
				//minDistSqr = distanceSqr;
				hasContact = true;
				contactPoint = nearestOnEdge;
			}
			#endif
		}
	}

	if (hasContact)
	{
		btVector3 contactToCentre = sphereCenter - contactPoint;
		btScalar distanceSqr = contactToCentre.length2();

		if (distanceSqr < radiusWithThreshold * radiusWithThreshold)
		{
			if (distanceSqr > SIMD_EPSILON)
			{
				btScalar distance = btSqrt(distanceSqr);
				resultNormal = contactToCentre;
				resultNormal /= distance;  //.normalize();
				point = contactPoint;
				depth = -(radius - distance);
			}
			else
			{
				resultNormal = normal;
				point = contactPoint;
				depth = -radius;
			}
			return true;
		}
	}

	return false;
}

bool SphereTriangleDetector::pointInTriangle(const btVector3 vertices[], const btVector3& normal, btVector3* p)
{
	const btVector3* p1 = &vertices[0];
	const btVector3* p2 = &vertices[1];
	const btVector3* p3 = &vertices[2];

	btVector3 edge1(*p2 - *p1);
	btVector3 edge2(*p3 - *p2);
	btVector3 edge3(*p1 - *p3);

	btVector3 p1_to_p(*p - *p1);
	btVector3 p2_to_p(*p - *p2);
	btVector3 p3_to_p(*p - *p3);

	return pointInTriangle_edges(edge1, edge2, edge3, p1_to_p, p2_to_p, p3_to_p, normal);
}
