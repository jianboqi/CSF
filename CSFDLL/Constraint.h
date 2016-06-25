#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include "Vec3.h"
#include "Particle.h"
//每次布料节点回弹的幅度
//当只有一端移动的时候
const float singleMove[14] = { 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765, 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322};
//当有两端移动时
const float doubleMove[14] = { 0.3, 0.42, 0.468, 0.4872, 0.49488, 0.49795, 0.49918, 0.49967, 0.49987, 0.49995, 0.49998, 0.49999, 0.5, 0.5};

class Constraint
{
private:
	double rest_distance; // the length between particle p1 and p2 in rest configuration

public:
	Particle *p1, *p2; // the two particles that are connected through this constraint

	Constraint(Particle *p1, Particle *p2) : p1(p1), p2(p2)
	{
		Vec3 vec = p1->getPos() - p2->getPos();
		rest_distance = vec.length();
	}

	/* This is one of the important methods, where a single constraint between two particles p1 and p2 is solved
	the method is called by Cloth.time_step() many times per frame*/
	void satisfyConstraint(int constraintTimes);
};




#endif