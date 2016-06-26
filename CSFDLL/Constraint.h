#ifndef _CONSTRAINT_H_
#define _CONSTRAINT_H_

#include "Vec3.h"
#include "Particle.h"
//每次布料节点回弹的幅度
//当只有一端移动的时候
const float singleMove[14] = { 0.4, 0.64, 0.784, 0.8704, 0.92224, 0.95334, 0.97201, 0.9832, 0.98992, 0.99395, 0.99637, 0.99782, 0.99869, 0.99922};
//当有两端移动时
const float doubleMove[14] = { 0.4, 0.48, 0.496, 0.4992, 0.49984, 0.49997, 0.49999, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

class Constraint
{
private:
	double rest_distance; // the length between particle p1 and p2 in rest configuration

public:
	Particle *p1, *p2; // the two particles that are connected through this constraint

	Constraint(Particle *p1, Particle *p2) : p1(p1), p2(p2)
	{
	//	Vec3 vec = p1->getPos() - p2->getPos();
	//	rest_distance = vec.length();
	}

	/* This is one of the important methods, where a single constraint between two particles p1 and p2 is solved
	the method is called by Cloth.time_step() many times per frame*/
	void satisfyConstraint();
	//void satisfyConstraint(int constraintTimes);
};




#endif