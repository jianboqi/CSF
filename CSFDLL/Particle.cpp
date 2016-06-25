
#include "Particle.h"
/* This is one of the important methods, where the time is progressed a single step size (TIME_STEPSIZE)
The method is called by Cloth.time_step()
Given the equation "force = mass * acceleration" the next position is found through verlet integration*/
void Particle::timeStep()
{
	if (movable)
	{
		Vec3 temp = pos;
		pos = pos + (pos - old_pos)*(1.0 - DAMPING) + acceleration*time_step2;
		old_pos = temp;
		//acceleration = Vec3(0, 0, 0); // acceleration is reset since it HAS been translated into a change in position (and implicitely into velocity)	
	}
}

//取周围领域的平均向量
void Particle::satisfyConstraintSelf(int constraintTimes)
{
	if (isMovable())
	{
		double tmpY = pos.f[1];
		Vec3 total = Vec3(0, 0, 0);
		for (int i = 0; i < neighborsList.size(); i++)
		{
			Particle * p2 = neighborsList[i];
			Vec3 correctionVector(0, p2->pos.f[1] - tmpY, 0);
			Vec3 correctionVectorHalf = Vec3(0, 0, 0);
			if (p2->isMovable())
			{
				correctionVectorHalf = correctionVector * (constraintTimes>14 ? 0.5 : doubleMove1[constraintTimes]); // Lets make it half that length, so that we can move BOTH p1 and p2.
				tmpY += correctionVectorHalf.f[1];
				p2->offsetPos(-correctionVectorHalf);
			}
			else if (!p2->isMovable())
			{
				correctionVectorHalf = correctionVector * (constraintTimes>14 ? 1 : singleMove1[constraintTimes]); // Lets make it half that length, so that we can move BOTH p1 and p2.
				tmpY += correctionVectorHalf.f[1];
			}
			total = total + correctionVectorHalf;		
		}

		this->offsetPos(total);
	}
	
}


