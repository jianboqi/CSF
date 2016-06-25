#include "Constraint.h"


void Constraint::satisfyConstraint(int constraintTimes)
{
	Vec3 correctionVector(0, p2->pos.f[1] - p1->pos.f[1], 0);
	if (p1->isMovable() && p2->isMovable())
	{
		Vec3 correctionVectorHalf = correctionVector * (constraintTimes>14 ? 0.5 : doubleMove[constraintTimes-1]); // Lets make it half that length, so that we can move BOTH p1 and p2.
		p1->offsetPos(correctionVectorHalf);
		p2->offsetPos(-correctionVectorHalf);
	}
	else if (p1->isMovable() && !p2->isMovable())
	{
		Vec3 correctionVectorHalf = correctionVector * (constraintTimes>14 ? 1 : singleMove[constraintTimes - 1]); // Lets make it half that length, so that we can move BOTH p1 and p2.
		p1->offsetPos(correctionVectorHalf);
	}
	else if (!p1->isMovable() && p2->isMovable())
	{
		Vec3 correctionVectorHalf = correctionVector * (constraintTimes>14 ? 1 : singleMove[constraintTimes - 1]); // Lets make it half that length, so that we can move BOTH p1 and p2.
		p2->offsetPos(-correctionVectorHalf);
	}
}


