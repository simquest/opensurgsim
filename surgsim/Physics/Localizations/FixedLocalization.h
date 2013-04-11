#include "Localization.h"

class FixedLocalization : Localization
{
public:
	FixedLocalization();
	virtual ~FixedLocalization();
private:
	virtual bool isEqual(const Localization& localization) const = 0;
};

struct FixedConstraintLocalization : public ConstraintLocalization
{
	double Position[3];
	double Twist;
	int ElementID; //! Useful for suturable to locate in which OctreeNode the constraint is located
	FixedConstraintLocalization() {}
	FixedConstraintLocalization(const FixedConstraintLocalization& other)
	{
		*this = other;
	}
	bool operator==(const FixedConstraintLocalization& other) const
	{
		return ( Twist == other.Twist && 
			Position[0] == other.Position[0] && Position[1] == other.Position[1] && Position[2] == other.Position[2] &&
			ElementID == other.ElementID);
	}
	FixedConstraintLocalization& operator=(const FixedConstraintLocalization& other)
	{
		ElementID = other.ElementID;
		Twist = other.Twist;
		for (int i=0; i<3; ++i)
			Position[i] = other.Position[i];
		return *this;
	}
	virtual ConstraintLocalizationType getType() const
	{ return FIXED_LOCALIZATION; }
};
