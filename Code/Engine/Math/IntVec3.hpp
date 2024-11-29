#pragma once

struct IntVec3
{
public:
	int				x = 0;
	int				y = 0;
	int				z = 0;

	static IntVec3 const ZERO;
	static IntVec3 const ONE;

	static IntVec3 const NORTH;
	static IntVec3 const SOUTH;
	static IntVec3 const EAST;
	static IntVec3 const WEST;
	static IntVec3 const UP;
	static IntVec3 const DOWN;

public:
					IntVec3() {}
					IntVec3(IntVec3 const& copyFrom);
	explicit		IntVec3(int initialX, int initialY, int initialZ);
					~IntVec3() {}

	float			 GetLength() const;

	IntVec3	const	operator+(IntVec3 const& copyFrom);
	IntVec3 const	operator-(IntVec3 const& copyFrom);

	bool				operator==(IntVec3 const& compare) const;
};