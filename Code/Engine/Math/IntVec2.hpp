#pragma once

struct IntVec2
{
public:
	int				x = 0;
	int				y = 0;

	static IntVec2 const ZERO;
	static IntVec2 const ONE;

	static IntVec2 const NORTH;
	static IntVec2 const SOUTH;
	static IntVec2 const EAST;
	static IntVec2 const WEST;
	static IntVec2 const NORTH_EAST;
	static IntVec2 const NORTH_WEST;
	static IntVec2 const SOUTH_EAST;
	static IntVec2 const SOUTH_WEST;

public:
					IntVec2() {}
					IntVec2(IntVec2 const& copyFrom);
	explicit		IntVec2(int initialX, int initialY);
					~IntVec2() {}

	float			GetLength() const;
	int				GetTaxicabLength() const;
	int				GetLengthSquared() const;
	float			GetOrientationRadians() const;
	float			GetOrientationDegrees() const;
	IntVec2 const	GetRotated90Degrees() const;
	IntVec2 const	GetRotatedMinus90Degrees() const;

	void			Rotate90Degrees();
	void			RotateMinus90Degrees();

	IntVec2			SetFromText(char const* text);

	IntVec2	const	operator+(IntVec2 const& copyFrom);
	IntVec2 const	operator-(IntVec2 const& copyFrom);
	bool			operator==( IntVec2 const& compare ) const;		// vec2 == vec2
	void			operator=( IntVec2 const& copyFrom );				// vec2 = vec2

};