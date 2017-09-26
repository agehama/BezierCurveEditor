//Copied from Siv3D.

#pragma once
#include <Siv3d.hpp>

struct DMat3x2
{
	double _11, _12;
	double _21, _22;
	double _31, _32;

	DMat3x2() = default;

	constexpr DMat3x2(double _11, double _12, double _21, double _22, double _31, double _32)
		: _11{ _11 }
		, _12{ _12 }
		, _21{ _21 }
		, _22{ _22 }
		, _31{ _31 }
		, _32{ _32 } {}

	static constexpr DMat3x2 Identity()
	{
		return DMat3x2(1.0, 0.0,
			0.0, 1.0,
			0.0, 0.0);
	}

	static constexpr DMat3x2 Translate(const Vec2& v)
	{
		return DMat3x2(1.0, 0.0,
			0.0, 1.0,
			v.x, v.y);
	}

	static constexpr DMat3x2 Translate(double x, double y)
	{
		return Translate({ x, y });
	}

	static constexpr DMat3x2 Scale(const Vec2& scale, const Vec2& center = { 0,0 })
	{
		return DMat3x2(scale.x, 0.0,
			0.0, scale.y,
			center.x - scale.x * center.x, center.y - scale.x * center.y);
	}

	static constexpr DMat3x2 Scale(double s, const Vec2& center = { 0,0 })
	{
		return Scale({ s, s }, center);
	}

	static constexpr DMat3x2 Scale(double sx, double sy, const Vec2& center = { 0,0 })
	{
		return Scale({ sx, sy }, center);
	}

	static DMat3x2 Rotate(double angle, const Vec2& center = { 0,0 })
	{
		DMat3x2 m;
		const double s = std::sinf(static_cast<double>(angle));
		const double c = std::cosf(static_cast<double>(angle));
		m._11 = c;
		m._12 = s;
		m._21 = -s;
		m._22 = c;
		m._31 = center.x - center.x * c + center.y * s;
		m._32 = center.y - center.x * s - center.y * c;
		return m;
	}

	static constexpr DMat3x2 Screen(double width, double height)
	{
		return Screen({ width, height });
	}

	static constexpr DMat3x2 Screen(const Vec2& size)
	{
		return DMat3x2(2.0 / size.x, 0.0,
			0.0, -2.0 / size.y,
			-1.0, 1.0);
	}

	DMat3x2 translate(const Vec2& v) const
	{
		DMat3x2 result;
		result.setProduct(*this, Translate(v));
		return result;
	}

	DMat3x2 translate(double x, double y) const
	{
		return translate({ x, y });
	}

	DMat3x2 scale(double s, const Vec2& center = { 0,0 }) const
	{
		return scale({ s, s }, center);
	}

	DMat3x2 scale(const Vec2& scale, const Vec2& center = { 0,0 }) const
	{
		DMat3x2 result;
		result.setProduct(*this, Scale(scale, center));
		return result;
	}

	DMat3x2 scale(double sx, double sy, const Vec2& center = { 0,0 }) const
	{
		return scale({ sx, sy }, center);
	}

	DMat3x2 rotate(double angle, const Vec2& center = { 0,0 }) const
	{
		DMat3x2 result;
		result.setProduct(*this, Rotate(angle, center));
		return result;
	}

	constexpr double determinant() const
	{
		return (_11 * _22) - (_12 * _21);
	}

	DMat3x2 inverse() const
	{
		const double det = determinant();
		assert(det != 0.0);
		const double detInv = 1.0 / det;

		DMat3x2 out;
		out._11 = (_22)* detInv;
		out._12 = (-_12) * detInv;
		out._21 = (-_21) * detInv;
		out._22 = (_11)* detInv;
		out._31 = (_21*_32 - _22*_31) * detInv;
		out._32 = (_12*_31 - _11*_32) * detInv;
		return out;
	}

	void setProduct(const DMat3x2 &a, const DMat3x2 &b)
	{
		_11 = a._11 * b._11 + a._12 * b._21;
		_12 = a._11 * b._12 + a._12 * b._22;
		_21 = a._21 * b._11 + a._22 * b._21;
		_22 = a._21 * b._12 + a._22 * b._22;
		_31 = a._31 * b._11 + a._32 * b._21 + b._31;
		_32 = a._31 * b._12 + a._32 * b._22 + b._32;
	}

	DMat3x2 operator*(const DMat3x2 &matrix) const
	{
		DMat3x2 result;
		result.setProduct(*this, matrix);
		return result;
	}

	constexpr Vec2 transform(const Point& pos) const
	{
		return
		{
			pos.x * _11 + pos.y * _21 + _31,
			pos.x * _12 + pos.y * _22 + _32
		};
	}

	constexpr Vec2 transform(const Vec2& pos) const
	{
		return
		{
			pos.x * _11 + pos.y * _21 + _31,
			pos.x * _12 + pos.y * _22 + _32
		};
	}
};

template <class CharType>
inline std::basic_ostream<CharType>& operator << (std::basic_ostream<CharType>& os, const DMat3x2& mat)
{
	return os << CharType('(')
		<< Vec2(mat._11, mat._12) << CharType(',')
		<< Vec2(mat._21, mat._22) << CharType(',')
		<< Vec2(mat._31, mat._32) << CharType(')');
}

template <class CharType>
inline std::basic_istream<CharType>& operator >> (std::basic_istream<CharType>& is, DMat3x2& mat)
{
	CharType unused;
	Vec2 r1, r2, r3;

	is >> unused
		>> r1 >> unused
		>> r2 >> unused
		>> r3 >> unused;

	mat._11 = r1.x;
	mat._12 = r1.y;
	mat._21 = r2.x;
	mat._22 = r2.y;
	mat._31 = r3.x;
	mat._32 = r3.y;

	return is;
}

namespace s3d
{
	inline void Formatter(FormatData& formatData, const DMat3x2& value)
	{
		formatData.string.push_back(L'(');

		formatData.string.push_back(L'(');
		formatData.string.append(ToString(value._11, formatData.decimalPlace.value));
		formatData.string.push_back(L',');
		formatData.string.append(ToString(value._12, formatData.decimalPlace.value));
		formatData.string.push_back(L')');

		formatData.string.push_back(L',');

		formatData.string.push_back(L'(');
		formatData.string.append(ToString(value._21, formatData.decimalPlace.value));
		formatData.string.push_back(L',');
		formatData.string.append(ToString(value._22, formatData.decimalPlace.value));
		formatData.string.push_back(L')');

		formatData.string.push_back(L',');

		formatData.string.push_back(L'(');
		formatData.string.append(ToString(value._31, formatData.decimalPlace.value));
		formatData.string.push_back(L',');
		formatData.string.append(ToString(value._32, formatData.decimalPlace.value));
		formatData.string.push_back(L')');

		formatData.string.push_back(L')');
	}
}