/*
版权所有 (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

本软件按“原样”提供，不附带任何明示或暗示的保证。
在任何情况下，作者均不对因使用本软件而引起的任何损害承担责任。
允许任何人出于任何目的使用本软件，
包括商业应用，并可以自由修改和重新分发，
但须遵守以下限制：

1. 本软件的来源不得歪曲；您不得声称是您编写了原始软件。如果您在产品中使用本软件，产品文档中的致谢将不胜感激，但不是必需的。
2. 更改的源版本必须明确标记，并且不得歪曲为原始软件。
3. 本通知不得从任何源分发中删除或更改。
*/



#ifndef TF_VECTOR3_H // 防止头文件被多次包含
#define TF_VECTOR3_H


#include "Scalar.h" // 包含 Scalar 类型定义
#include "MinMax.h" // 包含 MinMax 函数定义

namespace tf{ // 定义 tf 命名空间

#define Vector3Data Vector3DoubleData // 定义 Vector3Data 宏，用于数据序列化或兼容性
#define Vector3DataName "Vector3DoubleData" // 定义 Vector3DataName 宏




/**
 * @class Vector3
 * @brief Vector3 可用于表示 3D 点和向量。
 * 它有一个未使用的 w 分量，以适应 Vector3 存储在容器中时的 16 字节对齐。这个额外的分量可以被派生类（Quaternion？）或用户使用。
 * 理想情况下，这个类应该被一个平台优化的 SIMD 版本替换，该版本将数据保存在寄存器中。
 */
ATTRIBUTE_ALIGNED16(class) Vector3 // 确保类按 16 字节对齐
{
public:

#if defined (__SPU__) && defined (__CELLOS_LV2__) // 针对 SPU 和 CELLOS_LV2 平台的条件编译
		tfScalar	m_floats[4]; // 存储向量分量的浮点数组
public:
	TFSIMD_FORCE_INLINE const vec_float4&	get128() const // 获取 128 位向量数据
	{
		return *((const vec_float4*)&m_floats[0]);
	}
public:
#else //__CELLOS_LV2__ __SPU__
#ifdef TF_USE_SSE // _WIN32 // 针对 SSE 优化的条件编译
	union { // 使用联合体实现内存共享
		__m128 mVec128; // SSE 128 位向量类型
		tfScalar	m_floats[4]; // 存储向量分量的浮点数组
	};
	TFSIMD_FORCE_INLINE	__m128	get128() const // 获取 128 位向量数据
	{
		return mVec128;
	}
	TFSIMD_FORCE_INLINE	void	set128(__m128 v128) // 设置 128 位向量数据
	{
		mVec128 = v128;
	}
#else // 非 SSE 优化
	tfScalar	m_floats[4]; // 存储向量分量的浮点数组
#endif
#endif //__CELLOS_LV2__ __SPU__

	public:

  /**@brief 无初始化构造函数 */
	TFSIMD_FORCE_INLINE Vector3() {} // 默认构造函数，不进行初始化


  /**@brief 从标量构造函数 
   * @param x X 值
   * @param y Y 值 
   * @param z Z 值 
   */
	TFSIMD_FORCE_INLINE Vector3(const tfScalar& x, const tfScalar& y, const tfScalar& z)
	{
		m_floats[0] = x; // 设置 X 分量
		m_floats[1] = y; // 设置 Y 分量
		m_floats[2] = z; // 设置 Z 分量
		m_floats[3] = tfScalar(0.); // 设置 W 分量为 0
	}

/**@brief 将一个向量加到当前向量上 
 * @param v 要添加的向量 */
	TFSIMD_FORCE_INLINE Vector3& operator+=(const Vector3& v)
	{

		m_floats[0] += v.m_floats[0]; m_floats[1] += v.m_floats[1];m_floats[2] += v.m_floats[2]; // 分量相加
		return *this;
	}


  /**@brief 从当前向量中减去一个向量
   * @param v 要减去的向量 */
	TFSIMD_FORCE_INLINE Vector3& operator-=(const Vector3& v) 
	{
		m_floats[0] -= v.m_floats[0]; m_floats[1] -= v.m_floats[1];m_floats[2] -= v.m_floats[2]; // 分量相减
		return *this;
	}
  /**@brief 缩放向量
   * @param s 缩放因子 */
	TFSIMD_FORCE_INLINE Vector3& operator*=(const tfScalar& s)
	{
		m_floats[0] *= s; m_floats[1] *= s;m_floats[2] *= s; // 分量乘以缩放因子
		return *this;
	}

  /**@brief 反向缩放向量 
   * @param s 要除以的缩放因子 */
	TFSIMD_FORCE_INLINE Vector3& operator/=(const tfScalar& s) 
	{
		tfFullAssert(s != tfScalar(0.0)); // 断言缩放因子不为 0
		return *this *= tfScalar(1.0) / s; // 等效于乘以其倒数
	}

  /**@brief 返回点积
   * @param v 点积中的另一个向量 */
	TFSIMD_FORCE_INLINE tfScalar dot(const Vector3& v) const
	{
		return m_floats[0] * v.m_floats[0] + m_floats[1] * v.m_floats[1] +m_floats[2] * v.m_floats[2]; // 计算点积
	}

  /**@brief 返回向量长度的平方 */
	TFSIMD_FORCE_INLINE tfScalar length2() const
	{
		return dot(*this); // 向量与自身的点积即为长度的平方
	}

  /**@brief 返回向量的长度 */
	TFSIMD_FORCE_INLINE tfScalar length() const
	{
		return tfSqrt(length2()); // 长度的平方的平方根即为长度
	}

  /**@brief 返回此向量和另一个向量末端之间的距离平方
   * 这在语义上将向量视为一个点 */
	TFSIMD_FORCE_INLINE tfScalar distance2(const Vector3& v) const; // 声明计算距离平方的函数

  /**@brief 返回此向量和另一个向量末端之间的距离
   * 这在语义上将向量视为一个点 */
	TFSIMD_FORCE_INLINE tfScalar distance(const Vector3& v) const; // 声明计算距离的函数

  /**@brief 归一化此向量 
   * x^2 + y^2 + z^2 = 1 */
	TFSIMD_FORCE_INLINE Vector3& normalize() 
	{
		return *this /= length(); // 将向量除以其长度进行归一化
	}

  /**@brief 返回此向量的归一化版本 */	
	TFSIMD_FORCE_INLINE Vector3 normalized() const; // 声明返回归一化向量的函数

  /**@brief 旋转此向量 
   * @param wAxis 旋转轴 
   * @param angle 旋转角度 */
	TFSIMD_FORCE_INLINE Vector3 rotate( const Vector3& wAxis, const tfScalar angle ) const; // 声明旋转向量的函数

  /**@brief 返回此向量与另一个向量之间的夹角
   * @param v 另一个向量 */
	TFSIMD_FORCE_INLINE tfScalar angle(const Vector3& v) const 
	{
		tfScalar s = tfSqrt(length2() * v.length2()); // 计算长度乘积
		tfFullAssert(s != tfScalar(0.0)); // 断言长度乘积不为 0
		return tfAcos(dot(v) / s); // 使用点积和长度计算夹角
	}
  /**@brief 返回一个向量，其每个元素都是绝对值 */
	TFSIMD_FORCE_INLINE Vector3 absolute() const 
	{
		return Vector3(
			tfFabs(m_floats[0]), // X 分量的绝对值
			tfFabs(m_floats[1]), // Y 分量的绝对值
			tfFabs(m_floats[2])); // Z 分量的绝对值
	}
  /**@brief 返回此向量与另一个向量的叉积 
   * @param v 另一个向量 */
	TFSIMD_FORCE_INLINE Vector3 cross(const Vector3& v) const
	{
		return Vector3(
			m_floats[1] * v.m_floats[2] -m_floats[2] * v.m_floats[1], // 计算叉积的 X 分量
			m_floats[2] * v.m_floats[0] - m_floats[0] * v.m_floats[2], // 计算叉积的 Y 分量
			m_floats[0] * v.m_floats[1] - m_floats[1] * v.m_floats[0]); // 计算叉积的 Z 分量
	}

	TFSIMD_FORCE_INLINE tfScalar triple(const Vector3& v1, const Vector3& v2) const // 声明三重积函数
	{
		return m_floats[0] * (v1.m_floats[1] * v2.m_floats[2] - v1.m_floats[2] * v2.m_floats[1]) + 
			m_floats[1] * (v1.m_floats[2] * v2.m_floats[0] - v1.m_floats[0] * v2.m_floats[2]) + 
			m_floats[2] * (v1.m_floats[0] * v2.m_floats[1] - v1.m_floats[1] * v2.m_floats[0]);
	}

  /**@brief 返回值最小的轴 
   * 注意返回值 0,1,2 分别代表 x, y, 或 z */
	TFSIMD_FORCE_INLINE int minAxis() const
	{
		return m_floats[0] < m_floats[1] ? (m_floats[0] <m_floats[2] ? 0 : 2) : (m_floats[1] <m_floats[2] ? 1 : 2); // 比较分量大小，返回最小值的轴索引
	}

  /**@brief Return the axis with the largest value 
   * Note return values are 0,1,2 for x, y, or z */
	TFSIMD_FORCE_INLINE int maxAxis() const 
	{
		return m_floats[0] < m_floats[1] ? (m_floats[1] <m_floats[2] ? 2 : 1) : (m_floats[0] <m_floats[2] ? 2 : 0);
	}

	TFSIMD_FORCE_INLINE int furthestAxis() const
	{
		return absolute().minAxis();
	}

	TFSIMD_FORCE_INLINE int closestAxis() const 
	{
		return absolute().maxAxis();
	}

	TFSIMD_FORCE_INLINE void setInterpolate3(const Vector3& v0, const Vector3& v1, tfScalar rt)
	{
		tfScalar s = tfScalar(1.0) - rt;
		m_floats[0] = s * v0.m_floats[0] + rt * v1.m_floats[0];
		m_floats[1] = s * v0.m_floats[1] + rt * v1.m_floats[1];
		m_floats[2] = s * v0.m_floats[2] + rt * v1.m_floats[2];
		//don't do the unused w component
		//		m_co[3] = s * v0[3] + rt * v1[3];
	}

  /**@brief Return the linear interpolation between this and another vector 
   * @param v The other vector 
   * @param t The ration of this to v (t = 0 => return this, t=1 => return other) */
	TFSIMD_FORCE_INLINE Vector3 lerp(const Vector3& v, const tfScalar& t) const 
	{
		return Vector3(m_floats[0] + (v.m_floats[0] - m_floats[0]) * t,
			m_floats[1] + (v.m_floats[1] - m_floats[1]) * t,
			m_floats[2] + (v.m_floats[2] -m_floats[2]) * t);
	}

  /**@brief Elementwise multiply this vector by the other 
   * @param v The other vector */
	TFSIMD_FORCE_INLINE Vector3& operator*=(const Vector3& v)
	{
		m_floats[0] *= v.m_floats[0]; m_floats[1] *= v.m_floats[1];m_floats[2] *= v.m_floats[2];
		return *this;
	}

	 /**@brief Return the x value */
		TFSIMD_FORCE_INLINE const tfScalar& getX() const { return m_floats[0]; }
  /**@brief Return the y value */
		TFSIMD_FORCE_INLINE const tfScalar& getY() const { return m_floats[1]; }
  /**@brief Return the z value */
		TFSIMD_FORCE_INLINE const tfScalar& getZ() const { return m_floats[2]; }
  /**@brief Set the x value */
		TFSIMD_FORCE_INLINE void	setX(tfScalar x) { m_floats[0] = x;};
  /**@brief Set the y value */
		TFSIMD_FORCE_INLINE void	setY(tfScalar y) { m_floats[1] = y;};
  /**@brief Set the z value */
		TFSIMD_FORCE_INLINE void	setZ(tfScalar z) {m_floats[2] = z;};
  /**@brief Set the w value */
		TFSIMD_FORCE_INLINE void	setW(tfScalar w) { m_floats[3] = w;};
  /**@brief Return the x value */
		TFSIMD_FORCE_INLINE const tfScalar& x() const { return m_floats[0]; }
  /**@brief Return the y value */
		TFSIMD_FORCE_INLINE const tfScalar& y() const { return m_floats[1]; }
  /**@brief Return the z value */
		TFSIMD_FORCE_INLINE const tfScalar& z() const { return m_floats[2]; }
  /**@brief Return the w value */
		TFSIMD_FORCE_INLINE const tfScalar& w() const { return m_floats[3]; }

	//TFSIMD_FORCE_INLINE tfScalar&       operator[](int i)       { return (&m_floats[0])[i];	}      
	//TFSIMD_FORCE_INLINE const tfScalar& operator[](int i) const { return (&m_floats[0])[i]; }
	///operator tfScalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
	TFSIMD_FORCE_INLINE	operator       tfScalar *()       { return &m_floats[0]; }
	TFSIMD_FORCE_INLINE	operator const tfScalar *() const { return &m_floats[0]; }

	TFSIMD_FORCE_INLINE	bool	operator==(const Vector3& other) const
	{
		return ((m_floats[3]==other.m_floats[3]) && (m_floats[2]==other.m_floats[2]) && (m_floats[1]==other.m_floats[1]) && (m_floats[0]==other.m_floats[0]));
	}

	TFSIMD_FORCE_INLINE	bool	operator!=(const Vector3& other) const
	{
		return !(*this == other);
	}

	 /**@brief Set each element to the max of the current values and the values of another Vector3
   * @param other The other Vector3 to compare with 
   */
		TFSIMD_FORCE_INLINE void	setMax(const Vector3& other)
		{
			tfSetMax(m_floats[0], other.m_floats[0]);
			tfSetMax(m_floats[1], other.m_floats[1]);
			tfSetMax(m_floats[2], other.m_floats[2]);
			tfSetMax(m_floats[3], other.w());
		}
  /**@brief Set each element to the min of the current values and the values of another Vector3
   * @param other The other Vector3 to compare with 
   */
		TFSIMD_FORCE_INLINE void	setMin(const Vector3& other)
		{
			tfSetMin(m_floats[0], other.m_floats[0]);
			tfSetMin(m_floats[1], other.m_floats[1]);
			tfSetMin(m_floats[2], other.m_floats[2]);
			tfSetMin(m_floats[3], other.w());
		}

		TFSIMD_FORCE_INLINE void 	setValue(const tfScalar& x, const tfScalar& y, const tfScalar& z)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3] = tfScalar(0.);
		}

		void	getSkewSymmetricMatrix(Vector3* v0,Vector3* v1,Vector3* v2) const
		{
			v0->setValue(0.		,-z()		,y());
			v1->setValue(z()	,0.			,-x());
			v2->setValue(-y()	,x()	,0.);
		}

		void	setZero()
		{
			setValue(tfScalar(0.),tfScalar(0.),tfScalar(0.));
		}

		TFSIMD_FORCE_INLINE bool isZero() const 
		{
			return m_floats[0] == tfScalar(0) && m_floats[1] == tfScalar(0) && m_floats[2] == tfScalar(0);
		}

		TFSIMD_FORCE_INLINE bool fuzzyZero() const 
		{
			return length2() < TFSIMD_EPSILON;
		}

		TFSIMD_FORCE_INLINE	void	serialize(struct	Vector3Data& dataOut) const;

		TFSIMD_FORCE_INLINE	void	deSerialize(const struct	Vector3Data& dataIn);

		TFSIMD_FORCE_INLINE	void	serializeFloat(struct	Vector3FloatData& dataOut) const;

		TFSIMD_FORCE_INLINE	void	deSerializeFloat(const struct	Vector3FloatData& dataIn);

		TFSIMD_FORCE_INLINE	void	serializeDouble(struct	Vector3DoubleData& dataOut) const;

		TFSIMD_FORCE_INLINE	void	deSerializeDouble(const struct	Vector3DoubleData& dataIn);

};

/**@brief Return the sum of two vectors (Point symantics)*/
TFSIMD_FORCE_INLINE Vector3 
operator+(const Vector3& v1, const Vector3& v2) 
{
	return Vector3(v1.m_floats[0] + v2.m_floats[0], v1.m_floats[1] + v2.m_floats[1], v1.m_floats[2] + v2.m_floats[2]);
}

/**@brief Return the elementwise product of two vectors */
TFSIMD_FORCE_INLINE Vector3 
operator*(const Vector3& v1, const Vector3& v2) 
{
	return Vector3(v1.m_floats[0] * v2.m_floats[0], v1.m_floats[1] * v2.m_floats[1], v1.m_floats[2] * v2.m_floats[2]);
}

/**@brief Return the difference between two vectors */
TFSIMD_FORCE_INLINE Vector3 
operator-(const Vector3& v1, const Vector3& v2)
{
	return Vector3(v1.m_floats[0] - v2.m_floats[0], v1.m_floats[1] - v2.m_floats[1], v1.m_floats[2] - v2.m_floats[2]);
}
/**@brief Return the negative of the vector */
TFSIMD_FORCE_INLINE Vector3 
operator-(const Vector3& v)
{
	return Vector3(-v.m_floats[0], -v.m_floats[1], -v.m_floats[2]);
}

/**@brief Return the vector scaled by s */
TFSIMD_FORCE_INLINE Vector3 
operator*(const Vector3& v, const tfScalar& s)
{
	return Vector3(v.m_floats[0] * s, v.m_floats[1] * s, v.m_floats[2] * s);
}

/**@brief Return the vector scaled by s */
TFSIMD_FORCE_INLINE Vector3 
operator*(const tfScalar& s, const Vector3& v)
{ 
	return v * s; 
}

/**@brief Return the vector inversely scaled by s */
TFSIMD_FORCE_INLINE Vector3
operator/(const Vector3& v, const tfScalar& s)
{
	tfFullAssert(s != tfScalar(0.0));
	return v * (tfScalar(1.0) / s);
}

/**@brief Return the vector inversely scaled by s */
TFSIMD_FORCE_INLINE Vector3
operator/(const Vector3& v1, const Vector3& v2)
{
	return Vector3(v1.m_floats[0] / v2.m_floats[0],v1.m_floats[1] / v2.m_floats[1],v1.m_floats[2] / v2.m_floats[2]);
}

/**@brief Return the dot product between two vectors */
TFSIMD_FORCE_INLINE tfScalar 
tfDot(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.dot(v2); 
}


/**@brief Return the distance squared between two vectors */
TFSIMD_FORCE_INLINE tfScalar
tfDistance2(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.distance2(v2); 
}


/**@brief Return the distance between two vectors */
TFSIMD_FORCE_INLINE tfScalar
tfDistance(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.distance(v2); 
}

/**@brief Return the angle between two vectors */
TFSIMD_FORCE_INLINE tfScalar
tfAngle(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.angle(v2); 
}

/**@brief Return the cross product of two vectors */
TFSIMD_FORCE_INLINE Vector3 
tfCross(const Vector3& v1, const Vector3& v2) 
{ 
	return v1.cross(v2); 
}

TFSIMD_FORCE_INLINE tfScalar
tfTriple(const Vector3& v1, const Vector3& v2, const Vector3& v3)
{
	return v1.triple(v2, v3);
}

/**@brief Return the linear interpolation between two vectors
 * @param v1 One vector 
 * @param v2 The other vector 
 * @param t The ration of this to v (t = 0 => return v1, t=1 => return v2) */
TFSIMD_FORCE_INLINE Vector3 
lerp(const Vector3& v1, const Vector3& v2, const tfScalar& t)
{
	return v1.lerp(v2, t);
}



TFSIMD_FORCE_INLINE tfScalar Vector3::distance2(const Vector3& v) const
{
	return (v - *this).length2();
}

TFSIMD_FORCE_INLINE tfScalar Vector3::distance(const Vector3& v) const
{
	return (v - *this).length();
}

TFSIMD_FORCE_INLINE Vector3 Vector3::normalized() const
{
	return *this / length();
} 

TFSIMD_FORCE_INLINE Vector3 Vector3::rotate( const Vector3& wAxis, const tfScalar angle ) const
{
	// wAxis must be a unit lenght vector

	Vector3 o = wAxis * wAxis.dot( *this );
	Vector3 x = *this - o;
	Vector3 y;

	y = wAxis.cross( *this );

	return ( o + x * tfCos( angle ) + y * tfSin( angle ) );
}

class tfVector4 : public Vector3
{
public:

	TFSIMD_FORCE_INLINE tfVector4() {}


	TFSIMD_FORCE_INLINE tfVector4(const tfScalar& x, const tfScalar& y, const tfScalar& z,const tfScalar& w) 
		: Vector3(x,y,z)
	{
		m_floats[3] = w;
	}


	TFSIMD_FORCE_INLINE tfVector4 absolute4() const 
	{
		return tfVector4(
			tfFabs(m_floats[0]), 
			tfFabs(m_floats[1]), 
			tfFabs(m_floats[2]),
			tfFabs(m_floats[3]));
	}



	tfScalar	getW() const { return m_floats[3];}


		TFSIMD_FORCE_INLINE int maxAxis4() const
	{
		int maxIndex = -1;
		tfScalar maxVal = tfScalar(-TF_LARGE_FLOAT);
		if (m_floats[0] > maxVal)
		{
			maxIndex = 0;
			maxVal = m_floats[0];
		}
		if (m_floats[1] > maxVal)
		{
			maxIndex = 1;
			maxVal = m_floats[1];
		}
		if (m_floats[2] > maxVal)
		{
			maxIndex = 2;
			maxVal =m_floats[2];
		}
		if (m_floats[3] > maxVal)
		{
			maxIndex = 3;
			maxVal = m_floats[3];
		}
		
		
		

		return maxIndex;

	}


	TFSIMD_FORCE_INLINE int minAxis4() const
	{
		int minIndex = -1;
		tfScalar minVal = tfScalar(TF_LARGE_FLOAT);
		if (m_floats[0] < minVal)
		{
			minIndex = 0;
			minVal = m_floats[0];
		}
		if (m_floats[1] < minVal)
		{
			minIndex = 1;
			minVal = m_floats[1];
		}
		if (m_floats[2] < minVal)
		{
			minIndex = 2;
			minVal =m_floats[2];
		}
		if (m_floats[3] < minVal)
		{
			minIndex = 3;
			minVal = m_floats[3];
		}
		
		return minIndex;

	}


	TFSIMD_FORCE_INLINE int closestAxis4() const 
	{
		return absolute4().maxAxis4();
	}

	
 

  /**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
		

/*		void getValue(tfScalar *m) const 
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] =m_floats[2];
		}
*/
/**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
		TFSIMD_FORCE_INLINE void	setValue(const tfScalar& x, const tfScalar& y, const tfScalar& z,const tfScalar& w)
		{
			m_floats[0]=x;
			m_floats[1]=y;
			m_floats[2]=z;
			m_floats[3]=w;
		}


};


///tfSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
TFSIMD_FORCE_INLINE void	tfSwapScalarEndian(const tfScalar& sourceVal, tfScalar& destVal)
{
	unsigned char* dest = (unsigned char*) &destVal;
	const unsigned char* src  = (const unsigned char*) &sourceVal;
	dest[0] = src[7];
    dest[1] = src[6];
    dest[2] = src[5];
    dest[3] = src[4];
    dest[4] = src[3];
    dest[5] = src[2];
    dest[6] = src[1];
    dest[7] = src[0];
}
///tfSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
TFSIMD_FORCE_INLINE void	tfSwapVector3Endian(const Vector3& sourceVec, Vector3& destVec)
{
	for (int i=0;i<4;i++)
	{
		tfSwapScalarEndian(sourceVec[i],destVec[i]);
	}

}

///tfUnSwapVector3Endian swaps vector endianness, useful for network and cross-platform serialization
TFSIMD_FORCE_INLINE void	tfUnSwapVector3Endian(Vector3& vector)
{

	Vector3	swappedVec;
	for (int i=0;i<4;i++)
	{
		tfSwapScalarEndian(vector[i],swappedVec[i]);
	}
	vector = swappedVec;
}

TFSIMD_FORCE_INLINE void tfPlaneSpace1 (const Vector3& n, Vector3& p, Vector3& q)
{
  if (tfFabs(n.z()) > TFSIMDSQRT12) {
    // choose p in y-z plane
    tfScalar a = n[1]*n[1] + n[2]*n[2];
    tfScalar k = tfRecipSqrt (a);
    p.setValue(0,-n[2]*k,n[1]*k);
    // set q = n x p
    q.setValue(a*k,-n[0]*p[2],n[0]*p[1]);
  }
  else {
    // choose p in x-y plane
    tfScalar a = n.x()*n.x() + n.y()*n.y();
    tfScalar k = tfRecipSqrt (a);
    p.setValue(-n.y()*k,n.x()*k,0);
    // set q = n x p
    q.setValue(-n.z()*p.y(),n.z()*p.x(),a*k);
  }
}


struct	Vector3FloatData
{
	float	m_floats[4];
};

struct	Vector3DoubleData
{
	double	m_floats[4];

};

TFSIMD_FORCE_INLINE	void	Vector3::serializeFloat(struct	Vector3FloatData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = float(m_floats[i]);
}

TFSIMD_FORCE_INLINE void	Vector3::deSerializeFloat(const struct	Vector3FloatData& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = tfScalar(dataIn.m_floats[i]);
}


TFSIMD_FORCE_INLINE	void	Vector3::serializeDouble(struct	Vector3DoubleData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = double(m_floats[i]);
}

TFSIMD_FORCE_INLINE void	Vector3::deSerializeDouble(const struct	Vector3DoubleData& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = tfScalar(dataIn.m_floats[i]);
}


TFSIMD_FORCE_INLINE	void	Vector3::serialize(struct	Vector3Data& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i=0;i<4;i++)
		dataOut.m_floats[i] = m_floats[i];
}

TFSIMD_FORCE_INLINE void	Vector3::deSerialize(const struct	Vector3Data& dataIn)
{
	for (int i=0;i<4;i++)
		m_floats[i] = dataIn.m_floats[i];
}

}

#endif //TFSIMD__VECTOR3_H
