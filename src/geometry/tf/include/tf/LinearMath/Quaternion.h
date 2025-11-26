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



#ifndef TF_QUATERNION_H_ // 防止头文件被多次包含
#define TF_QUATERNION_H_


#include "Vector3.h" // 包含 Vector3 类定义
#include "QuadWord.h" // 包含 QuadWord 类定义

#include <ros/macros.h> // 包含 ROS 宏定义，例如 ROS_DEPRECATED

namespace tf // 定义 tf 命名空间
{

/**@brief Quaternion 类实现了四元数，结合 Matrix3x3, Vector3 和 Transform 进行线性代数旋转。 */
class Quaternion : public QuadWord { // Quaternion 类继承自 QuadWord
public:
  /**@brief 无初始化构造函数 */
	Quaternion() {} // 默认构造函数


	//		template <typename tfScalar>
	//		explicit Quaternion(const tfScalar *v) : Tuple4<tfScalar>(v) {}
  /**@brief 从标量构造函数 */
	Quaternion(const tfScalar& x, const tfScalar& y, const tfScalar& z, const tfScalar& w) 
		: QuadWord(x, y, z, w) // 调用基类 QuadWord 的构造函数
	{}
  /**@brief 轴角构造函数
   * @param axis 旋转轴
   * @param angle 绕轴旋转的幅度（弧度） */
	Quaternion(const Vector3& axis, const tfScalar& angle) 
	{
		setRotation(axis, angle); // 设置旋转
	}
  /**@brief 从欧拉角构造函数
   * @param yaw 绕 Y 轴的角，除非定义了 TF_EULER_DEFAULT_ZYX 则为绕 Z 轴
   * @param pitch 绕 X 轴的角，除非定义了 TF_EULER_DEFAULT_ZYX 则为绕 Y 轴
   * @param roll 绕 Z 轴的角，除非定义了 TF_EULER_DEFAULT_ZYX 则为绕 X 轴 */
  ROS_DEPRECATED Quaternion(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll) // 标记为已弃用
	{
#ifndef TF_EULER_DEFAULT_ZYX // 如果未定义 TF_EULER_DEFAULT_ZYX
		setEuler(yaw, pitch, roll); // 使用默认的欧拉角顺序设置
#else
		setRPY(roll, pitch, yaw); // 使用 RPY 顺序设置
#endif 
	}
  /**@brief 使用轴角表示法设置旋转 
   * @param axis 旋转轴
   * @param angle 旋转幅度（弧度） */
	void setRotation(const Vector3& axis, const tfScalar& angle)
	{
		tfScalar d = axis.length(); // 计算轴的长度
		tfAssert(d != tfScalar(0.0)); // 断言轴的长度不为 0
		tfScalar s = tfSin(angle * tfScalar(0.5)) / d; // 计算 sin(angle/2) / 长度
		setValue(axis.x() * s, axis.y() * s, axis.z() * s, // 设置四元数的 x, y, z 分量
			tfCos(angle * tfScalar(0.5))); // 设置四元数的 w 分量
	}
  /**@brief 使用欧拉角设置四元数
   * @param yaw 绕 Y 轴的角
   * @param pitch 绕 X 轴的角
   * @param roll 绕 Z 轴的角 */
	void setEuler(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll)
	{
		tfScalar halfYaw = tfScalar(yaw) * tfScalar(0.5);   // 计算半个 yaw 角
		tfScalar halfPitch = tfScalar(pitch) * tfScalar(0.5);   // 计算半个 pitch 角
		tfScalar halfRoll = tfScalar(roll) * tfScalar(0.5);   // 计算半个 roll 角
		tfScalar cosYaw = tfCos(halfYaw); // 计算半个 yaw 角的余弦
		tfScalar sinYaw = tfSin(halfYaw); // 计算半个 yaw 角的正弦
		tfScalar cosPitch = tfCos(halfPitch); // 计算半个 pitch 角的余弦
		tfScalar sinPitch = tfSin(halfPitch); // 计算半个 pitch 角的正弦
		tfScalar cosRoll = tfCos(halfRoll); // 计算半个 roll 角的余弦
		tfScalar sinRoll = tfSin(halfRoll); // 计算半个 roll 角的正弦
		setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, // 设置四元数的 x 分量
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, // 设置四元数的 y 分量
			sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, // 设置四元数的 z 分量
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); // 设置四元数的 w 分量
	}
  /**@brief 使用固定轴 RPY 设置四元数
   * @param roll 绕 X 轴的角 
   * @param pitch 绕 Y 轴的角
   * @param yaw 绕 Z 轴的角*/
  void setRPY(const tfScalar& roll, const tfScalar& pitch, const tfScalar& yaw)
	{
		tfScalar halfYaw = tfScalar(yaw) * tfScalar(0.5);   // 计算半个 yaw 角
		tfScalar halfPitch = tfScalar(pitch) * tfScalar(0.5);   // 计算半个 pitch 角
		tfScalar halfRoll = tfScalar(roll) * tfScalar(0.5);   // 计算半个 roll 角
		tfScalar cosYaw = tfCos(halfYaw); // 计算半个 yaw 角的余弦
		tfScalar sinYaw = tfSin(halfYaw); // 计算半个 yaw 角的正弦
		tfScalar cosPitch = tfCos(halfPitch); // 计算半个 pitch 角的余弦
		tfScalar sinPitch = tfSin(halfPitch); // 计算半个 pitch 角的正弦
		tfScalar cosRoll = tfCos(halfRoll); // 计算半个 roll 角的余弦
		tfScalar sinRoll = tfSin(halfRoll); // 计算半个 roll 角的正弦
		setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x 分量
                         cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y 分量
                         cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z 分量
                         cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //w 分量 (以前是 yzx)
	}
  /**@brief 使用欧拉角设置四元数 
   * @param yaw 绕 Z 轴的角
   * @param pitch 绕 Y 轴的角
   * @param roll 绕 X 轴的角 */
  ROS_DEPRECATED void setEulerZYX(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll) // 标记为已弃用
	{
          setRPY(roll, pitch, yaw); // 调用 setRPY 函数
	}
  /**@brief 添加两个四元数
   * @param q 要添加到当前四元数的四元数 */
	TFSIMD_FORCE_INLINE	Quaternion& operator+=(const Quaternion& q)
	{
		m_floats[0] += q.x(); m_floats[1] += q.y(); m_floats[2] += q.z(); m_floats[3] += q.m_floats[3]; // 分量相加
		return *this;
	}

  /**@brief 减去一个四元数
   * @param q 要从当前四元数中减去的四元数 */
	Quaternion& operator-=(const Quaternion& q) 
	{
		m_floats[0] -= q.x(); m_floats[1] -= q.y(); m_floats[2] -= q.z(); m_floats[3] -= q.m_floats[3]; // 分量相减
		return *this;
	}

  /**@brief 缩放此四元数
   * @param s 缩放因子 */
	Quaternion& operator*=(const tfScalar& s)
	{
		m_floats[0] *= s; m_floats[1] *= s; m_floats[2] *= s; m_floats[3] *= s; // 分量乘以缩放因子
		return *this;
	}

  /**@brief 将此四元数乘以右侧的 q
   * @param q 另一个四元数 
   * 等同于 this = this * q */
	Quaternion& operator*=(const Quaternion& q)
	{
		setValue(m_floats[3] * q.x() + m_floats[0] * q.m_floats[3] + m_floats[1] * q.z() - m_floats[2] * q.y(), // 计算新的 x 分量
			m_floats[3] * q.y() + m_floats[1] * q.m_floats[3] + m_floats[2] * q.x() - m_floats[0] * q.z(), // 计算新的 y 分量
			m_floats[3] * q.z() + m_floats[2] * q.m_floats[3] + m_floats[0] * q.y() - m_floats[1] * q.x(), // 计算新的 z 分量
			m_floats[3] * q.m_floats[3] - m_floats[0] * q.x() - m_floats[1] * q.y() - m_floats[2] * q.z()); // 计算新的 w 分量
		return *this;
	}
  /**@brief 返回此四元数与另一个四元数的点积
   * @param q 另一个四元数 */
	tfScalar dot(const Quaternion& q) const
	{
		return m_floats[0] * q.x() + m_floats[1] * q.y() + m_floats[2] * q.z() + m_floats[3] * q.m_floats[3]; // 计算点积
	}

  /**@brief 返回四元数长度的平方 */
	tfScalar length2() const
	{
		return dot(*this); // 四元数与自身的点积即为长度的平方
	}

  /**@brief 返回四元数的长度 */
	tfScalar length() const
	{
		return tfSqrt(length2()); // 长度的平方的平方根即为长度
	}

  /**@brief 归一化四元数 
   * 使得 x^2 + y^2 + z^2 +w^2 = 1 */
	Quaternion& normalize() 
	{
		return *this /= length(); // 将四元数除以其长度进行归一化
	}

  /**@brief 返回此四元数的缩放版本
   * @param s 缩放因子 */
	TFSIMD_FORCE_INLINE Quaternion
	operator*(const tfScalar& s) const
	{
		return Quaternion(x() * s, y() * s, z() * s, m_floats[3] * s); // 返回一个新的缩放后的四元数
	}


  /**@brief 返回此四元数的反向缩放版本
   * @param s 反向缩放因子 */
	Quaternion operator/(const tfScalar& s) const
	{
		tfAssert(s != tfScalar(0.0)); // 断言缩放因子不为 0
		return *this * (tfScalar(1.0) / s); // 等效于乘以其倒数
	}

  /**@brief 反向缩放此四元数
   * @param s 缩放因子 */
	Quaternion& operator/=(const tfScalar& s) 
	{
		tfAssert(s != tfScalar(0.0));
		return *this *= tfScalar(1.0) / s;
	}

  /**@brief Return a normalized version of this quaternion */
	Quaternion normalized() const 
	{
		return *this / length();
	} 
  /**@brief Return the ***half*** angle between this quaternion and the other 
   * @param q The other quaternion */
	tfScalar angle(const Quaternion& q) const 
	{
		tfScalar s = tfSqrt(length2() * q.length2());
		tfAssert(s != tfScalar(0.0));
		return tfAcos(dot(q) / s);
	}
	/**@brief Return the angle between this quaternion and the other along the shortest path
	* @param q The other quaternion */
	tfScalar angleShortestPath(const Quaternion& q) const 
	{
		tfScalar s = tfSqrt(length2() * q.length2());
		tfAssert(s != tfScalar(0.0));
		if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
			return tfAcos(dot(-q) / s) * tfScalar(2.0);
		else 
			return tfAcos(dot(q) / s) * tfScalar(2.0);
	}
  	/**@brief Return the angle [0, 2Pi] of rotation represented by this quaternion */
	tfScalar getAngle() const 
	{
		tfScalar s = tfScalar(2.) * tfAcos(m_floats[3]);
		return s;
	}

	/**@brief Return the angle [0, Pi] of rotation represented by this quaternion along the shortest path*/
	tfScalar getAngleShortestPath() const 
	{
		tfScalar s;
		if (m_floats[3] < 0)
		    s = tfScalar(2.) * tfAcos(-m_floats[3]);
		else
		    s = tfScalar(2.) * tfAcos(m_floats[3]);
		return s;
	}

	/**@brief Return the axis of the rotation represented by this quaternion */
	Vector3 getAxis() const
	{
		tfScalar s_squared = tfScalar(1.) - tfPow(m_floats[3], tfScalar(2.));
		if (s_squared < tfScalar(10.) * TFSIMD_EPSILON) //Check for divide by zero
			return Vector3(1.0, 0.0, 0.0);  // Arbitrary
		tfScalar s = tfSqrt(s_squared);
		return Vector3(m_floats[0] / s, m_floats[1] / s, m_floats[2] / s);
	}

	/**@brief Return the inverse of this quaternion */
	Quaternion inverse() const
	{
		return Quaternion(-m_floats[0], -m_floats[1], -m_floats[2], m_floats[3]);
	}

  /**@brief Return the sum of this quaternion and the other 
   * @param q2 The other quaternion */
	TFSIMD_FORCE_INLINE Quaternion
	operator+(const Quaternion& q2) const
	{
		const Quaternion& q1 = *this;
		return Quaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(), q1.m_floats[3] + q2.m_floats[3]);
	}

  /**@brief Return the difference between this quaternion and the other 
   * @param q2 The other quaternion */
	TFSIMD_FORCE_INLINE Quaternion
	operator-(const Quaternion& q2) const
	{
		const Quaternion& q1 = *this;
		return Quaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(), q1.m_floats[3] - q2.m_floats[3]);
	}

  /**@brief Return the negative of this quaternion 
   * This simply negates each element */
	TFSIMD_FORCE_INLINE Quaternion operator-() const
	{
		const Quaternion& q2 = *this;
		return Quaternion( - q2.x(), - q2.y(),  - q2.z(),  - q2.m_floats[3]);
	}
  /**@todo document this and it's use */
	TFSIMD_FORCE_INLINE Quaternion farthest( const Quaternion& qd) const 
	{
		Quaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) > sum.dot(sum) )
			return qd;
		return (-qd);
	}

	/**@todo document this and it's use */
	TFSIMD_FORCE_INLINE Quaternion nearest( const Quaternion& qd) const 
	{
		Quaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) < sum.dot(sum) )
			return qd;
		return (-qd);
	}


  /**@brief Return the quaternion which is the result of Spherical Linear Interpolation between this and the other quaternion
   * @param q The other quaternion to interpolate with 
   * @param t The ratio between this and q to interpolate.  If t = 0 the result is this, if t=1 the result is q.
   * Slerp interpolates assuming constant velocity.  */
	Quaternion slerp(const Quaternion& q, const tfScalar& t) const
	{
          tfScalar theta = angleShortestPath(q) / tfScalar(2.0);
		if (theta != tfScalar(0.0))
		{
			tfScalar d = tfScalar(1.0) / tfSin(theta);
			tfScalar s0 = tfSin((tfScalar(1.0) - t) * theta);
			tfScalar s1 = tfSin(t * theta);   
                        if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
                          return Quaternion((m_floats[0] * s0 + -q.x() * s1) * d,
                                              (m_floats[1] * s0 + -q.y() * s1) * d,
                                              (m_floats[2] * s0 + -q.z() * s1) * d,
                                              (m_floats[3] * s0 + -q.m_floats[3] * s1) * d);
                        else
                          return Quaternion((m_floats[0] * s0 + q.x() * s1) * d,
                                              (m_floats[1] * s0 + q.y() * s1) * d,
                                              (m_floats[2] * s0 + q.z() * s1) * d,
                                              (m_floats[3] * s0 + q.m_floats[3] * s1) * d);
                        
		}
		else
		{
			return *this;
		}
	}

	static const Quaternion&	getIdentity()
	{
		static const Quaternion identityQuat(tfScalar(0.),tfScalar(0.),tfScalar(0.),tfScalar(1.));
		return identityQuat;
	}

	TFSIMD_FORCE_INLINE const tfScalar& getW() const { return m_floats[3]; }

	
};


/**@brief Return the negative of a quaternion */
TFSIMD_FORCE_INLINE Quaternion
operator-(const Quaternion& q)
{
	return Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
}



/**@brief Return the product of two quaternions */
TFSIMD_FORCE_INLINE Quaternion
operator*(const Quaternion& q1, const Quaternion& q2) {
	return Quaternion(q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
		q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
		q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
		q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z()); 
}

TFSIMD_FORCE_INLINE Quaternion
operator*(const Quaternion& q, const Vector3& w)
{
	return Quaternion( q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
		q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
		q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
		-q.x() * w.x() - q.y() * w.y() - q.z() * w.z()); 
}

TFSIMD_FORCE_INLINE Quaternion
operator*(const Vector3& w, const Quaternion& q)
{
	return Quaternion( w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
		w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
		w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
		-w.x() * q.x() - w.y() * q.y() - w.z() * q.z()); 
}

/**@brief Calculate the dot product between two quaternions */
TFSIMD_FORCE_INLINE tfScalar 
dot(const Quaternion& q1, const Quaternion& q2) 
{ 
	return q1.dot(q2); 
}


/**@brief Return the length of a quaternion */
TFSIMD_FORCE_INLINE tfScalar
length(const Quaternion& q) 
{ 
	return q.length(); 
}

/**@brief Return the ***half*** angle between two quaternions*/
TFSIMD_FORCE_INLINE tfScalar
angle(const Quaternion& q1, const Quaternion& q2) 
{ 
	return q1.angle(q2); 
}

/**@brief Return the shortest angle between two quaternions*/
TFSIMD_FORCE_INLINE tfScalar
angleShortestPath(const Quaternion& q1, const Quaternion& q2) 
{ 
	return q1.angleShortestPath(q2); 
}

/**@brief Return the inverse of a quaternion*/
TFSIMD_FORCE_INLINE Quaternion
inverse(const Quaternion& q) 
{
	return q.inverse();
}

/**@brief Return the result of spherical linear interpolation betwen two quaternions 
 * @param q1 The first quaternion
 * @param q2 The second quaternion 
 * @param t The ration between q1 and q2.  t = 0 return q1, t=1 returns q2 
 * Slerp assumes constant velocity between positions. */
TFSIMD_FORCE_INLINE Quaternion
slerp(const Quaternion& q1, const Quaternion& q2, const tfScalar& t) 
{
	return q1.slerp(q2, t);
}

TFSIMD_FORCE_INLINE Vector3 
quatRotate(const Quaternion& rotation, const Vector3& v) 
{
	Quaternion q = rotation * v;
	q *= rotation.inverse();
	return Vector3(q.getX(),q.getY(),q.getZ());
}

TFSIMD_FORCE_INLINE Quaternion 
shortestArcQuat(const Vector3& v0, const Vector3& v1) // Game Programming Gems 2.10. make sure v0,v1 are normalized
{
	Vector3 c = v0.cross(v1);
	tfScalar  d = v0.dot(v1);

	if (d < -1.0 + TFSIMD_EPSILON)
	{
		Vector3 n,unused;
		tfPlaneSpace1(v0,n,unused);
		return Quaternion(n.x(),n.y(),n.z(),0.0f); // just pick any vector that is orthogonal to v0
	}

	tfScalar  s = tfSqrt((1.0f + d) * 2.0f);
	tfScalar rs = 1.0f / s;

	return Quaternion(c.getX()*rs,c.getY()*rs,c.getZ()*rs,s * 0.5f);
}

TFSIMD_FORCE_INLINE Quaternion 
shortestArcQuatNormalize2(Vector3& v0,Vector3& v1)
{
	v0.normalize();
	v1.normalize();
	return shortestArcQuat(v0,v1);
}

}
#endif




