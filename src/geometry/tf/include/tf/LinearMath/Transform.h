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



#ifndef tfTransform_H // 防止头文件被多次包含
#define tfTransform_H


#include "Matrix3x3.h" // 包含 Matrix3x3 类的定义

namespace tf // 定义 tf 命名空间
{

#define TransformData TransformDoubleData // 定义 TransformData 宏，通常用于数据序列化或兼容性


/**@brief Transform 类支持仅包含平移和旋转，不包含缩放/剪切的刚体变换。
 *它可以与 Vector3、Quaternion 和 Matrix3x3 线性代数类结合使用。 */
class Transform {
	
  /// 旋转的存储
	Matrix3x3 m_basis;
  /// 平移的存储
	Vector3   m_origin;

public:
	
  /**@brief 无初始化构造函数 */
	Transform() {}
  /**@brief 从四元数（可选 Vector3）构造函数
   * @param q 四元数表示的旋转
   * @param c 向量表示的平移（默认为 0,0,0） */
	explicit TFSIMD_FORCE_INLINE Transform(const Quaternion& q, 
		const Vector3& c = Vector3(tfScalar(0), tfScalar(0), tfScalar(0))) 
		: m_basis(q), // 使用四元数初始化旋转矩阵
		m_origin(c) // 初始化平移向量
	{}

  /**@brief 从 Matrix3x3（可选 Vector3）构造函数
   * @param b 矩阵表示的旋转
   * @param c 向量表示的平移（默认为 0,0,0）*/
	explicit TFSIMD_FORCE_INLINE Transform(const Matrix3x3& b, 
		const Vector3& c = Vector3(tfScalar(0), tfScalar(0), tfScalar(0))) // 默认平移为零向量
		: m_basis(b), // 初始化旋转矩阵
		m_origin(c) // 初始化平移向量
	{}
  /**@brief 拷贝构造函数 */
	TFSIMD_FORCE_INLINE Transform (const Transform& other)
		: m_basis(other.m_basis), // 拷贝旋转矩阵
		m_origin(other.m_origin) // 拷贝平移向量
	{
	}
  /**@brief 赋值运算符 */
	TFSIMD_FORCE_INLINE Transform& operator=(const Transform& other)
	{
		m_basis = other.m_basis; // 赋值旋转矩阵
		m_origin = other.m_origin; // 赋值平移向量
		return *this;
	}

  /**@brief 将当前变换设置为两个变换乘积的值
   * @param t1 变换 1
   * @param t2 变换 2
   * This = Transform1 * Transform2 */
		TFSIMD_FORCE_INLINE void mult(const Transform& t1, const Transform& t2) {
			m_basis = t1.m_basis * t2.m_basis; // 旋转矩阵相乘
			m_origin = t1(t2.m_origin); // 平移向量的变换
		}

/*		void multInverseLeft(const Transform& t1, const Transform& t2) {
			Vector3 v = t2.m_origin - t1.m_origin;
			m_basis = tfMultTransposeLeft(t1.m_basis, t2.m_basis);
			m_origin = v * t1.m_basis;
		}
		*/

/**@brief 返回向量的变换结果 */
	TFSIMD_FORCE_INLINE Vector3 operator()(const Vector3& x) const
	{
		return Vector3(m_basis[0].dot(x) + m_origin.x(), 
			m_basis[1].dot(x) + m_origin.y(), 
			m_basis[2].dot(x) + m_origin.z());
	}

  /**@brief 返回向量的变换结果 (等同于 operator()) */
	TFSIMD_FORCE_INLINE Vector3 operator*(const Vector3& x) const
	{
		return (*this)(x);
	}

  /**@brief 返回四元数的变换结果 */
	TFSIMD_FORCE_INLINE Quaternion operator*(const Quaternion& q) const
	{
		return getRotation() * q; // 旋转部分与四元数相乘
	}

  /**@brief 返回旋转的基矩阵 */
	TFSIMD_FORCE_INLINE Matrix3x3&       getBasis()          { return m_basis; } // 获取旋转矩阵的非常量引用
  /**@brief 返回旋转的基矩阵 */
	TFSIMD_FORCE_INLINE const Matrix3x3& getBasis()    const { return m_basis; } // 获取旋转矩阵的常量引用

  /**@brief 返回原点向量（平移部分） */
	TFSIMD_FORCE_INLINE Vector3&         getOrigin()         { return m_origin; } // 获取平移向量的非常量引用
  /**@brief 返回原点向量（平移部分） */
	TFSIMD_FORCE_INLINE const Vector3&   getOrigin()   const { return m_origin; } // 获取平移向量的常量引用

  /**@brief 返回表示旋转的四元数 */
	Quaternion getRotation() const { 
		Quaternion q;
		m_basis.getRotation(q); // 从旋转矩阵获取四元数
		return q;
	}
	
	
  /**@brief 从数组设置变换
   * @param m 指向一个 15 元素数组的指针（12 个旋转元素（行主序，右侧填充 1），3 个平移元素） */
	void setFromOpenGLMatrix(const tfScalar *m)
	{
		m_basis.setFromOpenGLSubMatrix(m); // 从 OpenGL 矩阵设置旋转矩阵
		m_origin.setValue(m[12],m[13],m[14]); // 设置平移向量
	}

  /**@brief 填充数组表示
   * @param m 指向一个 15 元素数组的指针（12 个旋转元素（行主序，右侧填充 1），3 个平移元素） */
	void getOpenGLMatrix(tfScalar *m) const 
	{
		m_basis.getOpenGLSubMatrix(m); // 获取旋转矩阵的 OpenGL 表示
		m[12] = m_origin.x(); // 设置平移 x 分量
		m[13] = m_origin.y(); // 设置平移 y 分量
		m[14] = m_origin.z(); // 设置平移 z 分量
		m[15] = tfScalar(1.0); // 设置齐次坐标的 w 分量
	}

  /**@brief 设置平移元素
   * @param origin 要设置的平移向量 */
	TFSIMD_FORCE_INLINE void setOrigin(const Vector3& origin) 
	{
		m_origin = origin;
	}

	TFSIMD_FORCE_INLINE Vector3 invXform(const Vector3& inVec) const; // 声明逆变换函数


  /**@brief 通过 Matrix3x3 设置旋转元素 */
	TFSIMD_FORCE_INLINE void setBasis(const Matrix3x3& basis)
	{
		m_basis = basis;
	}

  /**@brief 通过 Quaternion 设置旋转元素 */
	TFSIMD_FORCE_INLINE void setRotation(const Quaternion& q)
	{
		m_basis.setRotation(q);
	}


  /**@brief 将此变换设置为单位变换 */
	void setIdentity()
	{
		m_basis.setIdentity(); // 设置旋转矩阵为单位矩阵
		m_origin.setValue(tfScalar(0.0), tfScalar(0.0), tfScalar(0.0)); // 设置平移向量为零向量
	}

  /**@brief 将此 Transform 与另一个相乘 (this = this * another) 
   * @param t 另一个变换 */
	Transform& operator*=(const Transform& t) 
	{
		m_origin += m_basis * t.m_origin; // 更新平移部分
		m_basis *= t.m_basis; // 更新旋转部分
		return *this;
	}

  /**@brief 返回此变换的逆变换 */
	Transform inverse() const
	{
		Matrix3x3 inv = m_basis.transpose(); // 计算旋转矩阵的转置（对于正交矩阵即为逆）
		return Transform(inv, inv * -m_origin); // 返回逆变换
	}

  /**@brief 返回此变换的逆变换乘以另一个变换的结果
   * @param t 另一个变换 
   * 返回 this.inverse() * other */
	Transform inverseTimes(const Transform& t) const;  

  /**@brief 返回此变换与另一个变换的乘积 */
	Transform operator*(const Transform& t) const;

  /**@brief Return an identity transform */
	static const Transform&	getIdentity()
	{
		static const Transform identityTransform(Matrix3x3::getIdentity());
		return identityTransform;
	}

	void	serialize(struct	TransformData& dataOut) const;

	void	serializeFloat(struct	TransformFloatData& dataOut) const;

	void	deSerialize(const struct	TransformData& dataIn);

	void	deSerializeDouble(const struct	TransformDoubleData& dataIn);

	void	deSerializeFloat(const struct	TransformFloatData& dataIn);

};


TFSIMD_FORCE_INLINE Vector3
Transform::invXform(const Vector3& inVec) const
{
	Vector3 v = inVec - m_origin;
	return (m_basis.transpose() * v);
}

TFSIMD_FORCE_INLINE Transform 
Transform::inverseTimes(const Transform& t) const  
{
	Vector3 v = t.getOrigin() - m_origin;
		return Transform(m_basis.transposeTimes(t.m_basis),
			v * m_basis);
}

TFSIMD_FORCE_INLINE Transform 
Transform::operator*(const Transform& t) const
{
	return Transform(m_basis * t.m_basis, 
		(*this)(t.m_origin));
}

/**@brief Test if two transforms have all elements equal */
TFSIMD_FORCE_INLINE bool operator==(const Transform& t1, const Transform& t2)
{
   return ( t1.getBasis()  == t2.getBasis() &&
            t1.getOrigin() == t2.getOrigin() );
}


///for serialization
struct	TransformFloatData
{
	Matrix3x3FloatData	m_basis;
	Vector3FloatData	m_origin;
};

struct	TransformDoubleData
{
	Matrix3x3DoubleData	m_basis;
	Vector3DoubleData	m_origin;
};



TFSIMD_FORCE_INLINE	void	Transform::serialize(TransformData& dataOut) const
{
	m_basis.serialize(dataOut.m_basis);
	m_origin.serialize(dataOut.m_origin);
}

TFSIMD_FORCE_INLINE	void	Transform::serializeFloat(TransformFloatData& dataOut) const
{
	m_basis.serializeFloat(dataOut.m_basis);
	m_origin.serializeFloat(dataOut.m_origin);
}


TFSIMD_FORCE_INLINE	void	Transform::deSerialize(const TransformData& dataIn)
{
	m_basis.deSerialize(dataIn.m_basis);
	m_origin.deSerialize(dataIn.m_origin);
}

TFSIMD_FORCE_INLINE	void	Transform::deSerializeFloat(const TransformFloatData& dataIn)
{
	m_basis.deSerializeFloat(dataIn.m_basis);
	m_origin.deSerializeFloat(dataIn.m_origin);
}

TFSIMD_FORCE_INLINE	void	Transform::deSerializeDouble(const TransformDoubleData& dataIn)
{
	m_basis.deSerializeDouble(dataIn.m_basis);
	m_origin.deSerializeDouble(dataIn.m_origin);
}

}

#endif






