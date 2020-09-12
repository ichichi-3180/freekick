/*! @file rx_matrix.h
	
	@brief ベクトル・行列ライブラリ - 行列クラス
 
	@author 
	@date  
*/

#ifndef _MATRIX_H_
#define _MATRIX_H_

//-----------------------------------------------------------------------------
// インクルードファイル
//-----------------------------------------------------------------------------
#include <memory.h>
#include "rx_utility.h"

//-----------------------------------------------------------------------------
// 定義
//-----------------------------------------------------------------------------
#define real double 

#define QUATERNION_NORMALIZATION_THRESHOLD  64

#ifndef RX_INV_PI
#define RX_INV_PI 0.318309886
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG  (real)(57.2957795130823208767981548141052)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD  (real)(0.0174532925199432957692369076848861)
#endif

#ifndef RX_EPSILON
#define RX_EPSILON     (real)(10e-6)
#endif

#ifndef RX_PI
#define RX_PI          (real)(3.1415926535897932384626433832795)    
#endif

#define equivalent(a,b)     (((a < b + RX_EPSILON) && (a > b - RX_EPSILON)) ? true : false)

//-----------------------------------------------------------------------------
//! 4x4行列クラス
//-----------------------------------------------------------------------------
class rxMatrix4
{
protected:
	real m[16];

public:
	//! コンストラクタ
	rxMatrix4(){ MakeIdentity(); }
	rxMatrix4(real r){ SetValue(r); }
	rxMatrix4(real *m){ SetValue(m); }
	rxMatrix4(real a00, real a01, real a02, real a03,
			real a10, real a11, real a12, real a13,
			real a20, real a21, real a22, real a23,
			real a30, real a31, real a32, real a33)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		element(0,3) = a03;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		element(1,3) = a13;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
		element(2,3) = a23;
		
		element(3,0) = a30;
		element(3,1) = a31;
		element(3,2) = a32;
		element(3,3) = a33;
	}

	//! 行列をmpに入れる
	void GetValue(real *mp) const
	{
		int c = 0;
		for(int j = 0; j < 4; j++){
			for(int i = 0; i < 4; i++){
				mp[c++] = element(i,j);
			}
		}
	}

	//! 行列をmpに入れる
	template<typename T> 
	void GetValueT(T *mp) const
	{
		int c = 0;
		for(int j = 0; j < 4; j++){
			for(int i = 0; i < 4; i++){
				mp[c++] = (T)element(i,j);
			}
		}
	}

	//! 行列をポインタで取得
	const real* GetValue() const { return m; }

	//! mpから行列に値を代入
	void SetValue(real *mp)
	{
		int c = 0;
		for(int j = 0; j < 4; j++){
			for(int i = 0; i < 4; i++){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mpから行列に値を代入
	template<typename T> 
	void SetValueT(T *mp)
	{
		int c = 0;
		for(int j = 0; j < 4; j++){
			for(int i = 0; i < 4; i++){
				element(i,j) = (T)mp[c++];
			}
		}
	}

	//! 行列のすべての値をrにする
	void SetValue(real r)
	{
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				element(i,j) = r;
			}
		}
	}

	//! 単位行列生成
	void MakeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		element(0,2) = 0.0; 
		element(0,3) = 0.0;
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
		element(1,2) = 0.0;
		element(1,3) = 0.0;
		
		element(2,0) = 0.0;
		element(2,1) = 0.0;
		element(2,2) = 1.0;
		element(2,3) = 0.0;
		
		element(3,0) = 0.0; 
		element(3,1) = 0.0; 
		element(3,2) = 0.0;
		element(3,3) = 1.0;
	}

	//! 単位行列を返す
	static rxMatrix4 Identity()
	{
		static rxMatrix4 mident(
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0);

		return mident;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(real s)
	{
		element(0,0) = s;
		element(1,1) = s;
		element(2,2) = s;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(const Vec3 &s)
	{
		element(0,0) = s[0];
		element(1,1) = s[1];
		element(2,2) = s[2];
	}

	//! 平行移動成分(0,3)-(0,2)を設定
	void SetTranslate(const Vec3 &t)
	{
		element(0,3) = t[0];
		element(1,3) = t[1];
		element(2,3) = t[2];
	}

	//! r行成分を設定
	void SetRow(int r, const Vec4 &t)
	{
		element(r,0) = t[0];
		element(r,1) = t[1];
		element(r,2) = t[2];
		element(r,3) = t[3];
	}

	//! c列成分を設定
	void SetColumn(int c, const Vec4&t)
	{
		element(0,c) = t[0];
		element(1,c) = t[1];
		element(2,c) = t[2];
		element(3,c) = t[3];
	}

	//! r行成分取得
	void GetRow(int r, Vec4&t) const
	{
		t[0] = element(r,0);
		t[1] = element(r,1);
		t[2] = element(r,2);
		t[3] = element(r,3);
	}

	//! r行成分取得
	Vec4 GetRow(int r) const
	{
		Vec4 v;
		GetRow(r, v);
		return v;
	}

	//! c列成分取得
	void GetColumn(int c, Vec4&t) const
	{
		t[0] = element(0,c);
		t[1] = element(1,c);
		t[2] = element(2,c);
		t[3] = element(3,c);
	}

	//! c列成分取得
	Vec4 GetColumn(int c) const
	{
		Vec4 v;
		GetColumn(c, v);
		return v;
	}

	//! 逆行列取得
	rxMatrix4 Inverse() const
	{
		rxMatrix4 minv;
		
		real r1[8], r2[8], r3[8], r4[8];
		real *s[4], *tmprow;
		
		s[0] = &r1[0];
		s[1] = &r2[0];
		s[2] = &r3[0];
		s[3] = &r4[0];
		
		register int i,j,p,jj;
		for(i = 0; i < 4; ++i){
			for(j = 0; j < 4; ++j){
				s[i][j] = element(i, j);

				if(i==j){
					s[i][j+4] = 1.0;
				}
				else{
					s[i][j+4] = 0.0;
				}
			}
		}

		real scp[4];
		for(i = 0; i < 4; ++i){
			scp[i] = real(fabs(s[i][0]));
			for(j=1; j < 4; ++j)
				if(real(fabs(s[i][j])) > scp[i]) scp[i] = real(fabs(s[i][j]));
				if(scp[i] == 0.0) return minv; // singular matrix!
		}
		
		int pivot_to;
		real scp_max;
		for(i = 0; i < 4; ++i){
			// select pivot row
			pivot_to = i;
			scp_max = real(fabs(s[i][i]/scp[i]));
			// find out which row should be on top
			for(p = i+1; p < 4; ++p)
				if(real(fabs(s[p][i]/scp[p])) > scp_max){
					scp_max = real(fabs(s[p][i]/scp[p])); pivot_to = p;
				}
				// Pivot if necessary
				if(pivot_to != i)
				{
					tmprow = s[i];
					s[i] = s[pivot_to];
					s[pivot_to] = tmprow;
					real tmpscp;
					tmpscp = scp[i];
					scp[i] = scp[pivot_to];
					scp[pivot_to] = tmpscp;
				}
				
				real mji;
				// perform gaussian elimination
				for(j = i+1; j < 4; ++j)
				{
					mji = s[j][i]/s[i][i];
					s[j][i] = 0.0;
					for(jj=i+1; jj<8; jj++)
						s[j][jj] -= mji*s[i][jj];
				}
		}

		if(s[3][3] == 0.0) return minv; // singular matrix!
		
		//
		// Now we have an upper triangular matrix.
		//
		//  x x x x | y y y y
		//  0 x x x | y y y y 
		//  0 0 x x | y y y y
		//  0 0 0 x | y y y y
		//
		//  we'll back substitute to Get the inverse
		//
		//  1 0 0 0 | z z z z
		//  0 1 0 0 | z z z z
		//  0 0 1 0 | z z z z
		//  0 0 0 1 | z z z z 
		//
		
		real mij;
		for(i = 3; i > 0; --i){
			for(j = i-1; j > -1; --j){
				mij = s[j][i]/s[i][i];
				for(jj = j+1; jj < 8; ++jj){
					s[j][jj] -= mij*s[i][jj];
				}
			}
		}
		
		for(i = 0; i < 4; ++i){
			for(j = 0; j < 4; ++j){
				minv(i,j) = s[i][j+4]/s[i][i];
			}
		}
			
		return minv;
	}

	//! 転置行列取得
    rxMatrix4 Transpose() const
	{
		rxMatrix4 mtrans;
		
		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				mtrans(i,j) = element(j,i);
			}
		}

		return mtrans;
	}

	//! 行列を右側から掛ける
	rxMatrix4 &multRight(const rxMatrix4 &b)
	{
		rxMatrix4 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				for(int c = 0; c < 4; ++c){
					element(i,j) += mt(i,c)*b(c,j);
				}
			}
		}

		return *this;
	}    

	//! 行列を左側から掛ける
	rxMatrix4 &multLeft(const rxMatrix4 &b)
	{
		rxMatrix4 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 4; ++i){
			for(int j = 0; j < 4; ++j){
				for(int c = 0; c < 4; ++c){
					element(i,j) += b(i,c)*mt(c,j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void multMatrixVec(const Vec3 &src, Vec3 &dst) const
	{
		real w = (src[0]*element(3,0)+src[1]*element(3,1)+src[2]*element(3,2)+element(3,3));
	    
		assert(w != 0.0);

		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)+element(0,3))/w;
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)+element(1,3))/w;
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)+element(2,3))/w;
	}

	void multMatrixVec(Vec3 & src_and_dst) const
	{ 
		multMatrixVec(Vec3(src_and_dst), src_and_dst); 
	}


	//! dst = src*M
	void multVecMatrix(const Vec3 &src, Vec3 &dst) const
	{
		real w = (src[0]*element(0,3)+src[1]*element(1,3)+src[2]*element(2,3)+element(3,3));
	    
		assert(w != 0.0);

		dst[0] = (src[0]*element(0,0)+src[1]*element(1,0)+src[2]*element(2,0)+element(3,0))/w;
		dst[1] = (src[0]*element(0,1)+src[1]*element(1,1)+src[2]*element(2,1)+element(3,1))/w;
		dst[2] = (src[0]*element(0,2)+src[1]*element(1,2)+src[2]*element(2,2)+element(3,2))/w;
	}
	    

	void multVecMatrix(Vec3 & src_and_dst) const
	{ 
		multVecMatrix(Vec3(src_and_dst), src_and_dst);
	}

	//! dst = M*src
	void multMatrixVec(const Vec4 &src, Vec4 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)+src[3]*element(0,3));
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)+src[3]*element(1,3));
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)+src[3]*element(2,3));
		dst[3] = (src[0]*element(3,0)+src[1]*element(3,1)+src[2]*element(3,2)+src[3]*element(3,3));
	}

	void multMatrixVec(Vec4 &src_and_dst) const
	{
		multMatrixVec(Vec4(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multVecMatrix(const Vec4 &src, Vec4 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(1,0)+src[2]*element(2,0)+src[3]*element(3,0));
		dst[1] = (src[0]*element(0,1)+src[1]*element(1,1)+src[2]*element(2,1)+src[3]*element(3,1));
		dst[2] = (src[0]*element(0,2)+src[1]*element(1,2)+src[2]*element(2,2)+src[3]*element(3,2));
		dst[3] = (src[0]*element(0,3)+src[1]*element(1,3)+src[2]*element(2,3)+src[3]*element(3,3));
	}
	    

	void multVecMatrix(Vec4 & src_and_dst) const
	{
		multVecMatrix(Vec4(src_and_dst), src_and_dst);
	}


	//! dst = M*src
	void multMatrixDir(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+src[1]*element(0,1)+src[2]*element(0,2)) ;
		dst[1] = (src[0]*element(1,0)+src[1]*element(1,1)+src[2]*element(1,2)) ;
		dst[2] = (src[0]*element(2,0)+src[1]*element(2,1)+src[2]*element(2,2)) ;
	}
	    

	void multMatrixDir(Vec3 & src_and_dst) const
	{
		multMatrixDir(Vec3(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multDirMatrix(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = (src[0]*element(0,0)+
				  src[1]*element(1,0)+
				  src[2]*element(2,0)) ;
		dst[1] = (src[0]*element(0,1)+
				  src[1]*element(1,1)+
				  src[2]*element(2,1)) ;
		dst[2] = (src[0]*element(0,2)+
				  src[1]*element(1,2)+
				  src[2]*element(2,2)) ;
	}

	void multDirMatrix(Vec3 &src_and_dst) const
	{ 
		multDirMatrix(Vec3(src_and_dst), src_and_dst);
	}

	//! 行列の要素を返す
	real& element(int row, int col){ return m[row | (col<<2)]; }
	const real& element(int row, int col) const { return m[row | (col<<2)]; }


	//! オペレータ
	real& operator()(int row, int col){ return element(row,col); }
	const real& operator()(int row, int col) const	{ return element(row,col); }

	rxMatrix4& operator*=(const rxMatrix4 &mat)
	{
		multRight(mat);
		return *this;
	}

	rxMatrix4& operator*=(const real & r)
	{
		for(int i = 0; i < 4; ++i){
			element(0,i) *= r;
			element(1,i) *= r;
			element(2,i) *= r;
			element(3,i) *= r;
		}

		return *this;
	}

	rxMatrix4& operator+=(const rxMatrix4 &mat)
	{
		for(int i = 0; i < 4; ++i){
			element(0,i) += mat.element(0,i);
			element(1,i) += mat.element(1,i);
			element(2,i) += mat.element(2,i);
			element(3,i) += mat.element(3,i);
		}

		return *this;
	}

	friend rxMatrix4 operator*(const rxMatrix4 &m1, const rxMatrix4 &m2);
	friend Vec3 operator*(const rxMatrix4 &m, const Vec3 &v);
	friend bool operator==(const rxMatrix4 &m1, const rxMatrix4 &m2);
	friend bool operator!=(const rxMatrix4 &m1, const rxMatrix4 &m2);
};

inline rxMatrix4 operator*(const rxMatrix4 &m1, const rxMatrix4 &m2)
{
	rxMatrix4 product;
	
	product = m1;
	product.multRight(m2);
	
	return product;
}

inline Vec3 operator*(const rxMatrix4 &m, const Vec3 &v)
{
	Vec3 product;
	
	m.multMatrixVec(v, product);
	
	return product;
}

inline bool operator==(const rxMatrix4 &m1, const rxMatrix4 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(0,2) == m2(0,2) &&
			m1(0,3) == m2(0,3) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1) &&
			m1(1,2) == m2(1,2) &&
			m1(1,3) == m2(1,3) &&
			m1(2,0) == m2(2,0) &&
			m1(2,1) == m2(2,1) &&
			m1(2,2) == m2(2,2) &&
			m1(2,3) == m2(2,3) &&
			m1(3,0) == m2(3,0) &&
			m1(3,1) == m2(3,1) &&
			m1(3,2) == m2(3,2) &&
			m1(3,3) == m2(3,3));
}

inline bool operator!=(const rxMatrix4 &m1, const rxMatrix4 &m2)
{
	return !(m1 == m2);
}  



//-----------------------------------------------------------------------------
//! 3x3行列クラス
//-----------------------------------------------------------------------------
class rxMatrix3
{
protected:
	real m[9];

public:
	//! コンストラクタ
	rxMatrix3(){ makeIdentity(); }
	rxMatrix3(real r){ SetValue(r); }
	rxMatrix3(real *m){ SetValue(m); }
	rxMatrix3(real a00, real a01, real a02,
			  real a10, real a11, real a12,
			  real a20, real a21, real a22)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
	}

	//! 行列をmpに入れる
	void GetValue(real *mp) const
	{
		int c = 0;
		for(int j = 0; j < 3; j++){
			for(int i = 0; i < 3; i++){
				mp[c++] = element(i,j);
			}
		}
	}

	//! 行列をmpに入れる
	void GetValue4x4(real *mp) const
	{
		mp[0]  = element(0,0);
		mp[1]  = element(1,0);
		mp[2]  = element(2,0);
		mp[3]  = 0.0;	  
						  
		mp[4]  = element(0,1);
		mp[5]  = element(1,1);
		mp[6]  = element(2,1);
		mp[7]  = 0.0;	  
						  
		mp[8]  = element(0,2);
		mp[9]  = element(1,2);
		mp[10] = element(2,2);
		mp[11] = 0.0;
	}

	//! 行列をポインタで取得
	const real* GetValue() const { return m; }

	//! mpから行列に値を代入
	void SetValue(real *mp)
	{
		int c = 0;
		for(int j = 0; j < 3; j++){
			for(int i = 0; i < 3; i++){
				element(i,j) = mp[c++];
			}
		}
	}

	//! mpから行列に値を代入
	void SetValue4x4(real *mp)
	{
		element(0,0) = mp[0] ;
		element(1,0) = mp[1] ;
		element(2,0) = mp[2] ;
				 
		element(0,1)  =mp[4] ;
		element(1,1) = mp[5] ;
		element(2,1) = mp[6] ;

		element(0,2) = mp[8] ;
		element(1,2) = mp[9] ;
		element(2,2) = mp[10];
	}					  
						  
	//! 行列のすべての値をにする
	void SetValue(real r)  
	{					  
		for(int i = 0; i < 3; i++){
			for(int j = 0;j < 3; j++){
				element(i,j) = r;
			}			  
		}				  
	}
	void SetValue(real a00, real a01, real a02,
				  real a10, real a11, real a12,
				  real a20, real a21, real a22)
	{
		element(0,0) = a00;
		element(0,1) = a01;
		element(0,2) = a02;
		
		element(1,0) = a10;
		element(1,1) = a11;
		element(1,2) = a12;
		
		element(2,0) = a20;
		element(2,1) = a21;
		element(2,2) = a22;
	}

	//! オイラー角から回転行列を設定
	void SetEuler(const double& yaw, const double& pitch, const double& roll) 
	{
		// yaw is CW around y-axis, pitch is CCW around x-axis, and roll is CW around z-axis
		double cy = cos(yaw); 
		double sy = sin(yaw); 
		double cp = cos(pitch); 
		double sp = sin(pitch); 
		double cr = cos(roll);
		double sr = sin(roll);

		double cc = cy*cr; 
		double cs = cy*sr; 
		double sc = sy*cr; 
		double ss = sy*sr;
		SetValue(cc+sp*ss, cs-sp*sc, -sy*cp,
				 -cp*sr,   cp*cr,    -sp,
				 sc-sp*cs, ss+sp*cc, cy*cp);
				    
	}

	//! 単位行列生成
	void makeIdentity()
	{
		element(0,0) = 1.0;
		element(0,1) = 0.0;
		element(0,2) = 0.0; 
		
		element(1,0) = 0.0;
		element(1,1) = 1.0; 
		element(1,2) = 0.0;
		
		element(2,0) = 0.0;
		element(2,1) = 0.0;
		element(2,2) = 1.0;
	}

	//! 単位行列を返す
	static rxMatrix3 Identity()
	{
		static rxMatrix3 mident(
			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0);

		return mident;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(real s)
	{
		element(0,0) = s;
		element(1,1) = s;
		element(2,2) = s;
	}

	//! スケール(対角成分)を設定	    
	void SetScale(const Vec3 &s)
	{
		element(0,0) = s[0];
		element(1,1) = s[1];
		element(2,2) = s[2];
	}

	//! 平行移動成分(0,3)-(0,2)を設定
	void SetTranslate(const Vec3 &t)
	{
//		element(0,3) = t[0];
//		element(1,3) = t[1];
//		element(2,3) = t[2];
	}

	//! r行成分を設定
	void SetRow(int r, const Vec3 &t)
	{
		element(r,0) = t[0];
		element(r,1) = t[1];
		element(r,2) = t[2];
	}

	//! c列成分を設定
	void SetColumn(int c, const Vec3 &t)
	{
		element(0,c) = t[0];
		element(1,c) = t[1];
		element(2,c) = t[2];
	}

	//! r行成分取得
	void GetRow(int r, Vec3 &t) const
	{
		t[0] = element(r,0);
		t[1] = element(r,1);
		t[2] = element(r,2);
	}

	//! r行成分取得
	Vec3 GetRow(int r) const
	{
		Vec3 v;
		GetRow(r, v);
		return v;
	}

	//! c列成分取得
	void GetColumn(int c, Vec3 &t) const
	{
		t[0] = element(0,c);
		t[1] = element(1,c);
		t[2] = element(2,c);
	}

	//! c列成分取得
	Vec3 GetColumn(int c) const
	{
		Vec3 v;
		GetColumn(c, v);
		return v;
	}

	//! 逆行列取得
	rxMatrix3 Inverse() const
	{
		real d = element(0, 0)*element(1, 1)*element(2, 2)- 
				 element(0, 0)*element(2, 1)*element(1, 2)+ 
				 element(1, 0)*element(2, 1)*element(0, 2)- 
				 element(1, 0)*element(0, 1)*element(2, 2)+ 
				 element(2, 0)*element(0, 1)*element(1, 2)- 
				 element(2, 0)*element(1, 1)*element(0, 2);

		if(d == 0) d = 1;

		return	rxMatrix3( (element(1, 1)*element(2, 2)-element(1, 2)*element(2, 1))/d,
						  -(element(0, 1)*element(2, 2)-element(0, 2)*element(2, 1))/d,
						   (element(0, 1)*element(1, 2)-element(0, 2)*element(1, 1))/d,
						  -(element(1, 0)*element(2, 2)-element(1, 2)*element(2, 0))/d,
						   (element(0, 0)*element(2, 2)-element(0, 2)*element(2, 0))/d,
						  -(element(0, 0)*element(1, 2)-element(0, 2)*element(1, 0))/d,
						   (element(1, 0)*element(2, 1)-element(1, 1)*element(2, 0))/d,
						  -(element(0, 0)*element(2, 1)-element(0, 1)*element(2, 0))/d,
						   (element(0, 0)*element(1, 1)-element(0, 1)*element(1, 0))/d);	
	}

	//! 転置行列取得
    rxMatrix3 Transpose() const
	{
		rxMatrix3 mtrans;
		
		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				mtrans(i, j) = element(j, i);
			}
		}

		return mtrans;
	}
		
		
	rxMatrix3 Scaled(const Vec3& s) const
	{
		return rxMatrix3(element(0, 0)*s[0], element(0, 1)*s[1], element(0, 2)*s[2],
						 element(1, 0)*s[0], element(1, 1)*s[1], element(1, 2)*s[2],
						 element(2, 0)*s[0], element(2, 1)*s[1], element(2, 2)*s[2]);
	}

	//! 行列を右側から掛ける
	rxMatrix3 &multRight(const rxMatrix3 &b)
	{
		rxMatrix3 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				for(int c = 0; c < 3; ++c){
					element(i, j) += mt(i, c)*b(c, j);
				}
			}
		}

		return *this;
	}    

	//! 行列を左側から掛ける
	rxMatrix3 &multLeft(const rxMatrix3 &b)
	{
		rxMatrix3 mt(*this);
		SetValue(real(0));

		for(int i = 0; i < 3; ++i){
			for(int j = 0; j < 3; ++j){
				for(int c = 0; c < 3; ++c){
					element(i, j) += b(i, c)*mt(c, j);
				}
			}
		}

		return *this;
	}

	//! dst = M*src
	void multMatrixVec(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(0, 1)+src[2]*element(0, 2);
		dst[1] = src[0]*element(1, 0)+src[1]*element(1, 1)+src[2]*element(1, 2);
		dst[2] = src[0]*element(2, 0)+src[1]*element(2, 1)+src[2]*element(2, 2);
	}

	void multMatrixVec(Vec3 &src_and_dst) const
	{
		multMatrixVec(Vec3(src_and_dst), src_and_dst);
	}


	//! dst = src*M
	void multVecMatrix(const Vec3 &src, Vec3 &dst) const
	{
		dst[0] = src[0]*element(0, 0)+src[1]*element(1, 0)+src[2]*element(2, 0);
		dst[1] = src[0]*element(0, 1)+src[1]*element(1, 1)+src[2]*element(2, 1);
		dst[2] = src[0]*element(0, 2)+src[1]*element(1, 2)+src[2]*element(2, 2);
	}
	    

	void multVecMatrix(Vec3 & src_and_dst) const
	{
		multVecMatrix(Vec3(src_and_dst), src_and_dst);
	}


	//! 行列の要素を返す
	real& element(int row, int col){ return m[3*row+col]; }
	const real& element(int row, int col) const { return m[3*row+col]; }


	//! オペレータ
	real& operator()(int row, int col){ return element(row, col); }
	const real& operator()(int row, int col) const	{ return element(row, col); }

	rxMatrix3& operator*=(const rxMatrix3 &mat)
	{
		multRight(mat);
		return *this;
	}

	rxMatrix3& operator*=(const real &r)
	{
		for(int i = 0; i < 3; ++i){
			element(0,i) *= r;
			element(1,i) *= r;
			element(2,i) *= r;
		}

		return *this;
	}

	rxMatrix3& operator+=(const rxMatrix3 &mat)
	{
		for(int i = 0; i < 3; ++i){
			element(0,i) += mat.element(0,i);
			element(1,i) += mat.element(1,i);
			element(2,i) += mat.element(2,i);
		}

		return *this;
	}

	friend rxMatrix3 operator*(const rxMatrix3 &m1, const rxMatrix3 &m2);
	friend Vec3 operator*(const rxMatrix3 &m, const Vec3 &v);
	friend Vec3 operator*(const Vec3 &v, const rxMatrix3 &m);
	friend bool operator==(const rxMatrix3 &m1, const rxMatrix3 &m2);
	friend bool operator!=(const rxMatrix3 &m1, const rxMatrix3 &m2);
};

inline rxMatrix3 operator*(const rxMatrix3 &m1, const rxMatrix3 &m2)
{
	rxMatrix3 product;
	
	product = m1;
	product.multRight(m2);
	
	return product;
}

inline Vec3 operator*(const rxMatrix3 &m, const Vec3 &v)
{
	Vec3 product;
	
	m.multMatrixVec(v, product);
	
	return product;
}

inline Vec3 operator*(const Vec3 &v, const rxMatrix3 &m)
{
	Vec3 product;
	
	m.multVecMatrix(v, product);
	
	return product;
}

inline bool operator==(const rxMatrix3 &m1, const rxMatrix3 &m2)
{
	return (m1(0,0) == m2(0,0) &&
			m1(0,1) == m2(0,1) &&
			m1(0,2) == m2(0,2) &&
			m1(1,0) == m2(1,0) &&
			m1(1,1) == m2(1,1) &&
			m1(1,2) == m2(1,2) &&
			m1(2,0) == m2(2,0) &&
			m1(2,1) == m2(2,1) &&
			m1(2,2) == m2(2,2));
}

inline bool operator!=(const rxMatrix3 &m1, const rxMatrix3 &m2)
{
	return !(m1 == m2);
}  



#endif // _MATRIX_H_