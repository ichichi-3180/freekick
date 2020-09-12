/*! 
 @file rx_shader.h

 @brief 髪レンダリング用GLSLシェーダー
 
 @author Makoto Fujisawa
 @date 2009-11
*/

//-----------------------------------------------------------------------------
// インクルードファイル
//-----------------------------------------------------------------------------
#include <GL/glut.h>


#define RXSTR(A) #A


//-----------------------------------------------------------------------------
// HACK:GLSLシェーダ
//-----------------------------------------------------------------------------
struct rxGLSL
{
	string VertProg;
	string FragProg;
	string Name;
	GLuint Prog;
};


//-----------------------------------------------------------------------------
// Shadow Map シェーダ
//-----------------------------------------------------------------------------
//MARK:Shadow Map vs
const char shadow_vs[] = RXSTR(
varying vec4 vPos;
varying vec3 vNrm;
void main(void)
{
	vPos = gl_ModelViewMatrix*gl_Vertex;
	vNrm = normalize(gl_NormalMatrix*gl_Normal);
	//vNrm = gl_NormalMatrix*gl_Normal;

	gl_Position = gl_ProjectionMatrix*vPos;
	gl_FrontColor = gl_Color*gl_LightSource[0].diffuse*vec4(max(dot(vNrm, gl_LightSource[0].position.xyz), 0.0));
	//gl_FrontColor = gl_Color;
	gl_TexCoord[0] = gl_MultiTexCoord0;
	//gl_TexCoord[0] = gl_TextureMatrix[0] * gl_MultiTexCoord0;

}
);

// MARK:Normal Mode fs
const char *shadow_single_fs = RXSTR( 
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;			//!< 模様
uniform sampler2DArray stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;				//!< 視錐台遠距離
varying vec4 vPos;
varying vec3 vNrm;

float ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
	}
	
	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;

	// 光源からのデプス値(遮蔽無し)を待避
	float light_d = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);
	
	// 格納された光源からの最小デプス値を取得
	float shadow_d = texture2DArray(stex, shadow_coord.xyz).x;
	
	// 光源からのデプス値と遮蔽を考慮したデプス値の差を求める
	float diff = shadow_d-light_d;

	// 影で0,日向で1を返す
	return clamp(diff*250.0+1.0, 0.0, 1.0);
}

void main(void)
{
	vec4 light_col;
	vec3 N = normalize(vNrm);
	//vec3 L = gl_LightSource[0].position.xyz;
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// 環境光の計算
	//  - OpenGLが計算した光源強度と反射係数の積(gl_FrontLightProduct)を用いる．
	light_col = gl_FrontLightProduct[0].ambient;

	// 拡散反射の計算
	float diff = max(dot(L, N), 0.0);

	// 鏡面反射の計算
	if(diff > 0.0){
		// 反射ベクトルの計算
		vec3 V = normalize(-vPos.xyz);
		//vec3 R = 2.0*dot(N, L)*N-L;
		vec3 R = reflect(-L, N);
		float spec = pow(abs(dot(R, V)), gl_FrontMaterial.shininess);

		light_col += gl_FrontLightProduct[0].diffuse*diff+
					 gl_FrontLightProduct[0].specular*spec;
	}

	const float shadow_ambient = 0.8;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	float shadow_coef = ShadowCoef();

	light_col += color_tex;

	gl_FragColor = shadow_ambient*shadow_coef*light_col+(1.0-shadow_ambient)*color_tex;
	//gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
	//gl_FragColor = shadow_ambient*shadow_coef*light_col*color_tex+(1.0-shadow_ambient)*color_tex;

}
);


//! 複数のシャドウサンプルを使用(リーク抑制版)
// MARK:Show Splits fs
const char *shadow_single_hl_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;			//!< 模様
uniform sampler2DArray stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;				//!< 視錐台遠距離
uniform vec4 color[4] = vec4[4](vec4(1.0, 0.5, 0.5, 1.0),
								vec4(0.5, 1.0, 0.5, 1.0),
								vec4(0.5, 0.5, 1.0, 1.0),
								vec4(1.0, 1.0, 0.5, 1.0));

// バーテックスシェーダから受け取る変数
varying vec4 vPos;
varying vec3 vNrm;

vec4 ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
	}
	
	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;

	// 光源からのデプス値(遮蔽無し)を待避
	float light_d = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);
	
	// 格納された光源からの最小デプス値を取得
	float shadow_d = texture2DArray(stex, shadow_coord.xyz).x;
	
	// 光源からのデプス値と遮蔽を考慮したデプス値の差を求める
	float diff = shadow_d-light_d;

	// 影で0,日向で1を返す
	return clamp(diff*250.0+1.0, 0.0, 1.0)*color[index];
}

void main(void)
{
	const float shadow_ambient = 0.9;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	vec4 shadow_coef = ShadowCoef();
	gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
}
);


//! 複数のシャドウサンプルを使用
// MARK:Smooth shadows fs
const char *shadow_multi_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;			//!< 模様
uniform sampler2DArray stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;				//!< 視錐台遠距離

// バーテックスシェーダから受け取る変数
varying vec4 vPos;
varying vec3 vNrm;

// 周囲の影係数参照用
const int nsamples = 8;
uniform vec4 offset[nsamples] = vec4[nsamples](vec4(0.000000, 0.000000, 0.0, 0.0),
											   vec4(0.079821, 0.165750, 0.0, 0.0),
											   vec4(-0.331500, 0.159642, 0.0, 0.0),
											   vec4(-0.239463, -0.497250, 0.0, 0.0),
											   vec4(0.662999, -0.319284, 0.0, 0.0),
											   vec4(0.399104, 0.828749, 0.0, 0.0),
											   vec4(-0.994499, 0.478925, 0.0, 0.0),
											   vec4(-0.558746, -1.160249, 0.0, 0.0));

/*!
 * 影係数の計算
 * @param[in] shadow_coord シャドウマップ参照用テクスチャ座標
 * @param[in] light_d 参照位置での光源からのデプス値
 * @return 影係数(影のあるところで1, それ以外で0)
 */
float GetOccCoef(vec4 shadow_coord, float light_d)
{
	// 格納された光源からの最小デプス値を取得
	float shadow_d = texture2DArray(stex, shadow_coord.xyz).x;
	
	// 光源からのデプス値と遮蔽を考慮したデプス値の差を求める
	float diff = shadow_d-light_d;
	
	// 影で0,日向で1を返す
	return clamp(diff*250.0+1.0, 0.0, 1.0);
}

/*!
 * 影生成のための係数(影のあるところで1, それ以外で0)
 * @return 影係数(影のあるところで1, それ以外で0)
 */
float ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
	}
	
	const float scale = 2.0/4096.0;

	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;
	
	// 光源からのデプス値(遮蔽無し)を待避
	float light_d = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);
	
	// 周囲の影係数も含めて取得
	float shadow_coef = 0.0;
	for(int i = 0; i < nsamples; ++i){
		shadow_coef += GetOccCoef(shadow_coord+scale*offset[i], light_d);
	}
	shadow_coef /= nsamples;
	
	return shadow_coef;
}

void main(void)
{
	vec4 light_col;
	vec3 N = normalize(vNrm);
	//vec3 L = gl_LightSource[0].position.xyz;
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// 環境光の計算
	//  - OpenGLが計算した光源強度と反射係数の積(gl_FrontLightProduct)を用いる．
	light_col = gl_FrontLightProduct[0].ambient;

	// 拡散反射の計算
	float diff = max(dot(L, N), 0.0);

	// 鏡面反射の計算
	if(diff > 0.0){
		// 反射ベクトルの計算
		vec3 V = normalize(-vPos.xyz);
		//vec3 R = 2.0*dot(N, L)*N-L;
		vec3 R = reflect(-L, N);
		float spec = pow(abs(dot(R, V)), gl_FrontMaterial.shininess);

		light_col += gl_FrontLightProduct[0].diffuse*diff+
					 gl_FrontLightProduct[0].specular*spec;
	}

	const float shadow_ambient = 0.8;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	float shadow_coef = ShadowCoef();

	gl_FragColor = shadow_ambient*shadow_coef*light_col+(1.0-shadow_ambient)*light_col;
	//gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
}
);


//! 複数のシャドウサンプルを使用(リーク抑制版)
// MARK:Smooth shadows, no leak fs
const char *shadow_multi_noleak_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;			//!< 模様
uniform sampler2DArray stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;				//!< 視錐台遠距離

// バーテックスシェーダから受け取る変数
varying vec4 vPos;
varying vec3 vNrm;

// 周囲の影係数参照用
const int nsamples = 8;
uniform vec4 offset[nsamples] = vec4[nsamples](vec4(0.000000, 0.000000, 0.0, 0.0),
											   vec4(0.079821, 0.165750, 0.0, 0.0),
											   vec4(-0.331500, 0.159642, 0.0, 0.0),
											   vec4(-0.239463, -0.497250, 0.0, 0.0),
											   vec4(0.662999, -0.319284, 0.0, 0.0),
											   vec4(0.399104, 0.828749, 0.0, 0.0),
											   vec4(-0.994499, 0.478925, 0.0, 0.0),
											   vec4(-0.558746, -1.160249, 0.0, 0.0));

/*!
 * 影係数の計算
 * @param[in] shadow_coord シャドウマップ参照用テクスチャ座標
 * @param[in] light_d 参照位置での光源からのデプス値
 * @return 影係数(影のあるところで1, それ以外で0)
 */
float GetOccCoef(vec4 shadow_coord, float light_d)
{
	// 格納された光源からの最小デプス値を取得
	float shadow_d = texture2DArray(stex, shadow_coord.xyz).x;
	
	// 光源からのデプス値と遮蔽を考慮したデプス値の差を求める
	float diff = shadow_d-light_d;
	
	// 影で0,日向で1を返す
	return clamp(diff*250.0+1.0, 0.0, 1.0);
}

/*!
 * 影生成のための係数(影のあるところで1, それ以外で0)
 * @return 影係数(影のあるところで1, それ以外で0)
 */
float ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
	}
	
	const float scale = 2.0/4096.0;

	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;

	vec4 light_normal4 = gl_TextureMatrix[index+4]*vec4(vNrm, 0.0);
	vec3 light_normal = normalize(light_normal4.xyz);
	
	float d = -dot(light_normal, shadow_coord.xyz);

	// 光源からのデプス値(遮蔽無し)を待避
	float light_d = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);
	
	// 周囲の影係数も含めて取得
	float shadow_coef = GetOccCoef(shadow_coord, light_d);
	for(int i = 1; i < nsamples; ++i){
		vec4 shadow_lookup = shadow_coord+scale*offset[i];

		float lookup_z = -(light_normal.x*shadow_lookup.x + light_normal.y*shadow_lookup.y + d)/light_normal.z;

		shadow_coef += GetOccCoef(shadow_lookup, lookup_z);
	}
	shadow_coef /= nsamples;
	
	return shadow_coef;
}

void main(void)
{
	vec4 light_col;
	vec3 N = normalize(vNrm);

	//vec3 L = gl_LightSource[0].position.xyz;
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// 環境光の計算
	//  - OpenGLが計算した光源強度と反射係数の積(gl_FrontLightProduct)を用いる．
	light_col = gl_FrontLightProduct[0].ambient;

	// 拡散反射の計算
	float diff = max(dot(L, N), 0.0);

	// 鏡面反射の計算
	if(diff > 0.0){
		// 反射ベクトルの計算
		vec3 V = normalize(-vPos.xyz);
		//vec3 R = 2.0*dot(N, L)*N-L;
		vec3 R = reflect(-L, N);
		float spec = pow(abs(dot(R, V)), gl_FrontMaterial.shininess);

		light_col += gl_FrontLightProduct[0].diffuse*diff+
					 gl_FrontLightProduct[0].specular*spec;
	}

	const float shadow_ambient = 0.9;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	float shadow_coef = ShadowCoef();

	//light_col += color_tex;

	gl_FragColor = shadow_ambient*shadow_coef*light_col+(1.0-shadow_ambient)*light_col;
	//gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
}
);


// sampler2DArrayShadowを使用
// MARK:PCF fs
const char *shadow_pcf_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;				//!< 模様
uniform sampler2DArrayShadow stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;					//!< 視錐台遠距離
uniform vec2 texSize;				//!< x - size, y - 1/size

// バーテックスシェーダから受け取る変数
varying vec4 vPos;
varying vec3 vNrm;

float ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
	}
	
	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;

	// 光源からのデプス値(遮蔽無し)を待避
	shadow_coord.w = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);
	
	// 影で0,日向で1を返す
	return shadow2DArray(stex, shadow_coord).x;
}

void main(void)
{
	vec4 light_col;
	vec3 N = normalize(vNrm);
	//vec3 L = gl_LightSource[0].position.xyz;
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// 環境光の計算
	//  - OpenGLが計算した光源強度と反射係数の積(gl_FrontLightProduct)を用いる．
	light_col = gl_FrontLightProduct[0].ambient;

	// 拡散反射の計算
	float diff = max(dot(L, N), 0.0);

	// 鏡面反射の計算
	if(diff > 0.0){
		// 反射ベクトルの計算
		vec3 V = normalize(-vPos.xyz);
		//vec3 R = 2.0*dot(N, L)*N-L;
		vec3 R = reflect(-L, N);
		float spec = pow(abs(dot(R, V)), gl_FrontMaterial.shininess);

		light_col += gl_FrontLightProduct[0].diffuse*diff+
					 gl_FrontLightProduct[0].specular*spec;
	}

	const float shadow_ambient = 0.8;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	float shadow_coef = ShadowCoef();

	gl_FragColor = shadow_ambient*shadow_coef*light_col+(1.0-shadow_ambient)*light_col;
	//gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
}
);


// sampler2DArrayShadowを使用
// MARK:PCF w/ trilinear fs
const char *shadow_pcf_trilinear_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;				//!< 模様
uniform sampler2DArrayShadow stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;					//!< 視錐台遠距離
uniform vec2 texSize;				//!< x - size, y - 1/size

// バーテックスシェーダから受け取る変数
varying vec4 vPos;
varying vec3 vNrm;

float ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	float blend = 0.0;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
		blend = clamp( (gl_FragCoord.z-far_d.x*0.995)*200.0, 0.0, 1.0); 
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
		blend = clamp( (gl_FragCoord.z-far_d.y*0.995)*200.0, 0.0, 1.0); 
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
		blend = clamp( (gl_FragCoord.z-far_d.z*0.995)*200.0, 0.0, 1.0); 
	}
	
	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;

	// 光源からのデプス値(遮蔽無し)を待避
	shadow_coord.w = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);
	
	// 影係数の取得
	float ret = shadow2DArray(stex, shadow_coord).x;
	
	if(blend > 0.0){
	    shadow_coord = gl_TextureMatrix[index+1]*vPos;
	
	    shadow_coord.w = shadow_coord.z;
	    shadow_coord.z = float(index+1);
    	
	    ret = ret*(1.0-blend) + shadow2DArray(stex, shadow_coord).x*blend; 
	}
	
	return ret;
}

void main(void)
{
	vec4 light_col;
	vec3 N = normalize(vNrm);
	//vec3 L = gl_LightSource[0].position.xyz;
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// 環境光の計算
	//  - OpenGLが計算した光源強度と反射係数の積(gl_FrontLightProduct)を用いる．
	light_col = gl_FrontLightProduct[0].ambient;

	// 拡散反射の計算
	float diff = max(dot(L, N), 0.0);

	// 鏡面反射の計算
	if(diff > 0.0){
		// 反射ベクトルの計算
		vec3 V = normalize(-vPos.xyz);
		//vec3 R = 2.0*dot(N, L)*N-L;
		vec3 R = reflect(-L, N);
		float spec = pow(abs(dot(R, V)), gl_FrontMaterial.shininess);

		light_col += gl_FrontLightProduct[0].diffuse*diff+
					 gl_FrontLightProduct[0].specular*spec;
	}

	const float shadow_ambient = 0.8;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	float shadow_coef = ShadowCoef();

	gl_FragColor = shadow_ambient*shadow_coef*light_col+(1.0-shadow_ambient)*light_col;
	//gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
}
);


//! 
// MARK:PCF w/ 4 taps fs
const char *shadow_pcf_4tap_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;				//!< 模様
uniform sampler2DArrayShadow stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;					//!< 視錐台遠距離
uniform vec2 texSize;				//!< x - size, y - 1/size

// バーテックスシェーダから受け取る変数
varying vec4 vPos;
varying vec3 vNrm;

float ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
	}
	
	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;

	// 光源からのデプス値(遮蔽無し)を待避
	shadow_coord.w = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);

	// 重み付き4-tapバイリニアフィルタ
	vec2 pos = mod(shadow_coord.xy*texSize.x, 1.0);
	vec2 offset = (0.5-step(0.5, pos))*texSize.y;
	float ret = 0.0f;
	ret += shadow2DArray(stex, shadow_coord+vec4( offset.x,  offset.y, 0, 0)).x * (pos.x) * (pos.y);
	ret += shadow2DArray(stex, shadow_coord+vec4( offset.x, -offset.y, 0, 0)).x * (pos.x) * (1-pos.y);
	ret += shadow2DArray(stex, shadow_coord+vec4(-offset.x,  offset.y, 0, 0)).x * (1-pos.x) * (pos.y);
	ret += shadow2DArray(stex, shadow_coord+vec4(-offset.x, -offset.y, 0, 0)).x * (1-pos.x) * (1-pos.y);
	
	return ret;
}

void main(void)
{
	vec4 light_col;
	vec3 N = normalize(vNrm);
	//vec3 L = gl_LightSource[0].position.xyz;
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// 環境光の計算
	//  - OpenGLが計算した光源強度と反射係数の積(gl_FrontLightProduct)を用いる．
	light_col = gl_FrontLightProduct[0].ambient;

	// 拡散反射の計算
	float diff = max(dot(L, N), 0.0);

	// 鏡面反射の計算
	if(diff > 0.0){
		// 反射ベクトルの計算
		vec3 V = normalize(-vPos.xyz);
		//vec3 R = 2.0*dot(N, L)*N-L;
		vec3 R = reflect(-L, N);
		float spec = pow(abs(dot(R, V)), gl_FrontMaterial.shininess);

		light_col += gl_FrontLightProduct[0].diffuse*diff+
					 gl_FrontLightProduct[0].specular*spec;
	}

	const float shadow_ambient = 0.8;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	float shadow_coef = ShadowCoef();

	gl_FragColor = shadow_ambient*shadow_coef*light_col+(1.0-shadow_ambient)*light_col;
	//gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
}
);


//! 
// MARK:PCF w/ 8 random taps fs
const char *shadow_pcf_8tap_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;				//!< 模様
uniform sampler2DArrayShadow stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;					//!< 視錐台遠距離
uniform vec2 texSize;				//!< x - size, y - 1/size

// バーテックスシェーダから受け取る変数
varying vec4 vPos;
varying vec3 vNrm;

// 周囲の影係数参照用
const int nsamples = 8;
uniform vec4 offset[nsamples] = vec4[nsamples](vec4(0.000000, 0.000000, 0.0, 0.0),
											   vec4(0.079821, 0.165750, 0.0, 0.0),
											   vec4(-0.331500, 0.159642, 0.0, 0.0),
											   vec4(-0.239463, -0.497250, 0.0, 0.0),
											   vec4(0.662999, -0.319284, 0.0, 0.0),
											   vec4(0.399104, 0.828749, 0.0, 0.0),
											   vec4(-0.994499, 0.478925, 0.0, 0.0),
											   vec4(-0.558746, -1.160249, 0.0, 0.0));

float ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
	}
	
	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;

	// 光源からのデプス値(遮蔽無し)を待避
	shadow_coord.w = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);

	// 重み付き8-tapランダムフィルタ
	float ret = 0.0f;
	for(int i = 0; i < nsamples; ++i){
	    vec4 shadow_lookup = shadow_coord+texSize.y*offset[i]*2.0; //scale the offsets to the texture size, and make them twice as large to cover a larger radius
	    ret += shadow2DArray(stex, shadow_lookup).x*0.125;
	}
	
	return ret;
}

void main(void)
{
	vec4 light_col;
	vec3 N = normalize(vNrm);
	//vec3 L = gl_LightSource[0].position.xyz;
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// 環境光の計算
	//  - OpenGLが計算した光源強度と反射係数の積(gl_FrontLightProduct)を用いる．
	light_col = gl_FrontLightProduct[0].ambient;

	// 拡散反射の計算
	float diff = max(dot(L, N), 0.0);

	// 鏡面反射の計算
	if(diff > 0.0){
		// 反射ベクトルの計算
		vec3 V = normalize(-vPos.xyz);
		//vec3 R = 2.0*dot(N, L)*N-L;
		vec3 R = reflect(-L, N);
		float spec = pow(abs(dot(R, V)), gl_FrontMaterial.shininess);

		light_col += gl_FrontLightProduct[0].diffuse*diff+
					 gl_FrontLightProduct[0].specular*spec;
	}

	const float shadow_ambient = 0.8;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	float shadow_coef = ShadowCoef();

	gl_FragColor = shadow_ambient*shadow_coef*light_col+(1.0-shadow_ambient)*light_col;
	//gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
}
);


//! 
// MARK:PCF w/ gaussian blur fs
const char *shadow_pcf_gaussian_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2D tex;				//!< 模様
uniform sampler2DArrayShadow stex;	//!< デプス値テクスチャ(×視錐台分割数)
uniform vec4 far_d;					//!< 視錐台遠距離
uniform vec2 texSize;				//!< x - size, y - 1/size

// バーテックスシェーダから受け取る変数
varying vec4 vPos;
varying vec3 vNrm;

float ShadowCoef(void)
{
	// 画素位置gl_FragCoord.xy におけるデプス値 gl_FragCoord.zから適切なデプスマップを検索
	// 分割数以上の部分には0を代入しておくこと(例えば分割数2なら,far_d.z,far_d.wは0)
	int index = 3;
	if(gl_FragCoord.z < far_d.x){
		index = 0;
	}
	else if(gl_FragCoord.z < far_d.y){
		index = 1;
	}
	else if(gl_FragCoord.z < far_d.z){
		index = 2;
	}
	
	// 視点座標系の位置を光源を視点とした座標系のものに変換
	vec4 shadow_coord = gl_TextureMatrix[index]*vPos;

	// 光源からのデプス値(遮蔽無し)を待避
	shadow_coord.w = shadow_coord.z;
	
	// どの分割を用いるか
	shadow_coord.z = float(index);

	// Gaussian 3x3 filter
	float ret = shadow2DArray(stex, shadow_coord).x * 0.25;
	ret += shadow2DArrayOffset(stex, shadow_coord, ivec2( -1, -1)).x * 0.0625;
	ret += shadow2DArrayOffset(stex, shadow_coord, ivec2( -1, 0)).x * 0.125;
	ret += shadow2DArrayOffset(stex, shadow_coord, ivec2( -1, 1)).x * 0.0625;
	ret += shadow2DArrayOffset(stex, shadow_coord, ivec2( 0, -1)).x * 0.125;
	ret += shadow2DArrayOffset(stex, shadow_coord, ivec2( 0, 1)).x * 0.125;
	ret += shadow2DArrayOffset(stex, shadow_coord, ivec2( 1, -1)).x * 0.0625;
	ret += shadow2DArrayOffset(stex, shadow_coord, ivec2( 1, 0)).x * 0.125;
	ret += shadow2DArrayOffset(stex, shadow_coord, ivec2( 1, 1)).x * 0.0625;
	
	return ret;
}

void main(void)
{
	vec4 light_col;
	vec3 N = normalize(vNrm);
	//vec3 L = gl_LightSource[0].position.xyz;
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// 環境光の計算
	//  - OpenGLが計算した光源強度と反射係数の積(gl_FrontLightProduct)を用いる．
	light_col = gl_FrontLightProduct[0].ambient;

	// 拡散反射の計算
	float diff = max(dot(L, N), 0.0);

	// 鏡面反射の計算
	if(diff > 0.0){
		// 反射ベクトルの計算
		vec3 V = normalize(-vPos.xyz);
		//vec3 R = 2.0*dot(N, L)*N-L;
		vec3 R = reflect(-L, N);
		float spec = pow(abs(dot(R, V)), gl_FrontMaterial.shininess);

		light_col += gl_FrontLightProduct[0].diffuse*diff+
					 gl_FrontLightProduct[0].specular*spec;
	}

	const float shadow_ambient = 0.8;
	vec4 color_tex = texture2D(tex, gl_TexCoord[0].st);
	float shadow_coef = ShadowCoef();

	gl_FragColor = shadow_ambient*shadow_coef*light_col+(1.0-shadow_ambient)*light_col;
	//gl_FragColor = shadow_ambient*shadow_coef*gl_Color+(1.0-shadow_ambient)*gl_Color;
}
);










//-----------------------------------------------------------------------------
// Shadow View シェーダ
//-----------------------------------------------------------------------------
const char *shadow_view_vs = RXSTR(
void main(void)
{
	gl_TexCoord[0] = vec4(0.5)*gl_Vertex + vec4(0.5);
	gl_Position = gl_Vertex;
}
);

const char *shadow_view_fs = RXSTR(
#version 110 @
#extension GL_EXT_texture_array : enable @

uniform sampler2DArray tex;
uniform float layer;

void main(void)
{
	vec4 tex_coord = vec4(gl_TexCoord[0].x, gl_TexCoord[0].y, layer, 1.0);
	gl_FragColor = texture2DArray(tex, tex_coord.xyz);
//	gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
);


//-----------------------------------------------------------------------------
// MARK:Kajiya-kai シェーダ
//-----------------------------------------------------------------------------

//! Kajiya-kaiモデル 頂点シェーダ
const char *vertexShader = RXSTR(
varying vec4 vPos;
varying vec3 vNrm;
void main(void)
{
	vPos = gl_ModelViewMatrix*gl_Vertex;
	vNrm = normalize(gl_NormalMatrix*gl_Normal);

	gl_Position = ftransform();
	gl_TexCoord[0] = gl_TextureMatrix[0] * gl_ModelViewMatrix * gl_Vertex;

	// 視点->頂点ベクトル
	vec3 E = normalize(vPos.xyz);
	vec3 L = normalize(gl_LightSource[0].position.xyz-vPos.xyz);	// 光源ベクトル

	// Kajiya-kayモデル
	vec3 nl = cross(vNrm, L);		// N×L
	float nnl = sqrt(dot(nl, nl));	// |N×L| = sinθ

	vec3 ne = cross(vNrm, E);		// N×E
	float nne = sqrt(dot(ne, ne));	// |N×E| = sinφ

	float dnl = dot(vNrm, L);
	float dne = dot(vNrm, E);

	float spec = dne*dnl+nne*nnl;
	//spec *= spec;	// 2乗
	//spec *= spec;	// 4乗
	//spec *= spec;	// 8乗
	spec = pow(max(spec, 0.0), 80.0f);
	
	// Kajiya-kay拡散項
	vec3 Ad = gl_FrontMaterial.diffuse.xyz*nnl;		// Kd sinθ = Kd |n×l|
	vec3 As = gl_FrontMaterial.specular.xyz*spec;	// Ks (cosγ)^n = Ks(cosφcosθ-sinφsinθ)^n

	// ランバート拡散
	float diff = max(0.0, dot(L, vNrm));
	vec3 Ld = gl_FrontLightProduct[0].diffuse.xyz*diff;

	gl_FrontColor.rgb = Ad+As+Ld;

	//gl_FrontColor.rgb = 0.5 * gl_Normal.xyz + 0.5;
	gl_FrontColor.a = 1.0;
}
);

//! Kajiya-kaiモデル ピクセルシェーダ
const char *pixelShader = RXSTR(
varying vec4 vPos;
varying vec3 vNrm;
void main(void)
{
	gl_FragColor = gl_Color;
}
);




//-----------------------------------------------------------------------------
// MARK:toonシェーダ
//-----------------------------------------------------------------------------

//! トゥーンレンダリング 頂点シェーダ
const char *toon_vs = RXSTR(
void main(void)
{
    // 位置座標を座標変換
    gl_Position = ftransform();
        
    // 法線と光源ベクトルとの内積(平行光源用)
    vec3 normal = normalize(gl_NormalMatrix * gl_Normal);
    vec3 light = normalize(gl_LightSource[0].position.xyz);
    float lgtdot = dot(light, normal);

    // 法線と視線ベクトルとの内積
    vec3 eye = - normalize(vec3(gl_ModelViewMatrix * gl_Vertex));
    float eyedot = dot(eye, normal);
    
    // テクスチャ座標に割り当てる
    gl_TexCoord[0].s = lgtdot;
    gl_TexCoord[0].t = eyedot;
}
);

//! トゥーンレンダリング ピクセルシェーダ
const char *toon_fs = RXSTR(
uniform sampler2D toontex;
void main(void)
{
	vec4 color = texture2D(toontex, gl_TexCoord[0].st);
	gl_FragColor = color;
}
);






//-----------------------------------------------------------------------------
// MARK:GLSLコンパイル
//-----------------------------------------------------------------------------
/*!
 * GLSLプログラムのコンパイル
 * @param[in] vsource vertexシェーダプログラム内容
 * @param[in] fsource pixel(fragment)シェーダプログラム内容
 * @return GLSLプログラム番号
 */
static GLuint CompileProgram(const char *vsource, const char *fsource)
{
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(vertexShader, 1, &vsource, 0);
	glShaderSource(fragmentShader, 1, &fsource, 0);
	
	glCompileShader(vertexShader);
	glCompileShader(fragmentShader);

	GLuint program = glCreateProgram();

	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);

	glLinkProgram(program);

	// check if program linked
	GLint success = 0;
	glGetProgramiv(program, GL_LINK_STATUS, &success);

	if (!success) {
		char temp[256];
		glGetProgramInfoLog(program, 256, 0, temp);
		printf("Failed to link program:\n%s\n", temp);
		glDeleteProgram(program);
		program = 0;
	}

	return program;
}

/*!
 * GLSLシェーダコンパイル
 * @param[in] target ターゲット(GL_VERTEX_SHADER,GL_FRAGMENT_SHADER)
 * @param[in] shader シェーダコード
 * @return GLSLオブジェクト
 */
inline GLuint CompileGLSLShader(GLenum target, const char* shader)
{
	// GLSLオブジェクト作成
	GLuint object = glCreateShader(target);

	if(!object) return 0;

	glShaderSource(object, 1, &shader, NULL);
	glCompileShader(object);

	// コンパイル状態の確認
	GLint compiled = 0;
	glGetShaderiv(object, GL_COMPILE_STATUS, &compiled);

	if(!compiled){
		char temp[256] = "";
		glGetShaderInfoLog( object, 256, NULL, temp);
		fprintf(stderr, " Compile failed:\n%s\n", temp);

		glDeleteShader(object);
		return 0;
	}

	return object;
}

/*!
 * GLSLシェーダコンパイル
 * @param[in] target ターゲット(GL_VERTEX_SHADER,GL_FRAGMENT_SHADER)
 * @param[in] fn シェーダファイルパス
 * @return GLSLオブジェクト
 */
inline GLuint CompileGLSLShaderFromFile(GLenum target, const char* fn)
{
	FILE *fp;

	// バイナリとしてファイル読み込み
	fp = fopen(fn, "rb");
	if(fp == NULL) return 0;

	// ファイルの末尾に移動し現在位置(ファイルサイズ)を取得
	fseek(fp, 0, SEEK_END);
	long size = ftell(fp);

	fseek(fp, 0, SEEK_SET);

	// シェーダの内容格納
	char *text = new char[size+1];
	fread(text, size, 1, fp);
	text[size] = '\0';

	//printf("%s\n", text);


	fclose(fp);

	// シェーダコンパイル
	printf("Compile %s\n", fn);
	GLuint object = CompileGLSLShader(target, text);

	delete [] text;

	return object;
}

/*!
 * バーテックスとフラグメントシェーダで構成されるGLSLプログラム作成
 * @param[in] vs バーテックスシェーダオブジェクト
 * @param[in] fs フラグメントシェーダオブジェクト
 * @return GLSLプログラムオブジェクト
 */
inline GLuint LinkGLSLProgram(GLuint vs, GLuint fs)
{
	// プログラムオブジェクト作成
	GLuint program = glCreateProgram();

	// シェーダオブジェクトを登録
	glAttachShader(program, vs);
	glAttachShader(program, fs);

	// プログラムのリンク
	glLinkProgram(program);

	// エラー出力
	GLint charsWritten, infoLogLength;
	glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogLength);

	char * infoLog = new char[infoLogLength];
	glGetProgramInfoLog(program, infoLogLength, &charsWritten, infoLog);
	printf(infoLog);
	delete [] infoLog;

	// リンカテスト
	GLint linkSucceed = GL_FALSE;
	glGetProgramiv(program, GL_LINK_STATUS, &linkSucceed);
	if(linkSucceed == GL_FALSE){
		glDeleteProgram(program);
		return 0;
	}

	return program;
}


/*!
 * バーテックス/ジオメトリ/フラグメントシェーダで構成されるGLSLプログラム作成
 * @param[in] vs バーテックスシェーダオブジェクト
 * @param[in] gs ジオメトリシェーダオブジェクト
 * @param[in] inputType ジオメトリシェーダへの入力タイプ
 * @param[in] vertexOut バーテックスの出力
 * @param[in] outputType ジオメトリシェーダからの出力タイプ
 * @param[in] fs フラグメントシェーダオブジェクト
 * @return GLSLプログラムオブジェクト
 */
inline GLuint LinkGLSLProgram(GLuint vs, GLuint gs, GLint inputType, GLint vertexOut, GLint outputType, GLuint fs)
{
	// プログラムオブジェクト作成
	GLuint program = glCreateProgram();

	// シェーダオブジェクトを登録
	glAttachShader(program, vs);
	glAttachShader(program, gs);

	glProgramParameteriEXT(program, GL_GEOMETRY_INPUT_TYPE_EXT, inputType);
	glProgramParameteriEXT(program, GL_GEOMETRY_VERTICES_OUT_EXT, vertexOut);
	glProgramParameteriEXT(program, GL_GEOMETRY_OUTPUT_TYPE_EXT, outputType);
	glAttachShader(program, fs);

	// プログラムのリンク
	glLinkProgram(program);

	// エラー出力
	GLint charsWritten, infoLogLength;
	glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogLength);

	char * infoLog = new char[infoLogLength];
	glGetProgramInfoLog(program, infoLogLength, &charsWritten, infoLog);
	printf(infoLog);
	delete [] infoLog;

	// リンカテスト
	GLint linkSucceed = GL_FALSE;
	glGetProgramiv(program, GL_LINK_STATUS, &linkSucceed);
	if(linkSucceed == GL_FALSE){
		glDeleteProgram(program);
		return 0;
	}

	return program;
}


/*!
 * GLSLのコンパイル・リンク(ファイルより)
 * @param[in] vs 頂点シェーダファイルパス
 * @param[in] fs フラグメントシェーダファイルパス
 * @param[in] name プログラム名
 * @return GLSLオブジェクト
 */
inline rxGLSL CreateGLSLFromFile(const string &vs, const string &fs, const string &name)
{
	rxGLSL gs;
	gs.VertProg = vs;
	gs.FragProg = fs;
	gs.Name = name;

	GLuint v, f;
	if(!(v = CompileGLSLShaderFromFile(GL_VERTEX_SHADER, vs.c_str()))){
		// skip the first three chars to deal with path differences
		v = CompileGLSLShaderFromFile(GL_VERTEX_SHADER, &vs.c_str()[3]);
	}

	if(!(f = CompileGLSLShaderFromFile(GL_FRAGMENT_SHADER, fs.c_str()))){
		f = CompileGLSLShaderFromFile(GL_FRAGMENT_SHADER, &fs.c_str()[3]);
	}

	gs.Prog = LinkGLSLProgram(v, f);
	//gs.Prog = GLSL_CreateShaders(gs.VertProg.c_str(), gs.FragProg.c_str());

	return gs;
}

/*!
 * #versionなどのプリプロセッサを文字列として書かれたシェーダ中に含む場合，改行がうまくいかないので，
 *  #version 110 @ のように最後に@を付け，改行に変換する
 * @param[in] s  シェーダ文字列
 * @param[in] vs 変換後のシェーダ文字列
 */
inline void CreateGLSLShaderString(const char* s, vector<char> &vs)
{
	int idx = 0;
	char c = s[0];
	while(c != '\0'){
		if(c == '@') c = '\n'; // #versionなどを可能にするために@を改行に変換

		vs.push_back(c);
		idx++;
		c = s[idx];
	}
	vs.push_back('\0');
}

/*!
 * GLSLのコンパイル・リンク(文字列より)
 * @param[in] vs 頂点シェーダ内容
 * @param[in] fs フラグメントシェーダ内容
 * @param[in] name プログラム名
 * @return GLSLオブジェクト
 */
inline rxGLSL CreateGLSL(const char* vs, const char* fs, const string &name)
{
	rxGLSL gs;
	gs.VertProg = "from char";
	gs.FragProg = "from char";
	gs.Name = name;

	vector<char> vs1, fs1;
	CreateGLSLShaderString(vs, vs1);
	CreateGLSLShaderString(fs, fs1);
	
	//printf("vertex shader : %d\n%s\n", vs1.size(), &vs1[0]);
	//printf("pixel shader  : %d\n%s\n", fs1.size(), &fs1[0]);

	GLuint v, f;
	v = CompileGLSLShader(GL_VERTEX_SHADER, &vs1[0]);
	f = CompileGLSLShader(GL_FRAGMENT_SHADER, &fs1[0]);
	gs.Prog = LinkGLSLProgram(v, f);

	return gs;
}