/*! 
  @file bt_simple.cpp
	
  @brief Bulletによる物理シミュレーション
		 シンプルな剛体落下
 
  @author Makoto Fujisawa
  @date 2013-03
*/
#pragma comment(lib, "glew32.lib")

#pragma comment(lib, "LinearMath.lib")
#pragma comment(lib, "BulletCollision.lib")
#pragma comment(lib, "BulletDynamics.lib")

#pragma comment(lib, "BulletSoftBody.lib")

//-----------------------------------------------------------------------------
// インクルードファイル
//-----------------------------------------------------------------------------
#include "utils.h"
//-----------------------------------------------------------------------------
// MARK:定義/定数
//-----------------------------------------------------------------------------
const GLfloat RX_LIGHT0_POS[4] = {2.0f, 4.0f, 1.0f, 0.0f};
const GLfloat RX_LIGHT1_POS[4] = {-1.0f, -10.0f, -1.0f, 0.0f};

const GLfloat RX_LIGHT_DIFF[4] = {1.0f, 1.0f, 1.0f, 1.0f};
const GLfloat RX_LIGHT_SPEC[4] = {0.7f, 0.6f, 0.6f, 1.0f};
const GLfloat RX_LIGHT_AMBI[4] = {0.3f, 0.3f, 0.3f, 1.0f};

const GLfloat RX_FOV = 45.0f;

const double DT = 0.01; //!< 時間ステップ幅Δt

//-----------------------------------------------------------------------------
// MARK:グローバル変数
//-----------------------------------------------------------------------------
//! ウィンドウサイズ
int g_iWinW = 720;
int g_iWinH = 720;
int g_iMouseButton = -1; //!< マウスボタンの状態

bool g_bIdle; //!< アニメーションフラグ

#define BALL_Z 8

//判断のためのグローバル変数
int ground = 1;
int goal = 0;
int ball = 0;
int pointer = 0;
int error = 0;
int cursor = 0;
int wall = 0;
int goal_message = 0;
int table = 0;
int judge_rl = 0;
int judge_fb = 0;
int line = 0;

//視界の回転のための変数
double q[4] = {1, 0, 0, 0};
double q_reset[4] = {1, 0, 0, 0};
float sin_b = 0;
float cos_b = 1;
int eye;
float span = 0.1;
float pointer_span = 0.02;
float view_updown = -7.0 - BALL_Z;
float view_rl = 0.0;

//サッカーゴール
const btScalar GOAL_HEIGHT_HALF = 1.0;
const btScalar GOAL_WIDTH_HALF = 3.0;
const btScalar GOAL_DEPTH_HALF = 0.8;
const btScalar GOAL_THICK = 0.05;
const btScalar GOAL_BACK_POSITION = -12;
const btScalar GOAL_CENTER = GOAL_BACK_POSITION + GOAL_DEPTH_HALF * 2;

//サッカーボールを動かすためのグローバル変数
btRigidBody *soccer_ball;
btRigidBody *Cursor;
float rotation = 0;
btRigidBody *Wall;
btRigidBody *Pointer;
btVector3 power;
btVector3 ball_pos(0, 0.1, BALL_Z);
float rotation_v[2];
float pointer_pos[2] = {ball_pos[0], ball_pos[2] + 1};
float pointer_origin_pos[2];
float ball_to_goal[2] = {0 - ball_pos[0], GOAL_CENTER - ball_pos[2]};
float fire_v[2];
float to_goal;

//エラーメッセージ
std::string error_message;

//固定された物体を動かすためのインスタンスを記録しておくグローバル変数
btRigidBody *g_pMoveBody;

//! トラックボール(マウスによる視点回転用)
rxTrackball g_tbView;

// シャドウマッピング(影付け用)
rxShadowMap g_ShadowMap;
int g_iShadowMapSize = 512;

// Bullet
btDynamicsWorld *g_pDynamicsWorld;							 //!< Bulletワールド
btAlignedObjectArray<btCollisionShape *> g_vCollisionShapes; //!< 剛体オブジェクトの形状を格納する動的配列
int cube_number = 1;
// マウスピック
btVector3 g_vPickPos;
btRigidBody *g_pPickBody = 0;
btPoint2PointConstraint *m_pPickConstraint = 0;
btSoftBody::Node *g_pPickNode = 0;
double g_fPickDist = 0.0;

//-----------------------------------------------------------------------------
// MARK:Bullet用関数
//-----------------------------------------------------------------------------
/*!
 * Bullet剛体(btRigidBody)の作成
 * @param[in] mass 質量
 * @param[in] init_tras 初期位置・姿勢
 * @param[in] shape 形状
 * @param[in] index オブジェクト固有の番号
 * @return 作成したbtRigidBody
 */
btRigidBody *CreateRigidBody(double mass, const btTransform &init_trans, btCollisionShape *shape, int index)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	// 質量が0ならば静的な(static)オブジェクトとして設定，
	bool isDynamic = (mass != 0.0);
	//cout << isDynamic << endl;

	// 形状から慣性テンソルを計算
	btVector3 inertia(0, 0, 0);
	if (isDynamic)
		//cout << "isDynamic = " << isDynamic << endl;
		shape->calculateLocalInertia(mass, inertia);

	// 初期位置，姿勢の設定
	btDefaultMotionState *motion_state = new btDefaultMotionState(init_trans);

	// 質量，慣性テンソル(回転しやすさ)，形状，位置・姿勢情報を一つの変数にまとめる
	btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shape, inertia);

	// 剛体オブジェクト(btRigidBody)の生成
	btRigidBody *body = new btRigidBody(rb_info);

	// 剛体オブジェクトに番号を設定
	body->setUserIndex(index);

	// 衝突応答のためのグループ
	enum CollisionGroup
	{
		RX_COL_NOTHING = 0, // 0000
		RX_COL_GROUND = 1,	// 0001
		RX_COL_GROUP1 = 2,	// 0010
		RX_COL_GROUP2 = 4,	// 0100
		RX_COL_GROUP3 = 8,
		RX_COL_GROUP4 = 16,
		RX_COL_GROUP5 = 32,
		RX_COL_GROUP6 = 64
	};

	//反発力・摩擦力の設定
	btScalar restitution = 0.8;		   //!< 反発係数
	btScalar friction = 0.5;		   //!< 摩擦係数
	body->setRestitution(restitution); //反発力
	body->setFriction(friction);	   //摩擦力

	// Bulletワールドに剛体オブジェクトを追加
	if (ground == 1)
	{
		g_pDynamicsWorld->addRigidBody(body, RX_COL_GROUND, RX_COL_GROUP1 | RX_COL_GROUP2 | RX_COL_GROUP3 | RX_COL_GROUP6);
		//ground = 0;
	}
	else if (ball == 1)
	{
		g_pDynamicsWorld->addRigidBody(body, RX_COL_GROUP1, RX_COL_GROUND | RX_COL_GROUP2);
		ball = 0;
	}
	else if (goal == 1)
	{
		g_pDynamicsWorld->addRigidBody(body, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1);
	}
	else if (pointer == 1)
	{
		g_pDynamicsWorld->addRigidBody(body, RX_COL_GROUP3, RX_COL_GROUND | RX_COL_GROUP4);
		pointer = 0;
	}
	else if (table == 1)
	{
		g_pDynamicsWorld->addRigidBody(body, RX_COL_GROUP4, RX_COL_GROUP3);
		table = 0;
	}
	else if (cursor == 1)
	{
		g_pDynamicsWorld->addRigidBody(body, RX_COL_GROUP5, RX_COL_GROUP5);
	}
	else if (line == 1)
	{
		g_pDynamicsWorld->addRigidBody(body, RX_COL_GROUP6, RX_COL_GROUND);
	}
	//g_pDynamicsWorld->addRigidBody(body);
	return body;
}

void DrawString(std::string str, int w, int h, int x0, int y0)
{
	//cout << "drawstring" << endl;
	glDisable(GL_LIGHTING);
	// 平行投影にする
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, w, h, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// 画面上にテキスト描画
	glRasterPos2f(x0, y0);
	int size = (int)str.size();
	for (int i = 0; i < size; ++i)
	{
		char ic = str[i];
		if (goal_message != 0)
		{
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, ic);
		}
		else
		{
			glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ic);
		}
	}

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

/*!
 * 剛体オブジェクトの追加
 */
void SetRigidBodies(void)
{

	btTransform trans;	 // 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity(); // 位置姿勢行列の初期化

	const btScalar CUBE_HALF_EXTENTS = 0.2; // 立方体の変の長さの半分(中心から辺までの距離)
	const btScalar GROUND_HEIGHT = 0.0;		// 地面の高さ

	// ----- 地面(質量0のx-z平面上で平べったい直方体で表現)の追加 -----

	btCollisionShape *ground_shape = new btBoxShape(btVector3(36, CUBE_HALF_EXTENTS, 36)); // 形状
	trans.setOrigin(btVector3(0, GROUND_HEIGHT - CUBE_HALF_EXTENTS, 0));				   // 上の面がy=0になるように設定

	// 剛体オブジェクト(Static)生成
	ground = 1;
	btRigidBody *body0 = CreateRigidBody(0.0, trans, ground_shape, 0); // Body
	ground = 0;
	g_vCollisionShapes.push_back(ground_shape); // 最後に破棄(delete)するために形状データを格納しておく
	// ----- ここまで (地面の追加) -----

	// ----- サッカーゴールオブジェクト追加 -----

	// 形状設定
	btCollisionShape *goal_back = new btBoxShape(btVector3(GOAL_WIDTH_HALF, GOAL_HEIGHT_HALF, GOAL_THICK));
	btCollisionShape *goal_side = new btBoxShape(btVector3(GOAL_THICK, GOAL_HEIGHT_HALF, GOAL_DEPTH_HALF));
	btCollisionShape *goal_roof = new btBoxShape(btVector3(GOAL_WIDTH_HALF, GOAL_THICK, GOAL_DEPTH_HALF));
	// 初期位置・姿勢
	btQuaternion qrot(0, 0, 0, 1);
	trans.setIdentity(); // 位置姿勢行列の初期化

	goal = 1;

	//ゴールの背面の追加
	trans.setOrigin(btVector3(0, GROUND_HEIGHT + GOAL_HEIGHT_HALF, GOAL_BACK_POSITION));
	trans.setRotation(qrot);
	btRigidBody *goal_b = CreateRigidBody(0.0, trans, goal_back, 0);

	//ゴール側面の追加
	trans.setOrigin(btVector3(-GOAL_WIDTH_HALF, GROUND_HEIGHT + GOAL_HEIGHT_HALF, GOAL_BACK_POSITION + GOAL_DEPTH_HALF));
	btRigidBody *goal_left = CreateRigidBody(0.0, trans, goal_side, 0);

	trans.setOrigin(btVector3(GOAL_WIDTH_HALF, GROUND_HEIGHT + GOAL_HEIGHT_HALF, GOAL_BACK_POSITION + GOAL_DEPTH_HALF));
	btRigidBody *goal_right = CreateRigidBody(0.0, trans, goal_side, 0);

	//ゴール天井の追加
	trans.setOrigin(btVector3(0, GROUND_HEIGHT + GOAL_HEIGHT_HALF * 2, GOAL_BACK_POSITION + GOAL_DEPTH_HALF));
	btRigidBody *goal_r = CreateRigidBody(0.0, trans, goal_roof, 0);

	goal = 0;

	g_vCollisionShapes.push_back(goal_back); // 最後に破棄(delete)するために形状データを格納しておく
	// ----- ここまで (サッカーゴールオブジェクト追加) -----

	// ----- ラインの追加 -----
	const btScalar GOAL_LINE_HALF = 9;
	const btScalar GOAL_AREA_VERTICAL_HALF = 2;
	const btScalar GOAL_AREA_SIDE_HALF = GOAL_WIDTH_HALF + GOAL_AREA_VERTICAL_HALF;

	const btScalar PENALTY_AREA_SIDE_HALF = GOAL_WIDTH_HALF + GOAL_AREA_VERTICAL_HALF * 3;
	const btScalar PENALTY_AREA_VERTICAL_HALF = GOAL_AREA_VERTICAL_HALF * 3;

	const btScalar PENALTY_MARK_R = 0.1;

	//形状設定
	btCollisionShape *goal_side_line_shape = new btBoxShape(btVector3(GOAL_AREA_SIDE_HALF, 0.01, 0.1));
	btCollisionShape *goal_line_shape = new btBoxShape(btVector3(GOAL_LINE_HALF, 0.01, 0.1));
	btCollisionShape *goal_vertical_line_shape = new btBoxShape(btVector3(0.1, 0.01, GOAL_AREA_VERTICAL_HALF));
	btCollisionShape *penalty_side_line_shape = new btBoxShape(btVector3(PENALTY_AREA_SIDE_HALF, 0.01, 0.1));
	btCollisionShape *penalty_vertical_line_shape = new btBoxShape(btVector3(0.1, 0.01, PENALTY_AREA_VERTICAL_HALF));
	btCylinderShape *penalty_mark_shape = new btCylinderShape(btVector3(PENALTY_MARK_R, 0.01, PENALTY_MARK_R));

	line = 1;
	trans.setOrigin(btVector3(0, 0.01 / 2, GOAL_CENTER + GOAL_AREA_VERTICAL_HALF * 2));
	btRigidBody *goal_line_side = CreateRigidBody(1, trans, goal_side_line_shape, 0);

	trans.setOrigin(btVector3(-GOAL_AREA_SIDE_HALF, 0.01 / 2, GOAL_CENTER + GOAL_AREA_VERTICAL_HALF));
	btRigidBody *goal_line_vertical1 = CreateRigidBody(1, trans, goal_vertical_line_shape, 0);

	trans.setOrigin(btVector3(GOAL_AREA_SIDE_HALF, 0.01 / 2, GOAL_CENTER + GOAL_AREA_VERTICAL_HALF));
	btRigidBody *goal_line_vertical2 = CreateRigidBody(1, trans, goal_vertical_line_shape, 0);

	trans.setOrigin(btVector3(0, 0.01 / 2, GOAL_CENTER));
	btRigidBody *goal_line = CreateRigidBody(1, trans, goal_line_shape, 0);

	trans.setOrigin(btVector3(-GOAL_LINE_HALF * 2, 0.01 / 2, GOAL_CENTER));
	btRigidBody *goal_line2 = CreateRigidBody(1, trans, goal_line_shape, 0);

	trans.setOrigin(btVector3(GOAL_LINE_HALF * 2, 0.01 / 2, GOAL_CENTER));
	btRigidBody *goal_line3 = CreateRigidBody(1, trans, goal_line_shape, 0);

	trans.setOrigin(btVector3(0, 0.01 / 2, GOAL_CENTER + PENALTY_AREA_VERTICAL_HALF * 2));
	btRigidBody *penalty_line_side = CreateRigidBody(1, trans, penalty_side_line_shape, 0);

	trans.setOrigin(btVector3(-PENALTY_AREA_SIDE_HALF, 0.01 / 2, GOAL_CENTER + PENALTY_AREA_VERTICAL_HALF));
	btRigidBody *penalty_line_vertical1 = CreateRigidBody(1, trans, penalty_vertical_line_shape, 0);

	trans.setOrigin(btVector3(PENALTY_AREA_SIDE_HALF, 0.01 / 2, GOAL_CENTER + PENALTY_AREA_VERTICAL_HALF));
	btRigidBody *penalty_line_vertical2 = CreateRigidBody(1, trans, penalty_vertical_line_shape, 0);

	trans.setOrigin(btVector3(0, 0.01 / 2, GOAL_CENTER + GOAL_AREA_VERTICAL_HALF * 4));
	btRigidBody *penalty_mark = CreateRigidBody(1, trans, penalty_mark_shape, 0);

	line = 0;

	g_vCollisionShapes.push_back(goal_side_line_shape); // 最後に破棄(delete)するために形状データを格納しておく
	g_vCollisionShapes.push_back(goal_line_shape);
	g_vCollisionShapes.push_back(goal_vertical_line_shape);
	g_vCollisionShapes.push_back(penalty_side_line_shape);
	g_vCollisionShapes.push_back(penalty_vertical_line_shape);
	g_vCollisionShapes.push_back(penalty_mark_shape);

	// ----- ここまで (ラインの追加) -----
}

void SetCursor(void)
{
	//----- カーソルの追加 -----
	cursor = 1;
	btTransform trans;	 // 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity(); // 位置姿勢行列の初期化
	//形状設定
	const btScalar BALL_R = 0.1;
	btCollisionShape *cursor_shape = new btSphereShape(BALL_R);

	//初期位置・姿勢
	btQuaternion qrot(0, 0, 0, 1);

	trans.setOrigin(btVector3(ball_pos[0], BALL_R, ball_pos[2]));
	trans.setRotation(qrot);

	Cursor = CreateRigidBody(0, trans, cursor_shape, 0);
	g_vCollisionShapes.push_back(cursor_shape);
	//----- ここまで（カーソルの追加）-----
}


void SetSoccerball(void)
{
	ball = 1;
	btTransform trans;	 // 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity(); // 位置姿勢行列の初期化

	//形状設定
	const btScalar BALL_R = 0.1;
	btCollisionShape *soccer_ball_shape = new btSphereShape(BALL_R);

	//初期位置・姿勢
	btQuaternion qrot(0, 0, 0, 1);

	trans.setOrigin(ball_pos);
	trans.setRotation(qrot);

	soccer_ball = CreateRigidBody(1, trans, soccer_ball_shape, 0);

	soccer_ball->setCcdMotionThreshold(BALL_R);
	soccer_ball->setCcdSweptSphereRadius(0.05 * BALL_R);

	g_vCollisionShapes.push_back(soccer_ball_shape);
}

void SetWall(void)
{
	wall = 1;
	btTransform trans;	 // 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity(); // 位置姿勢行列の初期化

	const btScalar WALL_HEIGHT_HALF = 0.7;
	const btScalar WALL_WIDTH_HALF = 1.0;
	const btScalar WALL_DEPTH_HALF = 0.1;

	btCollisionShape *wall_shape = new btBoxShape(btVector3(WALL_WIDTH_HALF, WALL_HEIGHT_HALF, WALL_DEPTH_HALF));

	btVector3 wall_pos = ball_pos;

	float y;
	if (ball_pos[0] < 0)
	{
		y = sqrt((1 - cos_b) / 2);
	}
	else if (ball_pos[0] > 0)
	{
		y = -sqrt((1 - cos_b) / 2);
	}
	else
	{
		y = 0;
		if (cos_b < 0)
		{
			cos_b *= -1;
		}
		cout << sqrt((1 + cos_b) / 2) << endl;
	}

	btQuaternion qrot(0, y, 0, sqrt((1 + cos_b) / 2));
	
	trans.setOrigin(btVector3(pointer_origin_pos[0] + 5.5 * sin_b, WALL_HEIGHT_HALF, pointer_origin_pos[1] - 5.5 * abs(cos_b)));
	trans.setRotation(qrot);

	Wall = CreateRigidBody(0, trans, wall_shape, 0);

	g_vCollisionShapes.push_back(wall_shape);
}

void SetPointer(void)
{
	pointer = 1;

	btTransform trans;
	trans.setIdentity();

	const btScalar POINTER_HALF_EXTENTS = 0.03;
	const btScalar FENCE_HALF_EXTENTS = 0.5;

	btCollisionShape *pointer_shape = new btBoxShape(btVector3(POINTER_HALF_EXTENTS, POINTER_HALF_EXTENTS * 2, POINTER_HALF_EXTENTS));

	pointer_origin_pos[0] = pointer_pos[0];
	pointer_origin_pos[1] = pointer_pos[1];

	float y;
	if (ball_pos[0] < 0)
	{
		y = sqrt((1 - cos_b) / 2);
	}
	else if (ball_pos[0] > 0)
	{
		y = -sqrt((1 - cos_b) / 2);
	}
	else
	{
		y = 0;
		if (cos_b < 0)
		{
			cos_b *= -1;
		}
	}

	btQuaternion qrot(0, y, 0, sqrt((1 + cos_b) / 2));

	trans.setOrigin(btVector3(pointer_pos[0], POINTER_HALF_EXTENTS, pointer_pos[1]));
	trans.setRotation(qrot);

	Pointer = CreateRigidBody(0.0, trans, pointer_shape, 0);
	pointer = 1;
	btRigidBody *origin = CreateRigidBody(0.0, trans, pointer_shape, 0);

	const btScalar CYLINDER_R = 0.46;
	const btScalar CYLINDER_H = 0.01;

	btCylinderShape *cylinder_shape = new btCylinderShape(btVector3(CYLINDER_R, CYLINDER_H, CYLINDER_R)); //半径,高さ

	trans.setOrigin(btVector3(pointer_pos[0], CYLINDER_H, pointer_pos[1]));

	table = 1;
	btRigidBody *Table = CreateRigidBody(2, trans, cylinder_shape, 0);

	g_vCollisionShapes.push_back(pointer_shape);
	g_vCollisionShapes.push_back(cylinder_shape);
}

/*!
 * Bullet初期化
 */
void InitBullet(void)
{
	// 衝突検出方法の選択(デフォルトを選択)
	btDefaultCollisionConfiguration *config = new btDefaultCollisionConfiguration();
	btCollisionDispatcher *dispatcher = new btCollisionDispatcher(config);

	// ブロードフェーズ法の設定
	//btDbvtBroadphase *broadphase = new btDbvtBroadphase();//(Dynamic AABB tree method)
	btAxisSweep3 *broadphase = new btAxisSweep3(btVector3(-10, -10, -10), btVector3(10, 10, 10));

	// 拘束(剛体間リンク)のソルバ設定
	btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver();

	// Bulletのワールド作成
	g_pDynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);

	// 重力加速度の設定(OpenGLに合わせてy軸方向を上下方向にする)
	g_pDynamicsWorld->setGravity(btVector3(0, -9.8, 0));

	SetRigidBodies();

	if (cursor == 0)
	{
		SetCursor();
	}
}

/*!
 * 設定したBulletの剛体オブジェクト，ワールドの破棄
 */
void CleanBullet(void)
{
	//Constraintの削除
	for (int i = g_pDynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
	{
		btTypedConstraint *constraint = g_pDynamicsWorld->getConstraint(i);
		g_pDynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}

	// 剛体オブジェクトの破棄
	for (int i = g_pDynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i)
	{
		btCollisionObject *obj = g_pDynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody *body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		g_pDynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	g_pMoveBody = 0;
	soccer_ball = 0;
	Cursor = 0;
	Wall = 0;

	// 形状の破棄
	for (int j = 0; j < (int)g_vCollisionShapes.size(); ++j)
	{
		btCollisionShape *shape = g_vCollisionShapes[j];
		g_vCollisionShapes[j] = 0;
		delete shape;
	}
	g_vCollisionShapes.clear();

	// ワールド破棄
	delete g_pDynamicsWorld;
}

//-----------------------------------------------------------------------------
// MARK:描画関数
//-----------------------------------------------------------------------------
/*!
 * 透視投影変換
 */
void Projection(void)
{
	gluPerspective(RX_FOV, (double)g_iWinW / (double)g_iWinH, 0.2f, 1000.0f);
}

/*!
 * Bulletのオブジェクトの描画シーン描画
 */
void DrawBulletObjects(void)
{
	static const GLfloat difr[] = {1.0, 0.4, 0.4, 1.0}; // 拡散色 : 赤
	static const GLfloat difg[] = {0.4, 0.6, 0.4, 1.0}; // 拡散色 : 緑
	static const GLfloat difb[] = {0.4, 0.4, 1.0, 1.0}; // 拡散色 : 青
	static const GLfloat difw[] = {1.0, 1.0, 1.0, 1.0};
	static const GLfloat spec[] = {0.3, 0.3, 0.3, 1.0}; // 鏡面反射色
	static const GLfloat ambi[] = {0.1, 0.1, 0.1, 1.0}; // 環境光

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glDisable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambi);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.0, 0.0, 1.0);

	glDisable(GL_CULL_FACE);

	if (g_pDynamicsWorld)
	{
		btScalar m[16];
		btMatrix3x3 rot;
		rot.setIdentity();

		// Bulletワールドから剛体オブジェクト情報を取得してOpenGLで描画
		const int n = g_pDynamicsWorld->getNumCollisionObjects(); // オブジェクト数の取得
		//cout << "n = " << n << endl;

		for (int i = 0; i < n; ++i)
		{

			// btCollisionObject → btRigidBodyへのキャストで剛体オブジェクトを取得
			btCollisionObject *obj = g_pDynamicsWorld->getCollisionObjectArray()[i];

			// 形状取得
			btCollisionShape *shape = obj->getCollisionShape();
			int shapetype = shape->getShapeType();

			if (shapetype == SOFTBODY_SHAPE_PROXYTYPE)
			{
				btSoftBody *body = btSoftBody::upcast(obj);

				glMaterialfv(GL_FRONT, GL_DIFFUSE, difb);

				// draw a softbody
				DrawBulletSoftBody(body);
			}
			else
			{
				btRigidBody *body = btRigidBody::upcast(obj);
				if (body && body->getMotionState())
				{
					// btRigidBodyからMotion Stateを取得して，OpenGLの変換行列として位置・姿勢情報を得る
					btDefaultMotionState *ms = (btDefaultMotionState *)body->getMotionState();
					ms->m_graphicsWorldTrans.getOpenGLMatrix(m);
					rot = ms->m_graphicsWorldTrans.getBasis();
				}
				else
				{
					obj->getWorldTransform().getOpenGLMatrix(m);
					rot = obj->getWorldTransform().getBasis();
				}

				//cout << body->getMass() << endl;
				if (body && body->getMass() == 1.0)
				{
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difw);
				}
				else if (body && body->getInvMass() > RX_FEQ_EPS)
				{
					// Dynamicボディは青で描画
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difb);
				}
				else
				{ // Kinematicボディの場合は緑で描画
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difg);
				}

				btVector3 world_min, world_max;
				g_pDynamicsWorld->getBroadphase()->getBroadphaseAabb(world_min, world_max);

				glPushMatrix();
				glMultMatrixf(m);

				// 形状描画
				DrawBulletShape(shape, world_min, world_max);

				glPopMatrix();
			}
		}
	}
}

/*!
 * シーン描画
 */
void RenderScene(void *x = 0)
{
	// 光源設定
	glLightfv(GL_LIGHT0, GL_POSITION, RX_LIGHT0_POS);

	DrawBulletObjects();
}

/*!
 * 描画関数
 */
void Display(void)
{
	// フレームバッファとデプスバッファをクリアする
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	g_tbView.Apply(); // マウスによる回転・平行移動の適用

	// シャドウマップを使って影付きでオブジェクト描画
	Vec3 light_pos = Vec3(RX_LIGHT0_POS[0], RX_LIGHT0_POS[1], RX_LIGHT0_POS[2]);
	rxFrustum light = CalFrustum(90, 0.02, 50.0, g_iShadowMapSize, g_iShadowMapSize, light_pos, Vec3(0.0));
	g_ShadowMap.RenderSceneWithShadow(light, RenderScene, 0);

	glPushMatrix();
	if (goal_message == 0)
	{
		if (Pointer && soccer_ball)
		{
			glColor3d(0.0, 0.0, 1.0);
			double a = sqrt(power[0] * power[0] + power[1] * power[1] + power[2] * power[2]);
			//cout << a << endl;
			std::string str2;
			std::string str1 = "power : ";
			str2 = std::to_string(a);
			str1 += str2;
			DrawString(str1, g_iWinW, g_iWinH, 20, 20);

			//cout << rotation*100 << endl;
			double b = rotation * 100 / 2;
			//cout << b << endl;
			std::string str3 = "rotation(left : 'a'  right : 'd') : ";
			if (b <= -1)
			{
				//cout << "left" << endl;
				b = abs(b);
				//cout << b << endl;
				str3 += "left stage";
				str3 += std::to_string((int)b);
			}
			else if (b >= 1)
			{
				//cout << "right" << endl;
				str3 += "right stage";
				str3 += std::to_string((int)b);
			}
			else
			{
				str3 += "None";
			}

			DrawString(str3, g_iWinW, g_iWinH, 20, 40);
			DrawString("Wall( apper/disapper : 'k'  left : 'n'  right : 'm')", g_iWinW, g_iWinH, 20, 60);


			if (error == 1)
			{
				glColor3d(1.0, 0.0, 0.0);
				DrawString(error_message, g_iWinW, g_iWinH, 20, 80);
			}

			glColor3d(0.0, 0.0, 1.0);
			DrawString("'r' : reset", g_iWinW, g_iWinH, 500, 20);
			DrawString("'s' : fire", g_iWinW, g_iWinH, 500, 40);
		}
		else if (Cursor)
		{
			glColor3d(0.0, 0.0, 1.0);
			DrawString("First   : Use the left/right arrow keys to set the direction.", g_iWinW, g_iWinH, 20, 20);
			DrawString("Second  : Use the up/down arrow keys to set the distance.", g_iWinW, g_iWinH, 20, 40);
			DrawString("Finally : press the w key.", g_iWinW, g_iWinH, 20, 60);
			to_goal = sqrt(pow(ball_pos[0] ,2) + pow(ball_pos[2] - GOAL_CENTER,2)) * 5.5 / 4 ;
			DrawString("distance (m) : ", g_iWinW, g_iWinH, 20, 100);
			std::string distance = std::to_string(int(to_goal));
			DrawString(distance,  g_iWinW, g_iWinH, 160, 100);
		}
	}
	else if (goal_message == 1)
	{
		glColor3d(1.0, 0.0, 0.0);
		DrawString("GOAL!!!", g_iWinW, g_iWinH, 280, 300);
		DrawString("press 'r' to reset", g_iWinW, g_iWinH, 280, 324);
	}
	else if (goal_message == 2)
	{
		glColor3d(1.0, 0.0, 0.0);
		DrawString("NO GOAL....", g_iWinW, g_iWinH, 280, 300);
		DrawString("press 'r' to reset", g_iWinW, g_iWinH, 280, 324);
	}

	glPopMatrix();

	glPopMatrix();

	glutSwapBuffers();
}

/*!
 * リサイズイベント処理関数
 * @param[in] w,h キャンバスサイズ
 */
void Resize(int w, int h)
{
	glViewport(0, 0, w, h);
	g_tbView.SetRegion(w, h);

	g_iWinW = w;
	g_iWinH = h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(RX_FOV, (float)w / (float)h, 0.01f, 50.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/*!
 * マウスイベント処理関数
 * @param[in] button マウスボタン(GLUT_LEFT_BUTTON,GLUT_MIDDLE_BUTTON,GLUT_RIGHT_BUTTON)
 * @param[in] state マウスボタンの状態(GLUT_UP, GLUT_DOWN)
 * @param[in] x,y マウス座標(スクリーン座標系)
 */
void Mouse(int button, int state, int x, int y)
{
	if (x < 0 || y < 0)
		//cout << x << endl;
		return;
	g_iMouseButton = button;
	int mod = glutGetModifiers(); // SHIFT,CTRL,ALTの状態取得

	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			g_pPickNode = 0;
			double ray_from0[3], ray_to0[3];
			double init_pos[3] = {0, 0, 0};
			g_tbView.CalLocalPos(ray_from0, init_pos);

			g_tbView.GetRayTo(x, y, RX_FOV, ray_to0);

			btVector3 ray_from = btVector3(ray_from0[0], ray_from0[1], ray_from0[2]);
			btVector3 ray_to = btVector3(ray_to0[0], ray_to0[1], ray_to0[2]);

			btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from, ray_to);
			g_pDynamicsWorld->rayTest(ray_from, ray_to, ray_callback);

			if (ray_callback.hasHit())
			{
				const btCollisionObject *obj = ray_callback.m_collisionObject;

				// 光線と衝突した剛体
				btRigidBody *body = const_cast<btRigidBody *>(btRigidBody::upcast(obj));

				// 衝突点座標(ジョイントになる位置座標)
				btVector3 picked_pos = ray_callback.m_hitPointWorld;

				if (body)
				{
					if (!(body->isStaticObject() || body->isKinematicObject()))
					{
						g_pPickBody = body;
						g_vPickPos = picked_pos;

						// 選択された剛体の座標系でのピック位置
						btVector3 local_pos = body->getCenterOfMassTransform().inverse() * picked_pos;

						g_pPickBody->setActivationState(DISABLE_DEACTIVATION); // 必要！

						if (m_pPickConstraint)
						{
							g_pDynamicsWorld->removeConstraint(m_pPickConstraint);
							delete m_pPickConstraint;
						}
						m_pPickConstraint = new btPoint2PointConstraint(*body, local_pos);
						g_pDynamicsWorld->addConstraint(m_pPickConstraint, true);

						m_pPickConstraint->m_setting.m_impulseClamp = 30.0;
						m_pPickConstraint->m_setting.m_tau = 0.001f;

						g_fPickDist = (g_vPickPos - ray_from).length();
					}
				}
				else
				{
					// 光線と衝突したbtSoftBody
					btSoftBody *body = const_cast<btSoftBody *>(btSoftBody::upcast(obj));
					btSoftBody::sRayCast res;
					body->rayTest(ray_from, ray_to, res);
					if (res.fraction < 1.0)
					{
						btVector3 impact = ray_from + (ray_to - ray_from) * res.fraction;
						cout << impact << endl;
						if (res.feature == btSoftBody::eFeature::Face)
						{
							btSoftBody::Face &face = res.body->m_faces[res.index];

							// 衝突点に最も近いノードを探索
							btSoftBody::Node *node = face.m_n[0];
							for (int i = 1; i < 3; ++i)
							{
								if ((node->m_x - impact).length2() > (face.m_n[i]->m_x - impact).length2())
								{
									node = face.m_n[i];
								}
							}
							g_pPickNode = node;
							g_fPickDist = (g_pPickNode->m_x - ray_from).length();
						}
					}
				}
			}

			if (!m_pPickConstraint && !g_pPickNode)
			{
				g_tbView.Start(x, y, mod + 1);
			}
		}
		else if (state == GLUT_UP)
		{
			if (m_pPickConstraint)
			{
				g_pDynamicsWorld->removeConstraint(m_pPickConstraint);
				delete m_pPickConstraint;
				m_pPickConstraint = 0;
				g_pPickBody = 0;
			}
			else if (g_pPickNode)
			{
				g_pPickNode = 0;
			}
			else
			{
				g_tbView.Stop(x, y);
			}
		}
	}
}

/*!
 * モーションイベント処理関数(マウスボタンを押したままドラッグ)
 * @param[in] x,y マウス座標(スクリーン座標系)
 */
void Motion(int x, int y)
{
	if (g_iMouseButton == GLUT_LEFT_BUTTON)
	{
		if (m_pPickConstraint || g_pPickNode)
		{
			double ray_from0[3], ray_to0[3];
			double init_pos[3] = {0, 0, 0};
			g_tbView.CalLocalPos(ray_from0, init_pos);
			g_tbView.GetRayTo(x, y, RX_FOV, ray_to0);

			btVector3 ray_from = btVector3(ray_from0[0], ray_from0[1], ray_from0[2]);
			btVector3 new_ray_to = btVector3(ray_to0[0], ray_to0[1], ray_to0[2]);

			btVector3 dir = new_ray_to - ray_from;
			dir.normalize();

			btVector3 new_pivot = ray_from + dir * g_fPickDist;

			if (m_pPickConstraint)
			{
				m_pPickConstraint->setPivotB(new_pivot);
			}
			else if (g_pPickNode)
			{
				g_pPickNode->m_f += (new_pivot - g_pPickNode->m_x) * 10.0;
			}

			g_vPickPos = new_pivot;
		}
		else
		{
			g_tbView.Motion(x, y);
		}
	}
	glutPostRedisplay();
}

/*!
 * アイドルイベント処理関数(CPUが暇なときに実行)
 */
void Idle(void)
{
	glutPostRedisplay();
}

/*!
 * タイマーイベント処理関数(ある時間間隔で実行)
 */
void Timer(int value)
{

	if (g_bIdle && g_pDynamicsWorld)
	{
		// シミュレーションを1ステップ進める
		int numstep = g_pDynamicsWorld->stepSimulation(DT, 1);
	}
	if (soccer_ball)
	{
		btVector3 fire_rotation = btVector3(rotation * rotation_v[0], 0, rotation * rotation_v[1]);
		soccer_ball->applyCentralImpulse(fire_rotation);

		btTransform new_trans;
		soccer_ball->getMotionState()->getWorldTransform(new_trans);
		ball_pos = new_trans.getOrigin();
		if ((0 < ball_pos[1] && ball_pos[1] < 2) && (-3 < ball_pos[0] && ball_pos[0] < 3) && (GOAL_BACK_POSITION < ball_pos[2] && ball_pos[2] < GOAL_CENTER))
		{
			goal_message = 1;
			g_pDynamicsWorld->removeCollisionObject(soccer_ball);
			soccer_ball = 0;
		}
		else if (
			((ball_pos[2] < GOAL_BACK_POSITION) && (((ball_pos[0] < -GOAL_WIDTH_HALF) || (GOAL_WIDTH_HALF < ball_pos[0])) || (ball_pos[1] > 2 * GOAL_HEIGHT_HALF))))
		{
			goal_message = 2;
		}		
	}

	glutPostRedisplay();
	glutTimerFunc(DT * 1000, Timer, 0);
}

/*!
 * キーボードイベント処理関数
 * @param[in] key キーの種類
 * @param[in] x,y キーが押されたときのマウス座標(スクリーン座標系)
 */
void Keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '\033': // '\033' は ESC の ASCII コード
		CleanBullet();
		exit(0);


	case 's': // アニメーション切り替え
		//cout << g_bIdle << endl;

		if (soccer_ball && power[0] == 0 && power[1] == 0 && power[2] == 0)
		{
			error = 1;
			error_message = "error : 'power' value is not decided.";
		}
		else if (g_bIdle == 0 && soccer_ball)
		{
			btVector3 unit_v = power;
			cout << unit_v.normalize() << endl;

			//疑似回転のための法線ベクトルを取得
			float v[2];
			v[0] = unit_v[0];
			v[1] = unit_v[2];
			float l = sqrt(v[0] * v[0] + v[1] * v[1]);
			v[0] /= l;
			v[1] /= l;

			rotation_v[0] = -1 * v[1];
			rotation_v[1] = v[0];

			g_bIdle = !g_bIdle;
			soccer_ball->setLinearVelocity(btVector3(rotation * rotation_v[0], 0, rotation * rotation_v[1]));
			cout << btVector3(rotation * rotation_v[0], 0, rotation * rotation_v[1]) << endl;
			soccer_ball->applyCentralImpulse(power);
			error = 0;
		}
		break;

	case ' ': // 1ステップだけ進める
		Timer(0);
		break;

	case 'r': // ワールドリセット
		if (!g_bIdle)
		{
			g_bIdle = !g_bIdle;
		}
		ground = 1;
		rotation = 0;
		error = 0;
		cursor = 0;
		goal_message = 0;
		view_rl = 0;
		judge_rl = 0;
		view_updown = -7.0 - BALL_Z;
		table = 0;
		power[0] = 0;
		power[1] = 0;
		power[2] = 0;
		ball_pos[0] = 0;
		ball_pos[1] = 0.1;
		ball_pos[2] = BALL_Z;
		pointer_pos[0] = ball_pos[0];
		pointer_pos[1] = ball_pos[2] + 1;
		sin_b = 0;
		cos_b = 1;
		g_tbView.SetScaling(-7.0 - BALL_Z);
		g_tbView.SetTranslation(0.0, -2.0);
		g_tbView.SetQuaternion(q_reset);
		judge_fb = 0;
		judge_rl = 0;
		CleanBullet();
		InitBullet();
		break;

	case 'w': //サッカーボールを追加
		//SetArrow();
		if (!soccer_ball)
		{
			g_pDynamicsWorld->removeCollisionObject(Cursor);

			SetPointer();
			SetSoccerball();
			g_bIdle = !g_bIdle;
		}
		break;
	case 'k': //壁を追加/削除
		if (soccer_ball && !Wall)
		{
			goal = 1;
			SetWall();
			goal = 0;
		}
		else if (Wall)
		{
			g_pDynamicsWorld->removeCollisionObject(Wall);
			Wall = 0;
		}
		break;

	case 'n': //壁を左に移動
		if (Wall)
		{
			btTransform new_trans;
			Wall->getMotionState()->getWorldTransform(new_trans);
			new_trans.getOrigin() -= btVector3(abs(pointer_span * (GOAL_CENTER - BALL_Z) / GOAL_CENTER * cos_b), 0.0, pointer_span * (GOAL_CENTER - BALL_Z) / GOAL_CENTER * sin_b);
			Wall->getMotionState()->setWorldTransform(new_trans);
		}
		break;

	case 'm': //壁を右に移動
		if (Wall)
		{
			btTransform new_trans;
			Wall->getMotionState()->getWorldTransform(new_trans);
			new_trans.getOrigin() += btVector3(abs(pointer_span * (GOAL_CENTER - BALL_Z) / GOAL_CENTER * cos_b), 0.0, pointer_span * (GOAL_CENTER - BALL_Z) / GOAL_CENTER * sin_b);
			Wall->getMotionState()->setWorldTransform(new_trans);
		}
		break;

	case 'a': //左回転をかける
		if (soccer_ball)
		{
			if (rotation > -0.14)
			{
				rotation -= 0.02;
			}
			cout << "rotation : " << rotation << endl;
		}
		break;

	case 'd': //右回転をかける
		if (soccer_ball)
		{
			if (rotation < 0.14)
			{
				rotation += 0.02;
			}
			cout << "rotation : " << rotation << endl;
		}
		break;


	default:
		break;
	}
}

void SpecialKey(int key, int x, int y)
{
	switch (key)
	{

	case GLUT_KEY_UP: //上矢印キーが押されたとき
		if (Pointer && soccer_ball)
		{
			btTransform new_trans;
			Pointer->getMotionState()->getWorldTransform(new_trans);
			if (0.44 > sqrt(pow(new_trans.getOrigin()[2] - pointer_origin_pos[1], 2) + pow(new_trans.getOrigin()[0] - pointer_origin_pos[0], 2)))
			{
				new_trans.getOrigin() += btVector3(pointer_span * sin_b, 0.0, -abs(pointer_span * cos_b));
				Pointer->getMotionState()->setWorldTransform(new_trans);
			}
			if (0.44 <= sqrt(pow(new_trans.getOrigin()[2] - pointer_origin_pos[1], 2) + pow(new_trans.getOrigin()[0] - pointer_origin_pos[0], 2)))
			{
				new_trans.getOrigin() -= btVector3(pointer_span * sin_b, 0.0, -abs(pointer_span * cos_b));
				Pointer->getMotionState()->setWorldTransform(new_trans);
			}

			fire_v[0] = new_trans.getOrigin()[0] - pointer_origin_pos[0]; //x方向
			fire_v[1] = new_trans.getOrigin()[2] - pointer_origin_pos[1]; //z方向

			power[0] = 40 * fire_v[0];
			power[2] = 40 * fire_v[1];
			power[1] = sqrt(power[0] * power[0] + power[2] * power[2]) / 2;
		}
		else if (Cursor)
		{
			judge_fb = 1;
			btTransform new_trans;
			Cursor->getMotionState()->getWorldTransform(new_trans);
			new_trans.getOrigin() += btVector3(span * sin_b, 0.0, -abs(span * cos_b));
			Cursor->getMotionState()->setWorldTransform(new_trans);
			ball_pos = new_trans.getOrigin();


			ball_to_goal[0] = 0 - ball_pos[0];
			ball_to_goal[1] = GOAL_CENTER - ball_pos[2];

			pointer_pos[0] = ball_pos[0] - ball_to_goal[0] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			pointer_pos[1] = ball_pos[2] - ball_to_goal[1] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);

			sin_b = ball_to_goal[0] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			cos_b = ball_to_goal[1] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			cout << ball_to_goal[1] << endl;


			view_updown += span;
			g_tbView.SetScaling(view_updown);
		}
		break;

	case GLUT_KEY_DOWN:
		if (Pointer && soccer_ball)
		{
			btTransform new_trans;
			Pointer->getMotionState()->getWorldTransform(new_trans);
			if (0.44 > sqrt(pow(new_trans.getOrigin()[2] - pointer_origin_pos[1], 2) + pow(new_trans.getOrigin()[0] - pointer_origin_pos[0], 2)))
			{
				new_trans.getOrigin() -= btVector3(pointer_span * sin_b, 0.0, -abs(pointer_span * cos_b));
				Pointer->getMotionState()->setWorldTransform(new_trans);
			}
			if (0.44 <= sqrt(pow(new_trans.getOrigin()[2] - pointer_origin_pos[1], 2) + pow(new_trans.getOrigin()[0] - pointer_origin_pos[0], 2)))
			{
				new_trans.getOrigin() += btVector3(pointer_span * sin_b, 0.0, -abs(pointer_span * cos_b));
				Pointer->getMotionState()->setWorldTransform(new_trans);
			}

			fire_v[0] = new_trans.getOrigin()[0] - pointer_origin_pos[0];
			fire_v[1] = new_trans.getOrigin()[2] - pointer_origin_pos[1];
			cout << "x:" << fire_v[0] << " z:" << fire_v[1] << endl;


			power[0] = 40 * fire_v[0];
			power[2] = 40 * fire_v[1];
			power[1] = sqrt(power[0] * power[0] + power[2] * power[2]) / 2;
		}
		else if (Cursor)
		{
			judge_fb = 1;
			btTransform new_trans;
			Cursor->getMotionState()->getWorldTransform(new_trans);
			new_trans.getOrigin() -= btVector3(span * sin_b, 0.0, -abs(span * cos_b));
			Cursor->getMotionState()->setWorldTransform(new_trans);
			ball_pos = new_trans.getOrigin();


			ball_to_goal[0] = 0 - ball_pos[0];
			ball_to_goal[1] = GOAL_CENTER - ball_pos[2];

			pointer_pos[0] = ball_pos[0] - ball_to_goal[0] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			pointer_pos[1] = ball_pos[2] - ball_to_goal[1] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);

			sin_b = ball_to_goal[0] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			cos_b = ball_to_goal[1] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);


			view_updown -= span;
			g_tbView.SetScaling(view_updown);
		}
		break;

	case GLUT_KEY_RIGHT: //右矢印キーが押されたとき
		if (Pointer && soccer_ball)
		{
			btTransform new_trans;
			Pointer->getMotionState()->getWorldTransform(new_trans);
			if (0.44 > sqrt(pow(new_trans.getOrigin()[2] - pointer_origin_pos[1], 2) + pow(new_trans.getOrigin()[0] - pointer_origin_pos[0], 2)))
			{
				new_trans.getOrigin() += btVector3(abs(pointer_span * cos_b), 0.0, pointer_span * sin_b);
				Pointer->getMotionState()->setWorldTransform(new_trans);
			}
			if (0.44 <= sqrt(pow(new_trans.getOrigin()[2] - pointer_origin_pos[1], 2) + pow(new_trans.getOrigin()[0] - pointer_origin_pos[0], 2)))
			{
				new_trans.getOrigin() -= btVector3(abs(pointer_span * cos_b), 0.0, pointer_span * sin_b);
				Pointer->getMotionState()->setWorldTransform(new_trans);
			}

			fire_v[0] = new_trans.getOrigin()[0] - pointer_origin_pos[0];
			fire_v[1] = new_trans.getOrigin()[2] - pointer_origin_pos[1];
			cout << "x:" << fire_v[0] << " z:" << fire_v[1] << endl;

			power[0] = 40 * fire_v[0];
			power[2] = 40 * fire_v[1];
			power[1] = sqrt(power[0] * power[0] + power[2] * power[2]) / 2;
		}
		else if (Cursor && judge_fb == 0)
		{
			judge_rl = 1;
			btTransform new_trans;

			Cursor->getMotionState()->getWorldTransform(new_trans);

			new_trans.getOrigin() += btVector3(abs(span * (GOAL_CENTER - BALL_Z) / GOAL_CENTER * cos_b), 0.0, span * (GOAL_CENTER - BALL_Z) / GOAL_CENTER * sin_b);
			Cursor->getMotionState()->setWorldTransform(new_trans);
			ball_pos = new_trans.getOrigin();

			view_rl += span * cos_b;
			g_tbView.SetTranslation(view_rl, -2.0);

			view_updown += span * sin_b;
			g_tbView.SetScaling(view_updown);

			ball_to_goal[0] = 0 - ball_pos[0];
			ball_to_goal[1] = GOAL_CENTER - ball_pos[2];

			pointer_pos[0] = ball_pos[0] - ball_to_goal[0] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			pointer_pos[1] = ball_pos[2] - ball_to_goal[1] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);

			sin_b = ball_to_goal[0] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			cos_b = ball_to_goal[1] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);

			if (ball_pos[0] < 0)
			{
				eye = -1;
			}
			else
			{
				eye = 1;
			}

			q[2] = eye * sqrt((1 + cos_b) / 2);
			q[3] = 0;
			q[0] = -sqrt((1 - cos_b) / 2);
			q[1] = 0;
			g_tbView.SetQuaternion(q);
		}

		break;

	case GLUT_KEY_LEFT: //左矢印キーが押されたとき
		if (Pointer && soccer_ball)
		{
			btTransform new_trans;
			Pointer->getMotionState()->getWorldTransform(new_trans);
			if (0.44 > sqrt(pow(new_trans.getOrigin()[2] - pointer_origin_pos[1], 2) + pow(new_trans.getOrigin()[0] - pointer_origin_pos[0], 2)))
			{
				new_trans.getOrigin() -= btVector3(abs(pointer_span * cos_b), 0.0, pointer_span * sin_b);
				Pointer->getMotionState()->setWorldTransform(new_trans);
			}
			if (0.44 <= sqrt(pow(new_trans.getOrigin()[2] - pointer_origin_pos[1], 2) + pow(new_trans.getOrigin()[0] - pointer_origin_pos[0], 2)))
			{
				new_trans.getOrigin() += btVector3(abs(pointer_span * cos_b), 0.0, pointer_span * sin_b);
				Pointer->getMotionState()->setWorldTransform(new_trans);
			}

			fire_v[0] = new_trans.getOrigin()[0] - pointer_origin_pos[0];
			fire_v[1] = new_trans.getOrigin()[2] - pointer_origin_pos[1];
			cout << "x:" << fire_v[0] << " z:" << fire_v[1] << endl;

			power[0] = 40 * fire_v[0];
			power[2] = 40 * fire_v[1];
			power[1] = sqrt(power[0] * power[0] + power[2] * power[2]) / 2;
		}
		else if (Cursor && judge_fb == 0)
		{
			judge_rl = 1;
			btTransform new_trans;

			Cursor->getMotionState()->getWorldTransform(new_trans);

			new_trans.getOrigin() -= (btVector3(abs(span * (GOAL_CENTER - BALL_Z) / GOAL_CENTER * cos_b), 0.0, span * (GOAL_CENTER - BALL_Z) / GOAL_CENTER * sin_b));
			Cursor->getMotionState()->setWorldTransform(new_trans);

			ball_pos = new_trans.getOrigin();

			view_rl -= span * cos_b;
			g_tbView.SetTranslation(view_rl, -2.0);

			view_updown -= span * sin_b;
			g_tbView.SetScaling(view_updown);

			ball_to_goal[0] = 0 - ball_pos[0];
			ball_to_goal[1] = GOAL_CENTER - ball_pos[2];


			pointer_pos[0] = ball_pos[0] - ball_to_goal[0] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			pointer_pos[1] = ball_pos[2] - ball_to_goal[1] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);

			sin_b = ball_to_goal[0] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);
			cos_b = ball_to_goal[1] / sqrt(ball_to_goal[0] * ball_to_goal[0] + ball_to_goal[1] * ball_to_goal[1]);

			if (ball_pos[0] > 0)
			{
				eye = 1;
			}
			else if (ball_pos[0] < 0)
			{
				eye = -1;
			}

			q[2] = eye * sqrt((1 + cos_b) / 2);
			q[3] = 0;
			q[0] = -sqrt((1 - cos_b) / 2);
			q[1] = 0;

			cout << q[0] << ":" << q[1] << ":" << q[2] << ":" << q[3] << endl;
			g_tbView.SetQuaternion(q);
		}
		break;
	}
	glutPostRedisplay();
}

/*!
 * OpenGLの初期化
 */
void InitGL(void)
{
	// OpenGLのバージョンチェック
	printf("OpenGL Ver. %s\n", glGetString(GL_VERSION));

	// 背景色
	glClearColor(0.8, 0.8, 0.9, 1.0);

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// 光源の初期設定
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, RX_LIGHT_DIFF);
	glLightfv(GL_LIGHT0, GL_SPECULAR, RX_LIGHT_SPEC);
	glLightfv(GL_LIGHT0, GL_AMBIENT, RX_LIGHT_AMBI);
	glLightfv(GL_LIGHT0, GL_POSITION, RX_LIGHT0_POS);

	//glShadeModel(GL_SMOOTH);

	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);

	// 視点の初期化
	g_tbView.SetScaling(-7.0 - BALL_Z);
	g_tbView.SetTranslation(0.0, -2.0);

	// シャドウマップ初期化
	g_ShadowMap.InitShadow(g_iShadowMapSize, g_iShadowMapSize);

	// Bullet初期化
	InitBullet();
}

/*!
 * メインルーチン
 * @param[in] argc コマンドライン引数の数
 * @param[in] argv コマンドライン引数
 */
int main(int argc, char *argv[])
{
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(g_iWinW, g_iWinH);
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow(argv[0]);

	// イベントハンドラの設定
	glutDisplayFunc(Display);
	glutReshapeFunc(Resize);
	glutMouseFunc(Mouse);

	glutMotionFunc(Motion);
	glutKeyboardFunc(Keyboard);
	glutSpecialFunc(SpecialKey);
	//glutIdleFunc(Idle);
	glutTimerFunc(DT * 1000, Timer, 0);
	g_bIdle = true;

	InitGL();

	glutMainLoop();

	CleanBullet();

	return 0;
}
