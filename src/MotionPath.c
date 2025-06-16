/** -------------------------------------------------------------------------
 *
 *	@file	MotionPath.h
 *	@brief	Time-constrained S-curve motion path
 *	@author	H.Someya
 *	@date	2024/04/24
 *
 */
/*
MIT License

Copyright (c) 2024 Hirobumi Someya

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "MotionPath.h"

#include <string.h>
#include <math.h>
#include <float.h>
#include "nullptr.h"
#include "ArrayCap.h"

/* --------------------------------------------------------------------------
 *  P R I V A T E S
 */

#define MOTIONPATH_INLINE inline

/**
 * 浮動小数点値が0であるか判定する。
 * @param fpv 浮動小数点値
 */
#define DBL_ISZERO(fpv) (fabs((fpv)) < DBL_EPSILON)

/* --------------------------------------------------------------------------
 *  P U B L I C   I N T E R F A C E S
 */

/**
 *  @brief 値正規化 @n
 *    開始値、終了値に合わせ、値を正規化する。
 *  @param value 値。
 *  @param as 開始値。
 *  @param ae 終了値。
 *  @return 係数。
 */
double MotionPath_Normalized(
	double value,
	double s, double e)
{
	double result = value;

	// まずは正にする
	if (result < 0)
	{
		result = -result;
	}

	// 終了の方が小さければ、負の値にする
	if (e < s)
	{
		result = -result;
	}

	return result;
}

/**
 *  @brief ジャーク付き加速度計算 @n
 *    ジャークを加味した加速度計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param j ジャーク[m/s^3]。
 *  @param a0 初期加速度[m/s^2]。
 *  @return 差分時間経過後の加速度[m/s^2]。
 */
static MOTIONPATH_INLINE double MotionPath_JerkedAccel(
	double dt,
	double j,
	double a0)
{
	/*
	 * aを求める。
	 * a = (j * dt) + a0
	 */
	double a = (j * dt) + a0;
	return a;
}

/**
 *  @brief 加速度計算 @n
 *    ジャーク有無に応じた加速度計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param j ジャーク[m/s^3]。
 *  @param a0 初期加速度[m/s^2]。
 *  @return 差分時間経過後の加速度[m/s^2]。
 */
double MotionPath_Accel(
	double dt,
	double j,
	double a0)
{
	double a = a0;

	if (!DBL_ISZERO(j))
	{
		a = MotionPath_JerkedAccel(dt, j, a0);
	}

	return a;
}

/**
 *  @brief ジャークなし速度計算 @n
 *    ジャークなし(定加速度)の速度計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param a 加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @return 差分時間経過後の速度[m/s]。
 */
static MOTIONPATH_INLINE double MotionPath_NonJerkedVelocity(
	double dt,
	double a,
	double v0)
{
	/*
	 * vを求める。
	 * v = (a * dt) + v0
	 */
	double v = (a * dt) + v0;
	return v;
}

/**
 *  @brief ジャーク付き速度計算 @n
 *    ジャークを加味した速度計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param j ジャーク[m/s^3]。
 *  @param a0 初期加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @return 差分時間経過後の速度[m/s]。
 */
static MOTIONPATH_INLINE double MotionPath_JerkedVelocity(
	double dt,
	double j,
	double a0,
	double v0)
{
	/*
	 * vを求める。
	 * 式変形して計算量削減できそうな気もするが、
	 * 今回は積分を使った素直な方法で求める。
	 * a = (j * dt) + a0
	 * v = (j * 1/2 * dt^2) + (a0 * dt) + v0
	 */
	double dt2 = dt * dt;
	double v = (j * dt2 / 2.0) + (a0 * dt) + v0;
	return v;
}

/**
 *  @brief 速度計算 @n
 *    ジャーク有無に応じた速度計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param j ジャーク[m/s^3]。
 *  @param a0 初期加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @return 差分時間経過後の速度[m/s]。
 */
double MotionPath_Velocity(
	double dt,
	double j,
	double a0,
	double v0)
{
	double v = v0;

	if (!DBL_ISZERO(j))
	{
		v = MotionPath_JerkedVelocity(dt, j, a0, v0);
	}
	else
	{
		v = MotionPath_NonJerkedVelocity(dt, a0, v0);
	}

	return v;
}

/**
 *  @brief ジャークなし位置計算 @n
 *    ジャークなし(定加速度)の位置計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param a 加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @param x0 初期位置[m]。
 *  @param xsign 位置符号(位置の移動方向)。1で正方向、-1で負方向。
 *  @return 差分時間経過後の位置[m]。
 */
static MOTIONPATH_INLINE double MotionPath_NonJerkedLocation(
	double dt,
	double a,
	double v0,
	double x0, double xsign)
{
	/*
	 * xを求める。
	 * 式変形して計算量削減できそうな気もするが、
	 * というか ((v0 + v) / 2) * dt) が使えると言えば使えるのだが、
	 * 今回は積分を使った素直な方法で求める。
	 * v = (a * dt) + v0
	 * x = (1/2 * a * dt^2) + (v0 * dt) + x0
	 */
	double dt2 = dt * dt;
	double dx = (a * dt2 / 2.0) + (v0 * dt);
	dx *= xsign;
	double x = dx + x0;
	return x;
}

/**
 *  @brief ジャーク付き位置計算 @n
 *    ジャークを加味した位置計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param j ジャーク[m/s^3]。
 *  @param a0 初期加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @param x0 初期位置[m]。
 *  @param xsign 位置符号(位置の移動方向)。1で正方向、-1で負方向。
 *  @return 差分時間経過後の位置[m]。
 */
static MOTIONPATH_INLINE double MotionPath_JerkedLocation(
	double dt,
	double j,
	double a0,
	double v0,
	double x0, double xsign)
{
	/*
	 * xを求める。
	 * 式変形して計算量削減できそうな気もするが、
	 * 今回は積分を使った素直な方法で求める。
	 * a = (j * dt) + a0
	 * v = (j * 1/2 * dt^2) + (a0 * dt) + v0
	 * x = (j * 1/2 * 1/3 * dt^3) + (1/2 * a0 * dt^2) + (v0 * dt) + x0
	 */
	double dt2 = dt * dt;
	double dt3 = dt2 * dt;
	double dx = (j * dt3 / 6.0) + (a0 * dt2 / 2.0) + (v0 * dt);
	dx *= xsign;
	double x = dx + x0;
	return x;
}

/**
 *  @brief 位置計算 @n
 *    ジャーク有無に応じた位置計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param j ジャーク[m/s^3]。
 *  @param a0 初期加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @param x0 初期位置[m]。
 *  @param xsign 位置符号(位置の移動方向)。1で正方向、-1で負方向。
 *  @return 差分時間経過後の位置[m]。
 */
double MotionPath_Location(
	double dt,
	double j,
	double a0,
	double v0,
	double x0, double xsign)
{
	double x = x0;

	if (!DBL_ISZERO(j))
	{
		x = MotionPath_JerkedLocation(dt, j, a0, v0, x0, xsign);
	}
	else
	{
		x = MotionPath_NonJerkedLocation(dt, a0, v0, x0, xsign);
	}

	return x;
}

/**
 *  @brief ジャークなし運動状態計算 @n
 *    ジャークなし(定加速度)の運動状態計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param a 加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @param x0 初期位置[m]。
 *  @param xsign 位置符号(位置の移動方向)。1で正方向、-1で負方向。
 *  @return 差分時間経過後の位置[m]。
 */
static MOTIONPATH_INLINE double MotionPath_NonJerkedStates(
	double dt,
	double a,
	double v0,
	double x0, double xsign,
	MotionPath_MotionStates *results)
{
	double result = 0;

	double v = MotionPath_NonJerkedVelocity(dt, a, v0);
	double x = MotionPath_NonJerkedLocation(dt, a, v0, x0, xsign);
	if (results != nullptr)
	{
		results->x = x;
		results->v = v;
		results->a = a;
		result = x;
	}

	return result;
}

/**
 *  @brief ジャーク付き運動状態計算 @n
 *    ジャークを加味した運動状態計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param j ジャーク[m/s^3]。
 *  @param a0 初期加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @param x0 初期位置[m]。
 *  @param xsign 位置符号(位置の移動方向)。1で正方向、-1で負方向。
 *  @return 差分時間経過後の位置[m]。
 */
static MOTIONPATH_INLINE double MotionPath_JerkedStates(
	double dt,
	double j,
	double a0,
	double v0,
	double x0, double xsign,
	MotionPath_MotionStates *results)
{
	double result = 0;

	double a = MotionPath_JerkedAccel(dt, j, a0);
	double v = MotionPath_JerkedVelocity(dt, j, a0, v0);
	double x = MotionPath_JerkedLocation(dt, j, a0, v0, x0, xsign);
	if (results != nullptr)
	{
		results->x = x;
		results->v = v;
		results->a = a;
		result = x;
	}

	return result;
}

/**
 *  @brief 運動状態計算 @n
 *    ジャーク有無に応じた運動状態計算を行う。
 *  @param dt 差分時間[sec]。
 *  @param j ジャーク[m/s^3]。
 *  @param a0 初期加速度[m/s^2]。
 *  @param v0 初期速度[m/s]。
 *  @param x0 初期位置[m]。
 *  @param xsign 位置符号(位置の移動方向)。1で正方向、-1で負方向。
 *  @return 差分時間経過後の位置[m]。
 */
double MotionPath_States(
	double dt,
	double j,
	double a0,
	double v0,
	double x0, double xsign,
	MotionPath_MotionStates *results)
{
	double result = 0;

	if (!DBL_ISZERO(j))
	{
		result = MotionPath_JerkedStates(dt, j, a0, v0, x0, xsign, results);
	}
	else
	{
		result = MotionPath_NonJerkedStates(dt, a0, v0, x0, xsign, results);
	}

	return result;
}

/**
 *  @brief 運動軌跡初期化 @n
 *    運動軌跡を初期化する。
 *  @param ctxt コンテキスト。
 *  @return なし。
 */
void MotionPath_Init(
	MotionPath *ctxt)
{
	if (ctxt != nullptr)
	{
		memset(ctxt, 0, sizeof(MotionPath));
	}
}

/**
 *  @brief 運動軌跡開始入力保存 @n
 *    運動軌跡開始時の入力パラメータを保存する。
 *  @return なし。
 */
static MOTIONPATH_INLINE void MotionPath_SaveIns(
	double timAtStart,
	double locAtStart, double locAtEnd,
	double velAtStart, double velAtEnd,
	double accAtStart, double accAtEnd,
	MotionPath *ctxt)
{
	if (ctxt != nullptr)
	{
		ctxt->TimAtStart = timAtStart;
		ctxt->LocAtStart = locAtStart;
		ctxt->VelAtStart = velAtStart;
		ctxt->AccAtStart = accAtStart;
		ctxt->LocAtEnd = locAtEnd;
		ctxt->VelAtEnd = velAtEnd;
		ctxt->AccAtEnd = accAtEnd;
	}
}

/**
 *  @brief 二次方程式解計算 @n
 *    二次方程式の解を計算する。
 *  @param a 2乗項の係数a。
 *  @param b 1乗項の係数b。
 *  @param c 定数項c。
 *  @return 二次方程式の解。
 */
double QuadraticRoots(double a, double b, double c)
{
	double x = NAN;

	double D = (b * b) - (4 * a * c);
	if (DBL_ISZERO(a))
	{
		x = -(c / b);
	}
	else if (D >= 0)
	{
		x = (-b + sqrt(D)) / (2 * a);
	}

	return x;
}

/**
 *  @brief 動作時間による運動軌跡作成 @n
 *    位置および動作時間を拘束することによる、運動軌跡を作成する。
 *  @param locAtStart 開始時の位置[m]。
 *  @param locAtEnd 終了時の位置[m]。
 *  @param timAtStart 開始時の時間[sec]。
 *  @param timAtEnd 終了時の時間[sec]。
 *  @param accToCruise 巡航への加速度[m/s^2]。
 *  @param accToEnd 終了への加速度[m/s^2]。
 *  @param velAtStart 開始時の速度[m/s]。
 *  @param velAtEnd 終了時の速度[m/s]。
 *  @param accAtStart 開始時の加速度[m/s^2]。
 *  @param accAtEnd 終了時の加速度[m/s^2]。
 *  @param jerkAtStart 開始時のジャーク[m/s^3]。
 *  @param jerkToCruise 巡航へのジャーク[m/s^3]。
 *  @param jerkToEnd 終了へのジャーク[m/s^3]。
 *  @param jerkAtEnd 終了時のジャーク[m/s^3]。
 *  @param ctxt コンテキスト。
 *  @return 巡航速度[m/s]。負の場合エラー
 */
double MotionPath_CreateWithTime(
	double locAtStart, double locAtEnd,
	double timAtStart, double timAtEnd,
	double accToCruise, double accToEnd,
	double velAtStart, double velAtEnd,
	double accAtStart, double accAtEnd,
	double jerkAtStart, double jerkToCruise,
	double jerkToEnd, double jerkAtEnd,
	MotionPath *ctxt)
{
	double result = -1;

	if (ctxt != nullptr)
	{
		memset(ctxt, 0, sizeof(MotionPath));

		// パラメータを保存
		MotionPath_SaveIns(
			timAtStart,
			locAtStart, locAtEnd,
			velAtStart, velAtEnd,
			accAtStart, accAtEnd,
			ctxt);

		// 設計書の名前に置き換える
		double vss = velAtStart;
		double ass = accAtStart;
		double jss = jerkAtStart;
		double asm_ = accToCruise;
		double jse = jerkToCruise;
		double ac = 0;
		double jes = jerkToEnd;
		double aem = accToEnd;
		double vee = velAtEnd;
		double aee = accAtEnd;
		double jee = jerkAtEnd;
		double dta = timAtEnd - timAtStart;
		double dxa = fabs(locAtEnd - locAtStart);

		/*
		 * vcに関わらず計算できるものを計算する
		 */
		double dtss = 0;
		if (!DBL_ISZERO(jss))
		{
			dtss = (asm_ - ass) / jss;
		}

		double dtse = 0;
		if (!DBL_ISZERO(jse))
		{
			dtse = (ac - asm_) / jse;
		}

		double dtes = 0;
		if (!DBL_ISZERO(jes))
		{
			dtes = (aem - ac) / jes;
		}

		double dtee = 0;
		if (!DBL_ISZERO(jee))
		{
			dtee = (aee - aem) / jee;
		}

		double vsms = MotionPath_Velocity(dtss, jss, ass, vss);

		double veme = vee - MotionPath_Velocity(dtee, jee, aem, 0);

		double dxss = MotionPath_Location(dtss, jss, ass, vss, 0, 1);

		double dxee = MotionPath_Location(dtee, jee, aem, veme, 0, 1);

		/*
		 * dxsmについて
		 */
		double dtse2 = dtse * dtse;
		double csm = -((jse * dtse2) / 2) - (asm_ * dtse);

		/*
		 * dxseについて
		 */
		double dtse3 = dtse2 * dtse;
		double cse = ((jse * dtse3) / 6) + ((asm_ * dtse2) / 2);

		/*
		 * dxesについて
		 */
		double dtes2 = dtes * dtes;
		double dtes3 = dtes2 * dtes;
		double ces = ((jes * dtes3) / 6) + ((ac * dtes2) / 2);

		/*
		 * dxemについて
		 */
		double cem = ((jes * dtes2) / 2) + (ac * dtes);

		/*
		 * vc
		 * 2次方程式を解いてvcを求める。設計書を参照。
		 */
		double dtf = dtss + dtse + dtes + dtee;
		// 2次方程式の各項
		double qr_a = (asm_ - aem) / (2 * asm_ * aem);
		double qr_b = (vsms / asm_) - (veme / aem) + dtse + dtes + dta - dtf;
		double qr_c =
			(dxss) +
			(((csm * csm) - (vsms * vsms)) / (2 * asm_)) +
			(cse) +
			(csm * dtse) +
			(ces) +
			((-(cem * cem) + (veme * veme)) / (2 * aem)) +
			(dxee) +
			(-dxa);
		// 解く！
		double vc = QuadraticRoots(qr_a, qr_b, qr_c);

		/*
		 * vcが求まったことによって求まるその他要因
		 */
		double vsme = vc - MotionPath_Velocity(dtse, jse, asm_, 0);
		double dtsm = (vsme - vsms) / asm_;
		double dxsm = MotionPath_Location(dtsm, 0, asm_, vsms, 0, 1);
		double dxse = MotionPath_Location(dtse, jse, asm_, vsme, 0, 1);
		double dxes = MotionPath_Location(dtes, jes, ac, vc, 0, 1);
		double vems = MotionPath_Velocity(dtes, jes, ac, vc);
		double dtem = (veme - vems) / aem;
		double dxem = MotionPath_Location(dtem, 0, aem, vems, 0, 1);
		double dtc = dta - dtss - dtsm - dtse - dtes - dtem - dtee;
		double dxc = dxa - dxss - dxsm - dxse - dxes - dxem - dxee; // vc * dtc;

		/*
		 * 格納
		 */
		// 開始時ジャーク中
		ctxt->RegionSpecs[0].dt = dtss;
		ctxt->RegionSpecs[0].dx = dxss;
		ctxt->RegionSpecs[0].vs = vss;
		ctxt->RegionSpecs[0].as = ass;
		ctxt->RegionSpecs[0].j = jss;

		// 開始時定加速中
		ctxt->RegionSpecs[1].dt = dtsm;
		ctxt->RegionSpecs[1].dx = dxsm;
		ctxt->RegionSpecs[1].vs = vsms;
		ctxt->RegionSpecs[1].as = asm_;
		ctxt->RegionSpecs[1].j = 0;

		// 開始->巡航ジャーク中
		ctxt->RegionSpecs[2].dt = dtse;
		ctxt->RegionSpecs[2].dx = dxse;
		ctxt->RegionSpecs[2].vs = vsme;
		ctxt->RegionSpecs[2].as = asm_;
		ctxt->RegionSpecs[2].j = jse;

		// 巡航->終了ジャーク中
		ctxt->RegionSpecs[4].dt = dtes;
		ctxt->RegionSpecs[4].dx = dxes;
		ctxt->RegionSpecs[4].vs = vc;
		ctxt->RegionSpecs[4].as = ac;
		ctxt->RegionSpecs[4].j = jes;

		// 終了時定加速中
		ctxt->RegionSpecs[5].dt = dtem;
		ctxt->RegionSpecs[5].dx = dxem;
		ctxt->RegionSpecs[5].vs = vems;
		ctxt->RegionSpecs[5].as = aem;
		ctxt->RegionSpecs[5].j = 0;

		// 終了時ジャーク中
		ctxt->RegionSpecs[6].dt = dtee;
		ctxt->RegionSpecs[6].dx = dxee;
		ctxt->RegionSpecs[6].vs = veme;
		ctxt->RegionSpecs[6].as = aem;
		ctxt->RegionSpecs[6].j = jee;

		// 巡航中
		ctxt->RegionSpecs[3].dt = dtc;
		ctxt->RegionSpecs[3].dx = dxc;
		ctxt->RegionSpecs[3].vs = vc;
		ctxt->RegionSpecs[3].as = ac;
		ctxt->RegionSpecs[3].j = 0;

		result = vc;
	}

	return result;
}

/**
 *  @brief 巡航速度による運動軌跡作成 @n
 *    位置および巡航速度を拘束することによる、運動軌跡を作成する。
 *  @param locAtStart 開始時の位置[m]。
 *  @param locAtEnd 終了時の位置[m]。
 *  @param timAtStart 開始時の時間[sec]。
 *  @param velCruising 巡航速度[m/s]。
 *  @param accToCruise 巡航への加速度[m/s^2]。
 *  @param accToEnd 終了への加速度[m/s^2]。
 *  @param velAtStart 開始時の速度[m/s]。
 *  @param velAtEnd 終了時の速度[m/s]。
 *  @param accAtStart 開始時の加速度[m/s^2]。
 *  @param accAtEnd 終了時の加速度[m/s^2]。
 *  @param jerkAtStart 開始時のジャーク[m/s^3]。
 *  @param jerkToCruise 巡航へのジャーク[m/s^3]。
 *  @param jerkToEnd 終了へのジャーク[m/s^3]。
 *  @param jerkAtEnd 終了時のジャーク[m/s^3]。
 *  @param ctxt コンテキスト。
 *  @return 巡航中の差分時間[sec]。負の場合エラー
 */
double MotionPath_CreateWithVelocity(
	double locAtStart, double locAtEnd,
	double timAtStart,
	double velCruising,
	double accToCruise, double accToEnd,
	double velAtStart, double velAtEnd,
	double accAtStart, double accAtEnd,
	double jerkAtStart, double jerkToCruise,
	double jerkToEnd, double jerkAtEnd,
	MotionPath *ctxt)
{
	double result = -1;

	if (ctxt != nullptr)
	{
		memset(ctxt, 0, sizeof(MotionPath));

		// パラメータを保存
		MotionPath_SaveIns(
			timAtStart,
			locAtStart, locAtEnd,
			velAtStart, velAtEnd,
			accAtStart, accAtEnd,
			ctxt);

		// 開始から巡航への加速諸元を計算する
		double dxtc = 0;
		{
			double vs = velAtStart;
			double ve = velCruising;
			double as = accAtStart;
			double am = accToCruise;
			double ae = 0;
			double js = jerkAtStart;
			double je = jerkToCruise;
			/*
			 * dtsを求める
			 */
			// 計算
			double dts = 0;
			if (!DBL_ISZERO(js))
			{
				dts = (am - as) / js;
			}

			/*
			 * dteを求める
			 */
			// 計算
			double dte = 0;
			if (!DBL_ISZERO(je))
			{
				dte = (ae - am) / je;
			}

			/*
			 * dtmを求める
			 */
			double vms = (((as + am) / 2.0) * dts) + vs;
			double vme = ve - (((am + ae) / 2.0) * dte);
			double dtm = 0;
			if (!DBL_ISZERO(am))
			{
				dtm = (vme - vms) / am;
			}

			/*
			 * dxsを求める、dxなのでx0項はなし
			 */
			double dxs = MotionPath_Location(dts, js, as, vs, 0, 1);

			/*
			 * 同様にdxeを求める
			 */
			double dxe = MotionPath_Location(dte, je, am, vme, 0, 1);

			/*
			 * dxmを求める、ここはjerkなし
			 */
			double dxm = MotionPath_Location(dtm, 0, am, vms, 0, 1);

			/*
			 * 格納
			 */
			// 開始時ジャーク中
			ctxt->RegionSpecs[0].dt = dts;
			ctxt->RegionSpecs[0].dx = dxs;
			ctxt->RegionSpecs[0].vs = vs;
			ctxt->RegionSpecs[0].as = as;
			ctxt->RegionSpecs[0].j = js;

			// 開始時定加速中
			ctxt->RegionSpecs[1].dt = dtm;
			ctxt->RegionSpecs[1].dx = dxm;
			ctxt->RegionSpecs[1].vs = vms;
			ctxt->RegionSpecs[1].as = am;
			ctxt->RegionSpecs[1].j = 0;

			// 開始->巡航ジャーク中
			ctxt->RegionSpecs[2].dt = dte;
			ctxt->RegionSpecs[2].dx = dxe;
			ctxt->RegionSpecs[2].vs = vme;
			ctxt->RegionSpecs[2].as = am;
			ctxt->RegionSpecs[2].j = je;

			dxtc = (dxs + dxm + dxe);
		}

		// 巡航から終了への加速諸元を計算する
		double dxte = 0;
		{
			double vs = velCruising;
			double ve = velAtEnd;
			double as = 0;
			double am = accToEnd;
			double ae = accAtEnd;
			double js = jerkToEnd;
			double je = jerkAtEnd;
			/*
			 * dtsを求める
			 */
			// 計算
			double dts = 0;
			if (!DBL_ISZERO(js))
			{
				dts = (am - as) / js;
			}

			/*
			 * dteを求める
			 */
			// 計算
			double dte = 0;
			if (!DBL_ISZERO(je))
			{
				dte = (ae - am) / je;
			}

			/*
			 * dtmを求める
			 */
			double vms = (((as + am) / 2.0) * dts) + vs;
			double vme = ve - (((am + ae) / 2.0) * dte);
			double dtm = 0;
			if (!DBL_ISZERO(am) != 0)
			{
				dtm = (vme - vms) / am;
			}

			/*
			 * dxsを求める、dxなのでx0項はなし
			 */
			double dxs = MotionPath_Location(dts, js, as, vs, 0, 1);

			/*
			 * 同様にdxeを求める
			 */
			double dxe = MotionPath_Location(dte, je, am, vme, 0, 1);

			/*
			 * dxmを求める、ここはjerkなし
			 */
			double dxm = MotionPath_Location(dtm, 0, am, vms, 0, 1);

			/*
			 * 格納
			 */
			// 巡航->終了ジャーク中
			ctxt->RegionSpecs[4].dt = dts;
			ctxt->RegionSpecs[4].dx = dxs;
			ctxt->RegionSpecs[4].vs = vs;
			ctxt->RegionSpecs[4].as = as;
			ctxt->RegionSpecs[4].j = js;

			// 終了時定加速中
			ctxt->RegionSpecs[5].dt = dtm;
			ctxt->RegionSpecs[5].dx = dxm;
			ctxt->RegionSpecs[5].vs = vms;
			ctxt->RegionSpecs[5].as = am;
			ctxt->RegionSpecs[5].j = 0;

			// 終了時ジャーク中
			ctxt->RegionSpecs[6].dt = dte;
			ctxt->RegionSpecs[6].dx = dxe;
			ctxt->RegionSpecs[6].vs = vme;
			ctxt->RegionSpecs[6].as = am;
			ctxt->RegionSpecs[6].j = je;

			dxte = (dxs + dxm + dxe);
		}

		/*
		 * 加減速中の諸元は、指定されたパラメータで「決まってしまう」。
		 * 巡航中の時間を調整することで、指定された位置で終わるようにする。
		 */
		// 総移動距離
		double dx = locAtEnd - locAtStart;
		if (dx < 0)
		{
			dx = -dx;
		}
		// 巡航中に消費すべき距離
		double dxc = dx - dxtc - dxte;
		// 巡航中に消費すべき距離から、巡航中の時間を求める
		double dtc = dxc / velCruising;

		// 格納
		ctxt->RegionSpecs[3].dt = dtc;
		ctxt->RegionSpecs[3].dx = dxc;
		ctxt->RegionSpecs[3].vs = velCruising;
		ctxt->RegionSpecs[3].as = 0;
		ctxt->RegionSpecs[3].j = 0;

		result = dtc;
	}

	return result;
}

/**
 *  @brief 総動作時間取得 @n
 *    総動作時間を取得する。
 *  @param ctxt 運動軌跡コンテキスト。
 *  @return 総動作時間[sec]。
 */
double MotionPath_TotalTime(
	const MotionPath *ctxt)
{
	double t = 0;

	if (ctxt != nullptr)
	{
		for (int i = 0; i < (int)CAPACITY_OF(ctxt->RegionSpecs); i++)
		{
			t += ctxt->RegionSpecs[i].dt;
		}
	}

	return t;
}

/**
 *  @brief 指定時刻における運動状態取得 @n
 *    指定した時刻における運動状態を取得する。
 *  @param t 時刻[sec]。
 *  @param results 運動状態の格納先。
 *  @param ctxt 運動軌跡コンテキスト。
 *  @return 0:動作完了 / 正:動作中 / 負:エラー
 */
int MotionPath_MotionStates_When(
	double t,
	MotionPath_MotionStates *results,
	const MotionPath *ctxt)
{
	// 結果を初期化
	const int errRes = -64;
	int result = errRes;
	if (results != nullptr)
	{
		memset(results, 0, sizeof(MotionPath_MotionStates));
	}

	if (ctxt != nullptr)
	{
		// 位置符号
		double xsign = 1;
		if (ctxt->LocAtEnd < ctxt->LocAtStart)
		{
			xsign = -1;
		}

		// 期間開始時間
		double tps = ctxt->TimAtStart;
		// 期間終了時間
		double tpe = tps;
		// 期間開始位置
		double xps = ctxt->LocAtStart;
		// 期間終了位置
		double xpe = xps;

		/*
		 * Period before: 開始時間以前
		 */
		if (result <= errRes)
		{
			if (t < tpe)
			{
				if (results != nullptr)
				{
					// 開始時の状態を返すことにする
					results->x = ctxt->LocAtStart;
					results->v = ctxt->VelAtStart;
					results->a = ctxt->AccAtStart;
				}
				result = -1;
			}
		}

		/*
		 * Period in the regions: 各領域
		 */
		for (int ri = 0; ri < (int)CAPACITY_OF(ctxt->RegionSpecs); ri++)
		{
			// 結果が既に出ているならここで終わり
			if (result > errRes)
			{
				break;
			}
			else
			{
				// 期間を更新
				tps = tpe;
				tpe += ctxt->RegionSpecs[ri].dt;
				xps = xpe;
				xpe += (xsign * ctxt->RegionSpecs[ri].dx);

				// 期間に入っていれば演算
				if (t < tpe)
				{
					MotionPath_States(
						t - tps, // 差分時間
						ctxt->RegionSpecs[ri].j,
						ctxt->RegionSpecs[ri].as,
						ctxt->RegionSpecs[ri].vs,
						xps, xsign,
						results);
					result = (ri + 1) * 0x04;

					break;
				}
			}
		}

		/*
		 * Period after: これ以前のどこにも引っかからない場合
		 */
		if (result <= errRes)
		{
			if (results != nullptr)
			{
				// 終了時の状態を返すことにする
				results->x = ctxt->LocAtEnd;
				results->v = ctxt->VelAtEnd;
				results->a = ctxt->AccAtEnd;
			}
			result = 0;
		}
	}

	return result;
}

#ifdef _UNIT_TEST

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif
#include <stdio.h>

#define MIN_SERVO_DEGREE (0)   // Minimum angle of servo in degrees
#define MAX_SERVO_DEGREE (180) // Maximum angle of servo in degrees
#define MIN_SERVO_USEC (544)   // Minimum pulse width of servo in microseconds
#define MAX_SERVO_USEC (2400)  // Maximum pulse width of servo in microseconds
#define MAX_VELOCITY_DPS (600) // Maximum velocity in DPS(Degree Per Second)
// Maximum velocity in microseconds
#define MAX_VELOCITY_USEC (             \
	MAX_VELOCITY_DPS *                  \
	(MAX_SERVO_USEC - MIN_SERVO_USEC) / \
	(MAX_SERVO_DEGREE - MIN_SERVO_DEGREE))

double ProportionalValue(
	double sourceValue,
	double sourceRange,
	double destRange)
{
	double destValue =
		(sourceValue * destRange) / sourceRange;
	return destValue;
}

void MotionPath_UnitTest(void)
{
	double t, dt;
	double locAtStart, locAtEnd;
	double velCruising;
	double accToCruise, accToEnd;
	double velAtStart, velAtEnd;
	double accAtStart, accAtEnd;
	double jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd;

	MotionPath mp;
	MotionPath_Init(&mp);

	int whenRes;
	MotionPath_MotionStates ms;

	// -----------------------------------------
	// 1-1:544->2400,with velocity,fast
	FILE *fp = fopen("MotionPath01-01.csv", "w");

	// 開始時間
	t = 1.2;
	// 開始・終了位置
	locAtStart = MIN_SERVO_USEC;
	locAtEnd = MAX_SERVO_USEC;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = MAX_VELOCITY_USEC * 0.9;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	// 開始->巡航への加速度
	accToCruise = MotionPath_Normalized(
		velCruising / 0.15,
		velAtStart, velCruising);
	// 巡航->終了への加速度
	accToEnd = MotionPath_Normalized(
		velCruising / 0.2,
		velCruising, velAtEnd);
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = MotionPath_Normalized(
		accToCruise / 0.08,
		accAtStart, accToCruise);
	// 開始->巡航時ジャーク
	jerkToCruise = MotionPath_Normalized(
		accToCruise / 0.09,
		accToCruise, 0);
	// 巡航->終了時ジャーク
	jerkToEnd = MotionPath_Normalized(
		accToEnd / 0.10,
		0, accToEnd);
	// 終了時ジャーク
	jerkAtEnd = MotionPath_Normalized(
		accToEnd / 0.11,
		accToEnd, 0);
	MotionPath_CreateWithVelocity(
		locAtStart, locAtEnd,
		t,
		velCruising,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		&mp);
	fprintf(fp, "t,whenRes,ms.x,ms.v,ms.a\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		fprintf(
			fp, "%f,%d,%f,%f,%f\n",
			t, whenRes, ms.x, ms.v, ms.a);
		t += 0.020;
	}

	fclose(fp);

	// -----------------------------------------
	// 1-2:2400->544,with velocity,slow
	fp = fopen("MotionPath01-02.csv", "w");

	// 開始時間
	t = 1.2;
	// 開始・終了位置
	locAtStart = MAX_SERVO_USEC;
	locAtEnd = MIN_SERVO_USEC;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = MAX_VELOCITY_USEC * 0.4;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	// 開始->巡航への加速度
	accToCruise = MotionPath_Normalized(
		velCruising / 0.2,
		velAtStart, velCruising);
	// 巡航->終了への加速度
	accToEnd = MotionPath_Normalized(
		velCruising / 0.3,
		velCruising, velAtEnd);
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = MotionPath_Normalized(
		accToCruise / 0.10,
		accAtStart, accToCruise);
	// 開始->巡航時ジャーク
	jerkToCruise = MotionPath_Normalized(
		accToCruise / 0.15,
		accToCruise, 0);
	// 巡航->終了時ジャーク
	jerkToEnd = MotionPath_Normalized(
		accToEnd / 0.20,
		0, accToEnd);
	// 終了時ジャーク
	jerkAtEnd = MotionPath_Normalized(
		accToEnd / 0.25,
		accToEnd, 0);
	MotionPath_CreateWithVelocity(
		locAtStart, locAtEnd,
		t,
		velCruising,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		&mp);
	fprintf(fp, "t,whenRes,ms.x,ms.v,ms.a\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		fprintf(
			fp, "%f,%d,%f,%f,%f\n",
			t, whenRes, ms.x, ms.v, ms.a);
		t += 0.020;
	}

	fclose(fp);

	// -----------------------------------------
	// 2-1:544->2400,with time,2.0s,no jerk
	fp = fopen("MotionPath02-01.csv", "w");

	// 開始時間
	t = 1.2;
	dt = 2.0;
	// 開始・終了位置
	locAtStart = MIN_SERVO_USEC;
	locAtEnd = MAX_SERVO_USEC;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = MAX_VELOCITY_USEC * 0.4;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	// 開始->巡航への加速度
	accToCruise = MotionPath_Normalized(
		velCruising / 0.2,
		velAtStart, velCruising);
	// 巡航->終了への加速度
	accToEnd = MotionPath_Normalized(
		velCruising / 0.3,
		velCruising, velAtEnd);
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = MotionPath_Normalized(
		accToCruise / 0.08,
		accAtStart, accToCruise);
	// 開始->巡航時ジャーク
	jerkToCruise = MotionPath_Normalized(
		accToCruise / 0.09,
		accToCruise, 0);
	// 巡航->終了時ジャーク
	jerkToEnd = MotionPath_Normalized(
		accToEnd / 0.10,
		0, accToEnd);
	// 終了時ジャーク
	jerkAtEnd = MotionPath_Normalized(
		accToEnd / 0.11,
		accToEnd, 0);

	MotionPath_CreateWithTime(
		locAtStart, locAtEnd,
		t, t + dt,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		// jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		0, 0, 0, 0,
		&mp);
	fprintf(fp, "t,whenRes,ms.x,ms.v,ms.a\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		fprintf(
			fp, "%f,%d,%f,%f,%f\n",
			t, whenRes, ms.x, ms.v, ms.a);
		t += 0.020;
	}

	fclose(fp);

	// -----------------------------------------
	// 2-2:544->2400,with time,1.5s
	fp = fopen("MotionPath02-02.csv", "w");

	// 開始時間
	t = 1.2;
	dt = 1.5;
	// 開始・終了位置
	locAtStart = MIN_SERVO_USEC;
	locAtEnd = MAX_SERVO_USEC;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = MAX_VELOCITY_USEC * 0.4;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	// 開始->巡航への加速度
	accToCruise = MotionPath_Normalized(
		velCruising / 0.2,
		velAtStart, velCruising);
	// 巡航->終了への加速度
	accToEnd = MotionPath_Normalized(
		velCruising / 0.3,
		velCruising, velAtEnd);
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = MotionPath_Normalized(
		accToCruise / 0.08,
		accAtStart, accToCruise);
	// 開始->巡航時ジャーク
	jerkToCruise = MotionPath_Normalized(
		accToCruise / 0.09,
		accToCruise, 0);
	// 巡航->終了時ジャーク
	jerkToEnd = MotionPath_Normalized(
		accToEnd / 0.10,
		0, accToEnd);
	// 終了時ジャーク
	jerkAtEnd = MotionPath_Normalized(
		accToEnd / 0.11,
		accToEnd, 0);

	MotionPath_CreateWithTime(
		locAtStart, locAtEnd,
		t, t + dt,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		&mp);
	fprintf(fp, "t,whenRes,ms.x,ms.v,ms.a\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		fprintf(
			fp, "%f,%d,%f,%f,%f\n",
			t, whenRes, ms.x, ms.v, ms.a);
		t += 0.020;
	}

	fclose(fp);

	// -----------------------------------------
	// 2-3:2400->544,with time,1.5s
	fp = fopen("MotionPath02-03.csv", "w");

	// 開始時間
	t = 1.2;
	dt = 1.5;
	// 開始・終了位置
	locAtStart = MAX_SERVO_USEC;
	locAtEnd = MIN_SERVO_USEC;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = MAX_VELOCITY_USEC * 0.4;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	// 開始->巡航への加速度
	accToCruise = MotionPath_Normalized(
		velCruising / 0.2,
		velAtStart, velCruising);
	// 巡航->終了への加速度
	accToEnd = MotionPath_Normalized(
		velCruising / 0.3,
		velCruising, velAtEnd);
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = MotionPath_Normalized(
		accToCruise / 0.08,
		accAtStart, accToCruise);
	// 開始->巡航時ジャーク
	jerkToCruise = MotionPath_Normalized(
		accToCruise / 0.09,
		accToCruise, 0);
	// 巡航->終了時ジャーク
	jerkToEnd = MotionPath_Normalized(
		accToEnd / 0.10,
		0, accToEnd);
	// 終了時ジャーク
	jerkAtEnd = MotionPath_Normalized(
		accToEnd / 0.11,
		accToEnd, 0);

	MotionPath_CreateWithTime(
		locAtStart, locAtEnd,
		t, t + dt,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		&mp);
	fprintf(fp, "t,whenRes,ms.x,ms.v,ms.a\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		fprintf(
			fp, "%f,%d,%f,%f,%f\n",
			t, whenRes, ms.x, ms.v, ms.a);
		t += 0.020;
	}

	fclose(fp);

	// -----------------------------------------
	// 2-4:544->2400,with time,0.7s
	fp = fopen("MotionPath02-04.csv", "w");

	// 開始時間
	t = 1.2;
	dt = 0.7;
	// 開始・終了位置
	locAtStart = 544;
	locAtEnd = 2400;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = MAX_VELOCITY_USEC * 0.9;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	// 開始->巡航への加速度
	accToCruise = MotionPath_Normalized(
		velCruising / 0.24, // velCruisingに0.24secで達する
		velAtStart, velCruising);
	// 巡航->終了への加速度
	accToEnd = MotionPath_Normalized(
		velCruising / 0.24,
		velCruising, velAtEnd);
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = MotionPath_Normalized(
		accToCruise / 0.10, // accToCruiseに0.10secで達する
		accAtStart, accToCruise);
	// 開始->巡航時ジャーク
	jerkToCruise = MotionPath_Normalized(
		accToCruise / 0.10,
		accToCruise, 0);
	// 巡航->終了時ジャーク
	jerkToEnd = MotionPath_Normalized(
		accToEnd / 0.10,
		0, accToEnd);
	// 終了時ジャーク
	jerkAtEnd = MotionPath_Normalized(
		accToEnd / 0.10,
		accToEnd, 0);

	MotionPath_CreateWithTime(
		locAtStart, locAtEnd,
		t, t + dt,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		&mp);
	fprintf(fp, "t,whenRes,ms.x,ms.v,ms.a\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		fprintf(
			fp, "%f,%d,%f,%f,%f\n",
			t, whenRes, ms.x, ms.v, ms.a);
		t += 0.020;
	}

	fclose(fp);

	// -----------------------------------------
	// 2-5:544->1500,with time,0.7s
	fp = fopen("MotionPath02-05.csv", "w");

	// 開始時間
	t = 1.2;
	dt = 0.7;
	// 開始・終了位置
	locAtStart = 544;
	locAtEnd = 1500;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = MAX_VELOCITY_USEC * 0.9;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	// 開始->巡航への加速度
	// ※※※
	// 調整時は、ジャークを固定にし、距離が半分なら加速度を半分にする、
	// という感じで調整すると良さそう。
	// ※※※
	accToCruise = MotionPath_Normalized(
		velCruising / 0.48, // velCruisingに0.48secで達する
		velAtStart, velCruising);
	// 巡航->終了への加速度
	accToEnd = MotionPath_Normalized(
		velCruising / 0.48,
		velCruising, velAtEnd);
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = MotionPath_Normalized(
		velCruising / 0.24 / 0.10,
		accAtStart, accToCruise);
	// 開始->巡航時ジャーク
	jerkToCruise = MotionPath_Normalized(
		velCruising / 0.24 / 0.10,
		accToCruise, 0);
	// 巡航->終了時ジャーク
	jerkToEnd = MotionPath_Normalized(
		velCruising / 0.24 / 0.10,
		0, accToEnd);
	// 終了時ジャーク
	jerkAtEnd = MotionPath_Normalized(
		velCruising / 0.24 / 0.10,
		accToEnd, 0);

	MotionPath_CreateWithTime(
		locAtStart, locAtEnd,
		t, t + dt,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		&mp);
	fprintf(fp, "t,whenRes,ms.x,ms.v,ms.a\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		fprintf(
			fp, "%f,%d,%f,%f,%f\n",
			t, whenRes, ms.x, ms.v, ms.a);
		t += 0.020;
	}

	fclose(fp);

	// -----------------------------------------
	// 3-1:544->2400&544->1800,with time,0.7s
	fp = fopen("MotionPath03-01.csv", "w");

	// 開始時間
	t = 1.2;
	dt = 0.7;
	// 軸1
	// 開始・終了位置
	locAtStart = 544;
	locAtEnd = 2400;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = MAX_VELOCITY_USEC * 0.9;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	// 開始->巡航への加速度
	accToCruise = MotionPath_Normalized(
		velCruising / 0.18, // velCruisingに0.18secで達する
		velAtStart, velCruising);
	// 巡航->終了への加速度
	accToEnd = MotionPath_Normalized(
		velCruising / 0.18,
		velCruising, velAtEnd);
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = MotionPath_Normalized(
		accToCruise / 0.10, // accToCruiseに0.10secで達する
		accAtStart, accToCruise);
	// 開始->巡航時ジャーク
	jerkToCruise = MotionPath_Normalized(
		accToCruise / 0.10,
		accToCruise, 0);
	// 巡航->終了時ジャーク
	jerkToEnd = MotionPath_Normalized(
		accToEnd / 0.10,
		0, accToEnd);
	// 終了時ジャーク
	jerkAtEnd = MotionPath_Normalized(
		accToEnd / 0.10,
		accToEnd, 0);

	MotionPath_CreateWithTime(
		locAtStart, locAtEnd,
		t, t + dt,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		&mp);

	// 軸2
	// 開始・終了位置
	double locAtStart2 = 544;
	double locAtEnd2 = 1800;
	double sourceRange = fabs(locAtEnd - locAtStart);
	double destRange = fabs(locAtEnd2 - locAtStart2);
	// 開始速度、巡航速度、終了速度
	double velAtStart2 = velAtStart;
	double velCruising2 = velCruising;
	double velAtEnd2 = velAtEnd;
	// 開始時加速度
	double accAtStart2 = ProportionalValue(
		accAtStart, sourceRange, destRange);
	// 開始->巡航への加速度
	double accToCruise2 = ProportionalValue(
		accToCruise, sourceRange, destRange);
	// 巡航->終了への加速度
	double accToEnd2 = ProportionalValue(
		accToEnd, sourceRange, destRange);
	// 終了時加速度
	double accAtEnd2 = ProportionalValue(
		accAtEnd, sourceRange, destRange);
	// 開始時ジャーク
	double jerkAtStart2 = ProportionalValue(
		jerkAtStart, sourceRange, destRange);
	// 開始->巡航時ジャーク
	double jerkToCruise2 = ProportionalValue(
		jerkToCruise, sourceRange, destRange);
	// 巡航->終了時ジャーク
	double jerkToEnd2 = ProportionalValue(
		jerkToEnd, sourceRange, destRange);
	// 終了時ジャーク
	double jerkAtEnd2 = ProportionalValue(
		jerkAtEnd, sourceRange, destRange);

	MotionPath mp2;
	MotionPath_CreateWithTime(
		locAtStart2, locAtEnd2,
		t, t + dt,
		accToCruise2, accToEnd2,
		velAtStart2, velAtEnd2,
		accAtStart2, accAtEnd2,
		jerkAtStart2, jerkToCruise2, jerkToEnd2, jerkAtEnd2,
		&mp2);

	fprintf(
		fp,
		"t,whenRes(1),ms.x(1),ms.v(1),ms.a(1)"
		",whenRes(2),ms.x(2),ms.v(2),ms.a(2),\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		MotionPath_MotionStates ms2;
		int whenRes2 = MotionPath_MotionStates_When(t, &ms2, &mp2);
		fprintf(
			fp, "%f,%d,%f,%f,%f,%d,%f,%f,%f\n",
			t,
			whenRes, ms.x, ms.v, ms.a,
			whenRes2, ms2.x, ms2.v, ms2.a);
		t += 0.020;
	}

	fclose(fp);

	// -----------------------------------------
	// 4-1:0->800,with time,0.7s
	fp = fopen("MotionPath04-01.csv", "w");

	// 開始時間
	t = 0;
	dt = 0.7;
	// 開始・終了位置
	locAtStart = 0;
	locAtEnd = 800;
	// 開始速度、巡航速度、終了速度
	velAtStart = 0;
	velCruising = 0;
	velAtEnd = 0;
	// 開始時加速度
	accAtStart = 0;
	accToCruise = 10000;
	// 巡航->終了への加速度
	accToEnd = -10000;
	// 終了時加速度
	accAtEnd = 0;
	// 開始時ジャーク
	jerkAtStart = 100000;
	// 開始->巡航時ジャーク
	jerkToCruise = -100000;
	// 巡航->終了時ジャーク
	jerkToEnd = -100000;
	// 終了時ジャーク
	jerkAtEnd = 100000;

	MotionPath_CreateWithTime(
		locAtStart, locAtEnd,
		t, t + dt,
		accToCruise, accToEnd,
		velAtStart, velAtEnd,
		accAtStart, accAtEnd,
		jerkAtStart, jerkToCruise, jerkToEnd, jerkAtEnd,
		&mp);
	fprintf(fp, "t,whenRes,ms.x,ms.v,ms.a\n");
	whenRes = 1;
	while (whenRes > 0)
	{
		whenRes = MotionPath_MotionStates_When(t, &ms, &mp);
		fprintf(
			fp, "%f,%d,%f,%f,%f\n",
			t, whenRes, ms.x, ms.v, ms.a);
		t += 0.020;
	}

	fclose(fp);
}

#endif
