#ifndef __MotionPath_H__
#define __MotionPath_H__

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

/* --------------------------------------------------------------------------
 *  P U B L I C   D E F I N I T I O N S
 */

/* --------------------------------------------------------------------------
 *  P U B L I C   I N T E R F A C E S
 */

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * 運動状態。
	 */
	typedef struct
	{
		/**
		 * 位置[m]。
		 */
		double x;
		/**
		 * 速度[m/s]。
		 */
		double v;
		/**
		 * 加速度[m/s^2]。
		 */
		double a;
	} MotionPath_MotionStates;

	/**
	 * 運動軌跡領域諸元、
	 * 運動軌跡中のある1つの領域に対する諸元。
	 */
	typedef struct
	{
		/**
		 * 差分時間[sec]。
		 */
		double dt;

		/**
		 * 差分距離[m]。
		 */
		double dx;

		/**
		 * 初速度[m/s]。
		 */
		double vs;
		/**
		 * 初加速度[m/s^2]。
		 */
		double as;

		/**
		 * ジャーク[m/s^3]。
		 */
		double j;

	} MotionPath_RegionSpecs;

	/**
	 * 運動軌跡。
	 */
	typedef struct
	{
		/**
		 * 開始時の時間[sec]。
		 */
		double TimAtStart;
		/**
		 * 開始時の位置[m]。
		 */
		double LocAtStart;
		/**
		 * 開始時の速度[m/s]。
		 */
		double VelAtStart;
		/**
		 * 開始時の加速度[m/s^2]。
		 */
		double AccAtStart;
		/**
		 * 終了時の位置[m]。
		 */
		double LocAtEnd;
		/**
		 * 終了時の速度[m/s]。
		 */
		double VelAtEnd;
		/**
		 * 終了時の加速度[m/s^2]。
		 */
		double AccAtEnd;

		/**
		 * 領域ごとの諸元。
		 * [0]:開始時ジャーク中
		 * [1]:開始時定加速中
		 * [2]:開始->巡航ジャーク中
		 * [3]:巡航中
		 * [4]:巡航->終了ジャーク中
		 * [5]:終了時定加速中
		 * [6]:終了時ジャーク中
		 */
		MotionPath_RegionSpecs RegionSpecs[7];
	} MotionPath;

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
		double s, double e);

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
		double a0);

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
		double v0);

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
		double x0, double xsign);

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
		MotionPath_MotionStates *results);

	/**
	 *  @brief 運動軌跡初期化 @n
	 *    運動軌跡を初期化する。
	 *  @param ctxt コンテキスト。
	 *  @return なし。
	 */
	void MotionPath_Init(
		MotionPath *ctxt);

	/**
	 *  @brief 二次方程式解計算 @n
	 *    二次方程式の解を計算する。
	 *  @param a 2乗項の係数a。
	 *  @param b 1乗項の係数b。
	 *  @param c 定数項c。
	 *  @return 二次方程式の解。
	 */
	double QuadraticRoots(double a, double b, double c);

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
		MotionPath *ctxt);

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
		MotionPath *ctxt);

	/**
	 *  @brief 総動作時間取得 @n
	 *    総動作時間を取得する。
	 *  @param ctxt 運動軌跡コンテキスト。
	 *  @return 総動作時間[sec]。
	 */
	double MotionPath_TotalTime(
		const MotionPath *ctxt);

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
		const MotionPath *ctxt);

#ifdef _UNIT_TEST
	void MotionPath_UnitTest(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
