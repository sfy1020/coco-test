#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <queue>
#include <random>
#include <vector>
#include "json\reader.h"
#include "json\document.h"
#include "Singleton.h"
#include "cocos2d.h"
#include "AppMacros.h"
#include "GlobalValues.h"
#include "ParameterLoader.h"

//------------------------------------------------------------------------
//
//  Name:   Util.h
//
//  Desc:   Sets of global functions which could be called from anywhere.
//
//  Author: Insub Im (insooneelife@naver.com)
//
//------------------------------------------------------------------------

//전역함수를 담기위한 class

class Util {
public:
	//json 파일을 읽고 parsing한 결과를 document에 담는다.
	static inline void readJSON(std::string filename, rapidjson::Document& json) {
		std::ifstream fin(filename);
		char ch;
		std::string data;

		while (1) {
			fin.get(ch);
			if (fin.eof())break;
			data = data + ch;
		}
		fin.close();
		json.Parse(data.c_str());
	}

	//ZOrder를 위한 값 생성 함수
	template <typename T, typename U>
	static inline auto zOrdering(T x, U y) -> decltype(x * y) {
		return (10000000 - (y * ((int)designResolutionSize.width + 1) + x));
	}

	//random number 생성기
	template<unsigned int T = 0, unsigned int U = 1>
	static inline auto genRand() {
		static std::random_device rd;
		static std::mt19937 mt(rd());
		static std::uniform_real_distribution<double> dist(T, std::nextafter(U, DBL_MAX));

		return dist(mt);
	}

	//random number 생성기 (구간)
	template<typename T, typename U>
	static inline decltype(auto) genRand(T begin, U end) {
		double rand = genRand<0, 1>();
		return ((end - begin) * rand + begin);
	}

	//두 실수 값이 같은지?
	template <typename T, typename U>
	static inline bool isEqual(T a, U b){
		if (fabs(a - b) < 1E-12){
			return true;
		}
		return false;
	}

	//generic square 함수
	template <typename T>
	static inline auto square(T a) -> decltype(a * a) {
		return a * a;
	}

	//generic distance^2 함수
	template<typename X1, typename Y1, typename X2, typename Y2>
	static inline auto dis2(X1 x1, Y1 y1, X2 x2, Y2 y2) -> decltype(square(x2 - x1) + square(y2 - y1)) {
		return square(x2 - x1) + square(y2 - y1);
	}

	//generic range 함수, (x1, y1)으로부터 range 내에 (x2, y2)가 있는지 검사
	template<typename X1, typename Y1, typename X2, typename Y2, typename R>
	static inline bool inRange(X1 x1, Y1 y1, X2 x2, Y2 y2, R range) {
		return dis2(x1, y1, x2, y2) <= square(range);
	}

	//generic range 함수, Vec2(x1, y1)으로부터 range 내에 Vec2(x2, y2)가 있는지 검사
	template<typename T, typename R>
	static inline bool inRange(const T& p1, const T& p2, R range) {
		return dis2(p1.x, p1.y, p2.x, p2.y) <= square(range);
	}

	template<typename T>
	static inline int sign(const T& v1, const T& v2) {
		if (v1.y * v2.x > v1.x * v2.y)
			return 1;
		else
			return -1;
	}

	//clamps the first argument between the second two
	template <class T, class U, class V>
	static inline void clamp(T& arg, const U& minVal, const V& maxVal) {
		CCASSERT((minVal < maxVal), "<Clamp>MaxVal < MinVal!");

		if (arg < (T)minVal)
			arg = (T)minVal;
		if (arg > (T)maxVal)
			arg = (T)maxVal;
	}

	//returns a random double in the range -1 < n < 1
	static inline double randomClamped() { return genRand<0, 1>() - genRand<0, 1>(); }


	//Returns true if the two circles overlap
	template <typename T>
	static inline bool TwoCirclesOverlapped(const T& c1, double r1, const T& c2, double r2) {
		double DistBetweenCenters = sqrt((c1.x - c2.x) * (c1.x - c2.x) +
			(c1.y - c2.y) * (c1.y - c2.y));

		if ((DistBetweenCenters < (r1 + r2)) || (DistBetweenCenters < fabs(r1 - r2))) {
			return true;
		}
		return false;
	}

	//S > 0 : 반시계 방향
	//S = 0 : 일직선
	//S < 0 : 시계 방향
	static inline double ccw(double x1, double y1, double x2, double y2, double x3, double y3) {
		double temp = x1 * y2 + x2 * y3 + x3 * y1;
		temp = temp - y1 * x2 - y2 * x3 - y3 * x1;
		if (temp > 0)
			return 1;
		else if (temp < 0)
			return -1;
		else
			return 0;
	}

	static inline double ccw(cocos2d::Vec2 start, cocos2d::Vec2 mid, cocos2d::Vec2 end) {
		return ccw(start.x, start.y, mid.x, mid.y, end.x, end.y);
	}

	//벡터를 인자로 받아서 (1, 0)벡터로부터 회전한 각도를 return한다.
	static inline float getAngleFromZero(const cocos2d::Vec2 &point)
	{
		cocos2d::Vec2 a = { 0.0f, 1.0f };
		cocos2d::Vec2 b = { 1.0f, 0.0f };

		float angle = MATH_RAD_TO_DEG(acos(a.dot(point) / (a.getLength() * point.getLength())));

		if (angle > 90.0f) return 360.0f - MATH_RAD_TO_DEG(acos(b.dot(point) / (b.getLength() * point.getLength())));
		else return MATH_RAD_TO_DEG(acos(b.dot(point) / (b.getLength() * point.getLength())));
	}

	//container generator
	template <typename Container, typename How>
	static inline decltype(auto) genContainer(size_t size, const How& h) {
		Container v(size);
		generate(v.begin(), v.end(), h);
		return v;
	}

	//시간 측정을 위한 inner class
	class Timer{
		int count;
	public:
		Timer() { count = clock(); }
		~Timer(){
			int n = clock();
			CCLOG("time : %d", n - count);
		}
	};

	//시간을 측정하는 함수 wrapper
	template<typename F, typename ... Types>
	static inline decltype(auto) HowLong(F f, Types&& ... args){
		Timer tm;
		return f(std::forward<Types&&>(args)...);
	}

	//heading 벡터를 인자로 받아서 8방향에 대한 적합한 방향을 생성한다.
	static inline int make_direction(cocos2d::Vec2 heading) {
		static std::vector<int> degree_map = genContainer<std::vector<int>>(361, []() {
			static int i = 0;
			return (14 - (((i++ + 22) % 360) / 45))%8;
		});

		int degree = static_cast<int>(getAngleFromZero(heading));
		CCASSERT(degree >= 0, "degree_map index can't be minus!");
		return degree_map[degree];
	}

	//x,y가 경계에 있는지 검사한다.
	template<typename T, typename U>
	static inline bool isBorder(T x, U y) {
		int BORDER_SIZE = Param.BorderSize;
		return (x <= BORDER_SIZE ||
				designResolutionSize.width - BORDER_SIZE <= x ||
				y <= BORDER_SIZE ||
				designResolutionSize.height - BORDER_SIZE <= y);
	}

private:
	Util(const Util&) = delete; // no copies
	Util& operator=(const Util&) = delete; // no self-assignments
	Util() = delete;

	virtual void thisClassNeverInstantiates() = 0;
};