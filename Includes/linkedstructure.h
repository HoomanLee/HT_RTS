#ifndef __LINKEDSTRUCTURE_H__
#define __LINKEDSTRUCTURE_H__

#pragma once 

#ifdef WIN32
#define NOMINMAX
#include <Windows.h>
#endif

//#include <vector>
//
//#include <Dense>

#include "Precompiled.h" // 컴파일 시간 단축을 위해..

#include "..\3rdparty\GL\Includes\glut.h"

using namespace Eigen;

struct Color
{
  float r;
  float g;
  float b;
  float a;
  
  inline void apply() { glColor4f(r, g, b, a); }
};

class Link
{
public:
  
    Link(Color c)
    {
        mObj = gluNewQuadric();
        gluQuadricDrawStyle(mObj, GLU_FILL);
        gluQuadricOrientation(mObj, GLU_OUTSIDE);
        gluQuadricNormals(mObj, GLU_SMOOTH);
	
	mColor = c;
    }
    
    float mLength;
    float mAngle;
    GLUquadricObj *mObj;
    Color mColor;
};

class LinkedStructure
{

private:

    std::vector<Link*> mList;
    VectorXf mBasePosition;
    VectorXf mPosition;
    VectorXf mTargetPosition;
    float mStep;
    bool mResolveTarget;
    Color mColor;
    
    void calculatePosition();
    MatrixXf jacobian();
    MatrixXf pseudoInverse();

    
public:

    LinkedStructure()
    {
        mBasePosition = VectorXf::Zero(2, 1);
        mPosition = VectorXf::Zero(2, 1);
	mTargetPosition = VectorXf::Zero(2, 1);
	mStep = 0.5f;
	mResolveTarget = false;
    }
    void draw();
    void moveToPoint(const VectorXf position);
    void moveBy(float dx, float dy);
    void update();
    bool isTargetResolved();
    VectorXf getPointWithinRange();
    
    inline void setStep(float step) { step = mStep; }
    inline void addLink(Link *link) { mList.push_back(link); calculatePosition();}
    inline VectorXf getPosition() { return mPosition; }
};

#endif // LINKEDSTRUCTURE_H
