#include "linkedstructure.h"
#include <cmath>
#include <iostream>

#define degrees(x) x*180/3.1415926


MatrixXf LinkedStructure::pseudoInverse()
{
  // Simple math that represents the mathematics
  // explained on the website to computing the
  // pseudo inverse. this is exactly the math
  // discussed in the tutorial!!!
    MatrixXf j = jacobian();
    MatrixXf jjtInv = (j * j.transpose());
    jjtInv = jjtInv.inverse();

    return (j.transpose() * jjtInv);
}

void LinkedStructure::calculatePosition()
{
    float angle = 0;
    mPosition = VectorXf::Zero(2,1);

    // Computing the new position in a similar way
    // that we construct the jacobian. Can be seen
    // also as some type of series
    for (int i = 0; i < mList.size(); i++)
    {
        angle += mList[i]->mAngle;
        mPosition(0) += mList[i]->mLength*cos(angle);
        mPosition(1) += -mList[i]->mLength*sin(angle);
    }
}

MatrixXf LinkedStructure::jacobian()
{
  // Here we construct the jacobian of the linked structure
  // Jacobian construction has a specific patern with
  // respect to the partial differentiation
  // hence we can construct it easily in a nested for-loop
    MatrixXf j = MatrixXf::Zero(2,mList.size());

    for (int col = 0; col < mList.size(); col++)
    {
        float angle = 0;
        for (int i = 0; i < mList.size(); i++)
        {
            angle += mList[i]->mAngle;
            if (i >= col)
            {
                j(0, col) += (-mList[i]->mLength*sin(angle));
                j(1, col) += (mList[i]->mLength*cos(angle));
            }
        }
    }
    
    return j;
}

void LinkedStructure::moveToPoint(const VectorXf position)
{
  // Here we specify the new position we want to move to
  // and set the update flag to true
  mTargetPosition = position;
  mResolveTarget = true;
}

void LinkedStructure::draw()
{
    // Drawing the linke structureo on the scren
    glPushMatrix();
    glTranslatef(mBasePosition(0),mBasePosition(1),0.0f);
    glutSolidSphere(2.0f, 20, 20);

    // going through each link and at the tip of each link
    // We draw a sphere
    for (int i = 0; i < mList.size(); i++)
    {
        glRotatef(degrees(mList[i]->mAngle), 1.0f, 0.0f, 0.0f);
	mList[i]->mColor.apply();
        gluCylinder(mList[i]->mObj, 1, 1, mList[i]->mLength, 20, 20);
        glTranslatef(0, 0, mList[i]->mLength);
	Color c = { 1.0, 0, 0};
	c.apply();
	// Draw the sphere to show the joint position
        glutSolidSphere(1.5, 20, 20);
    }

    glPopMatrix();
}

void LinkedStructure::moveBy(float dx, float dy)
{
  // Here we move by dx an dy the linked structure's tip
    VectorXf dAngles = VectorXf::Zero(mList.size(), 1);
    VectorXf dPosition = VectorXf::Zero(2, 1);

   dPosition(0) = dx;
   dPosition(1) = dy;
  
    dAngles = pseudoInverse() * dPosition;
    
    // Adding to the angles the different required
    // to move to the new position
    for (int i = 0; i < mList.size(); i++)
        mList[i]->mAngle += dAngles(i);
    
    calculatePosition();
}

void LinkedStructure::update()
{
  // Performs an update to the position of the
  // linked structure. Performs a tip move by the
  // step that has been specified
  if(isTargetResolved()) return;
  
   float x = (mTargetPosition(0) - mPosition(0) > 1) ? mStep : -mStep;
   float y = (mTargetPosition(1) + mPosition(1) > 1) ? mStep : -mStep;

  moveBy(x, y);
}

bool LinkedStructure::isTargetResolved()
{
  // Checks if at the current position of the linked
  // structure the point we are trying to resolve
  // gets resolved
  if(fabs(mPosition(0) - mTargetPosition(0)) <= 2.0f &&
      fabs(mPosition(1) + mTargetPosition(1)) <= 2.0f)
  {
      mResolveTarget = false;
      return true;
  }
   
  return false;
}

VectorXf LinkedStructure::getPointWithinRange()
{
  // Returning a random point within the range that the linkedstructure
  // can reach. This method is just for demo purposes
   VectorXf point = VectorXf::Zero(2, 1);
   float length = 0;
   for(int i = 0; i < mList.size(); i++)
   {
     length += mList[i]->mLength;
   }
   
   point(0) = ((rand() % 2 == 0) ? -1 : 1) * (rand() % (int)(length*0.7));
   point(1) = ((rand() % 2 == 0) ? -1 : 1) * (rand() % (int)(length*0.7));
   
   return point;
}
