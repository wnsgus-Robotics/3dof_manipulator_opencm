#include <math.h>
#include <stdlib.h>
#include <stdio.h>

class wnsgus_ik
{
  public:
    wnsgus_ik(int num, int* lengt);
    bool solve(float x, float y, int* lengths);
    typedef struct 
    {
        float x; // x position of joint relative to origin
        float y; // y position of joint relative to origin
        float angle; // angle of joint (if the joint has adjacent joints or origin)
    } Joint;
    typedef struct 
    {
      Joint* joints; 
      float z;
      float angle; 
    } Chain;
    int numJoints;
    float tolerance;
    Chain* chain;
    void createChain(int* lengths);
    float distance(float x1, float y1, float x2, float y2);
    float getAngle(int joint);

};
wnsgus_ik::wnsgus_ik(int numJoints, int* lengths)
{
  this->numJoints = numJoints;
  createChain(lengths);
  this->tolerance = 1; 
}

// create chain algorithm
void wnsgus_ik::createChain(int* lengths)
{
    Chain* chain = (Chain*)malloc(sizeof(Chain));
    chain->joints = (Joint*)malloc(sizeof(Joint)*this->numJoints);
    chain->joints[0].x = 0;
    chain->joints[0].y = 0;
    chain->joints[0].angle = 0;
    int sumLengths = 0;
    for (int i = 1; i < this->numJoints; i++) 
    {
        sumLengths = sumLengths + lengths[i-1];
        chain->joints[i].x = 0;
        chain->joints[i].y = sumLengths;
        chain->joints[i].angle = 0;
    }
    this->chain = chain;
}

// 
float wnsgus_ik::distance(float x1, float y1, float x2, float y2) 
{
    float xDiff = x2-x1;
    float yDiff = y2-y1;
    return sqrt(xDiff*xDiff + yDiff*yDiff);
}

bool wnsgus_ik::solve(float x, float y, int* lengths)
{
    int dist = sqrt(x*x+y*y);
    int totalLength = 0;
    for (int i = 0; i < this->numJoints-1; i++)
    {
        totalLength = totalLength + lengths[i];
    }
    if (dist > totalLength) 
    {
       for (int i = 0; i < this->numJoints-1; i++)
       {
           float jx = this->chain->joints[i].x;
           float jy = this->chain->joints[i].y;
           float r_i = distance(jx,jy,x,y);
           float lambda_i = ((float)lengths[i])/r_i;
           this->chain->joints[i+1].x = (float)((1-lambda_i)*jx + lambda_i*x);
           this->chain->joints[i+1].y = (float)((1-lambda_i)*jy + lambda_i*y);
       }
       return false;
    }
    else 
    {
        float bx = this->chain->joints[0].x;
        float by = this->chain->joints[0].y;
        float ex = this->chain->joints[this->numJoints-1].x;
        float ey = this->chain->joints[this->numJoints-1].y;
        float dif = distance(ex,ey,x,y);
        float prevDif = 0;
        float tolerance = this->tolerance;
        while (dif > tolerance)
        {
            if (prevDif == dif)
                tolerance *= 2;
            prevDif = dif;
            this->chain->joints[this->numJoints-1].x = x;
            this->chain->joints[this->numJoints-1].y = y;
            for (int i = this->numJoints-2; i >= 0; i--) 
            {
                float jx = this->chain->joints[i].x;
                float jy = this->chain->joints[i].y;
                float nx = this->chain->joints[i+1].x;
                float ny = this->chain->joints[i+1].y;
                float r_i = distance(jx,jy,nx,ny);
                float lambda_i = ((float)lengths[i])/r_i;
                this->chain->joints[i].x = (float)((1-lambda_i)*nx + lambda_i*jx);
                this->chain->joints[i].y = (float)((1-lambda_i)*ny + lambda_i*jy);
            }
            this->chain->joints[0].x = bx;
            this->chain->joints[0].y = by;
            for (int i = 0; i < this->numJoints-1; i++)
            {
                float jx = this->chain->joints[i+1].x;
                float jy = this->chain->joints[i+1].y;
                float nx = this->chain->joints[i].x;
                float ny = this->chain->joints[i].y;
                float r_i = distance(jx,jy,nx,ny);
                float lambda_i = ((float)lengths[i])/r_i;
                this->chain->joints[i+1].x = (float)((1-lambda_i)*nx + lambda_i*jx);
                this->chain->joints[i+1].y = (float)((1-lambda_i)*ny + lambda_i*jy);
            }
            ex = this->chain->joints[this->numJoints-1].x;
            ey = this->chain->joints[this->numJoints-1].y;
            dif = distance(ex,ey,x,y);
        }
    }
    this->chain->joints[0].angle = atan2(this->chain->joints[1].y,this->chain->joints[1].x);
    float prevAngle = this->chain->joints[0].angle;
    for (int i = 2; i <= this->numJoints-1; i++)
    {
        float ax = this->chain->joints[i-1].x;
        float ay = this->chain->joints[i-1].y;
        float bx = this->chain->joints[i].x;
        float by = this->chain->joints[i].y;
        float aAngle = atan2(by-ay,bx-ax);
        this->chain->joints[i-1].angle = aAngle-prevAngle;
        prevAngle = aAngle;
    }
    return true;
}

float wnsgus_ik::getAngle(int joint)
{
  if (joint >= 0 && joint < numJoints) 
  {
      return this->chain->joints[joint].angle;
  }
  return 0;
}
