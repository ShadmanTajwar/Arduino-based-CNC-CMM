#include <AccelStepper.h> //https://www.airspayce.com/mikem/arduino/AccelStepper/
#include <MultiStepper.h>
#include <PrintEx.h> //https://github.com/Chris--A/PrintEx#printex-library-for-arduino-

PrintEx print = Serial;

int dirPin[3]= {5, 6, 7};
int stepPin[3]= {2, 3, 4}; 
int switchPin[3]= {9, 10, 11}; //limit switches for x, y, z respectively
int dir[3]= {0, 1, 1}; // homing direction
int n= 0;
float steps_per_mm= 50; //steps_per_rev*microstepping/80, 80mm per rev of rod, steps_per_rev= 200 at full step
float x= 0;
float y= 0;
float z= 0;

float dx= 30; // interval, distance between readings
float dy= 30; // interval, distance between readings
float dz= 20; // z travel between readings

float x0= 45; // start from
float y0= 0;
float z0= 0;

int xlim= 30;// no of columns
int ylim= 30;// no of rows

float v[100]= {}; //v, w size must be more than no of columns/rows
float w[100]= {};
int xc= 0;
int yc= 0;
float ba[3]= {0, 0, 0};
float ca[3]= {0, 0, 0};
float N[3]= {0, 0, 0};
float uv[3]= {0, 0, 0};
AccelStepper stepperX = AccelStepper(1, stepPin[0], dirPin[0]); //pull, dir
AccelStepper stepperY = AccelStepper(1, stepPin[1], dirPin[1]);
AccelStepper stepperZ = AccelStepper(1, stepPin[2], dirPin[2]);

MultiStepper steppers;

void setup() {
  Serial.begin(9600);
  pinMode(dirPin[0], OUTPUT);
  pinMode(stepPin[0], OUTPUT);
  delay(3000);

  while(n<1)
  {
    pinMode(dirPin[n], OUTPUT);
    pinMode(stepPin[n], OUTPUT);
    pinMode(switchPin[n], INPUT);
    n++; 
  }
  n=0; 

  while(n<3)
  {
    while(digitalRead(switchPin[n])!=LOW)
    {
      digitalWrite(dirPin[n],dir[n]);
      digitalWrite(stepPin[n],HIGH);
      digitalWrite(stepPin[n],LOW);
      delayMicroseconds(600);
    }

    while(digitalRead(switchPin[n])==LOW)
    {
      digitalWrite(dirPin[n],!dir[n]);
      digitalWrite(stepPin[n],HIGH);
      digitalWrite(stepPin[n],LOW);
      delayMicroseconds(800);
    }
    delay(2000);
    n++;
  }
  
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);

  stepperX.setMaxSpeed(1600);
  stepperY.setMaxSpeed(1600);
  stepperZ.setMaxSpeed(1600);

  steppers.addStepper(stepperX);
  steppers.addStepper(stepperY);
  steppers.addStepper(stepperZ);

  z= 100;
  x= x0;
  y= y0;
  G01(x, y, z);
  delay(1000);

  print.printf("solid mazdatest1\n");
  }

void loop() {
  while (xc< xlim)
  {
    while( digitalRead(switchPin[2])== LOW)
    {
      z-= 0.1;
      G01( x, y, z);
    }

    if(xc== 0)
    {
      if(z<=0)
      {
        v[yc]= 0;
      }
      else
      {
        v[yc]= z;
      }
    }

    else if(xc> 0 && yc== 0)
    { 
      if(z<=0)
      {
        w[0]= 0;
      }
      else
      {
        w[0]= z;
      }
      w[0]= z;

      float A[3]= {x, y, w[0]};
      float B[3]= {x-dx, y, v[0]};
      float C[3]= {x-dx, y+dy, v[1]};

      subtractMatrices(B, A, ca);
      subtractMatrices(C, A, ba);
      crossProduct(ba, ca, N);
      normalize(N, uv);

      print.printf("  facet normal %.4f %.4f %.4f\n", uv[0], uv[1], uv[2]);
      print.printf("    outer loop\n");
      print.printf("      vertex %07.2f %07.2f %07.2f\n", A[0], A[1], A[2]);
      print.printf("      vertex %07.2f %07.2f %07.2f\n", B[0], B[1], B[2]);
      print.printf("      vertex %07.2f %07.2f %07.2f\n", C[0], C[1], C[2]);
      print.printf("    endloop\n");
      print.printf("  endfacet\n");
    }

    else if(xc> 0 && yc== 1)
    {
      if(z<=0)
      {
        w[1]= 0;
      }
      else
      {
        w[1]= z;
      }

      float A[3]= {x, y, w[1]};
      float B[3]= {x, y-dy, w[0]};
      float C[3]= {x-dx, y, v[1]};

      subtractMatrices(B, A, ca);
      subtractMatrices(C, A, ba);
      crossProduct(ba, ca, N);
      normalize(N, uv);

      print.printf("  facet normal %.4f %.4f %.4f\n", uv[0], uv[1], uv[2]);
      print.printf("    outer loop\n");
      print.printf("      vertex %07.2f %07.2f %07.2f\n", A[0], A[1], A[2]);
      print.printf("      vertex %07.2f %07.2f %07.2f\n", B[0], B[1], B[2]);
      print.printf("      vertex %07.2f %07.2f %07.2f\n", C[0], C[1], C[2]);
      print.printf("    endloop\n");
      print.printf("  endfacet\n");
    }

    else
    {
      if(z<=0)
      {
        w[yc]= 0;
      }
      else
      {
        w[yc]= z;
      }

      float A[3]= {x, y-dy, w[yc-1]};
      float B[3]= {x-dx, y-dy, v[yc-1]};
      float C[3]= {x-dx, y, v[yc]};

      subtractMatrices(B, A, ca);
      subtractMatrices(C, A, ba);
      crossProduct(ba, ca, N);
      normalize(N, uv);

      print.printf("  facet normal %.4f %.4f %.4f\n", uv[0], uv[1], uv[2]);
      print.printf("    outer loop\n");
      print.printf("      vertex %07.2f %07.2f %07.2f\n", A[0], A[1], A[2]);
      print.printf("      vertex %07.2f %07.2f %07.2f\n", B[0], B[1], B[2]);
      print.printf("      vertex %07.2f %07.2f %07.2f\n", C[0], C[1], C[2]);
      print.printf("    endloop\n");
      print.printf("  endfacet\n");

      A[0]= x;
      A[1]= y;
      A[2]= w[yc];

      B[0]= x;
      B[1]= y-dy;
      B[2]= w[yc-1];

      C[0]= x-dx;
      C[1]= y;
      C[2]= v[yc];

      subtractMatrices(B, A, ca);
      subtractMatrices(C, A, ba);
      crossProduct(ba, ca, N);
      normalize(N, uv);

      print.printf("  facet normal %.4f %.4f %.4f\n", uv[0], uv[1], uv[2]);
      print.printf("    outer loop\n");
      print.printf("      vertex %07.2f %07.2f %07.2f\n", A[0], A[1], A[2]);
      print.printf("      vertex %07.2f %07.2f %07.2f\n", B[0], B[1], B[2]);
      print.printf("      vertex %07.2f %07.2f %07.2f\n", C[0], C[1], C[2]);
      print.printf("    endloop\n");
      print.printf("  endfacet\n");
    }

    delay(100);
    z+= dz;
    G01( x, y, z);
    yc++;

    if(yc< ylim)
    {
      y+= dy;
      G01(x, y, z);
    }

    else
    {
      z=70;
      G01(x, y, z);
      
      y= y0;
      yc= 0;
      x+= dx;

      if(xc>0)
      {
        for(n= 0; n< ylim; n++)
        {
          v[n]= w[n];
        }
      }
      xc++;

      if(xc< xlim)
      {
       G01(x, y, z); 
      }
    }
  }

  print.printf("endsolid");
  x= 150;
  y= 150;
  z= 100;
  G01(x, y, z);
  while(1);
}

void G01(float x, float y, float z)
{
  long positions[3];

  positions[0]=round(x*steps_per_mm);
  positions[1]=-round(y*steps_per_mm);
  positions[2]=round(z*steps_per_mm);
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
}

void crossProduct(float vect_A[], float vect_B[], float result[])
{
  result[0]= vect_A[1]* vect_B[2]- vect_A[2]* vect_B[1];
  result[1]= vect_A[2]* vect_B[0]- vect_A[0]* vect_B[2];
  result[2]= vect_A[0]* vect_B[1]- vect_A[1]* vect_B[0];
}

void subtractMatrices(float vect_A[], float vect_B[], float result[])
{
  for (int i= 0; i< 3; i++) 
  {
    result[i]= vect_B[i]- vect_A[i];
  }
}

void normalize(float vect_A[], float result[])
{
  float m= sqrt((vect_A[0]* vect_A[0])+ (vect_A[1]* vect_A[1]) +(vect_A[2]* vect_A[2]));
  result[0]= vect_A[0]/m;
  result[1]= vect_A[1]/m;
  result[2]= vect_A[2]/m;
}