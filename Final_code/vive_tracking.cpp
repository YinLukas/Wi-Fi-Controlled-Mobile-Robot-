#include "vive_tracking.h"

#define VIVE1_PIN 10
#define VIVE2_PIN 11

Vive510 vive1(VIVE1_PIN);
Vive510 vive2(VIVE2_PIN);

uint16_t vive1_x = 0, vive1_y = 0;
uint16_t vive2_x = 0, vive2_y = 0;

static uint16_t 
  x10, x11, x12, y10, y11, y12,
  x20, x21, x22, y20, y21, y22;

uint16_t med3(uint16_t a, uint16_t b, uint16_t c) {
  if (a <= b && a <= c) return (b <= c ? b : c);
  else if (b <= a && b <= c) return (a <= c ? a : c);
  else return (a <= b ? a : b);
}

void initVives() {
  vive1.begin();
  vive2.begin();
}

void processVive(Vive510 &v, 
                 uint16_t &x0, uint16_t &x1, uint16_t &x2,
                 uint16_t &y0, uint16_t &y1, uint16_t &y2,
                 uint16_t &outX, uint16_t &outY) 
{
  if (v.status() == VIVE_RECEIVING) {
    x2 = x1;  x1 = x0;  x0 = v.xCoord();
    y2 = y1;  y1 = y0;  y0 = v.yCoord();
    outX = med3(x0, x1, x2);
    outY = med3(y0, y1, y2);
  }
  else {
    v.sync(5);
    outX = outY = 0;
  }
}

void updateVives() {
  processVive(vive1, x10,x11,x12, y10,y11,y12, vive1_x, vive1_y);
  processVive(vive2, x20,x21,x22, y20,y21,y22, vive2_x, vive2_y);
}