#ifndef __FLUID_UPDATE_H__
#define __FLUID_UPDATE_H__

#include <QPoint>
#include <QVector2D>

struct FluidUpdate {
  FluidUpdate(const QPoint &_p, const QVector2D &_v, const bool _injectD, const bool _injectV) :
    p(_p), v(_v), injectD(_injectD), injectV(_injectV) { }
  FluidUpdate() : p(), v(), injectD(false), injectV(false) { }
  QPoint p;
  QVector2D v;
  bool injectD;
  bool injectV;
};

#endif //#ifndef __FLUID_UPDATE_H__
