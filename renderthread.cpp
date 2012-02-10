/****************************************************************************
**
** Copyright (C) 2011 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/
#include "renderthread.h"

#include <QtGui>
#include <cstdio>
#include <cmath>
#include <algorithm>

#include "fluid_update.h"

//The fluid dynamics code is based (heavily) on "Real-Time Fluid Dynamics for Games"
//by Jos Stam.

const double density_increment = 200;
const double velocity_increment = 5000;


#define IX(i, j, M, N) ((j)+(N+2)*(i))
#define SWAP(x0, x) do {double *tmp = x0; x0 = x; x = tmp; } while (0)

const double VISC = 200.0;
const double DIFF = 0.1;

void set_bnd(int M, int N, int b, double *x) {
    for(int i = 1; i <= N; i++) {
        x[IX(0, i, M, N)] = b==1 ? -x[IX(1, i, M, N)] : x[IX(1, i, M, N)];
        x[IX(M+1, i, M, N)] = b==1 ? -x[IX(M, i, M, N)] : x[IX(M, i, M, N)];
    }
    for(int i = 1; i <= M; i++) {
        x[IX(i, 0, M, N)] = b==2 ? -x[IX(i, 1, M, N)] : x[IX(i, 1, M, N)];
        x[IX(i, N+1, M, N)] = b==2 ? -x[IX(i, N, M, N)] : x[IX(i, N, M, N)];
    }
    x[IX(0, 0, M, N)] = 0.5*(x[IX(1, 0, M, N)] + x[IX(0, 1, M, N)]);
    x[IX(0, N+1, M, N)] = 0.5*(x[IX(1, N+1, M, N)] + x[IX(0, N, M, N)]);
    x[IX(M+1, 0, M, N)] = 0.5*(x[IX(M, 0, M, N)] + x[IX(M+1, 1, M, N)]);
    x[IX(M+1, N+1, M, N)] = 0.5*(x[IX(M, N+1, M, N)] + x[IX(M+1, N, M, N)]);
}

void add_source(int M, int N, double *x, double *s, double dt) {
  for(int i = 0; i < (M*N); i++)
    x[i] += dt * s[i];
}

void diffuse(int M, int N, int b, double *x, double *x0, double diff, double dt) {
    double a = dt * diff * M * N;
    for(int k = 0; k < 50; k++) {
        for(int i = 1; i <= M; i++) {
            for(int j = 1; j <= N; j++) {
                x[IX(i, j, M, N)] = (x0[IX(i, j, M, N)] + a*(x[IX(i-1, j, M, N)] + x[IX(i+1, j, M, N)] +
                                                            x[IX(i, j-1, M, N)] + x[IX(i, j+1, M, N)])) / (1+4*a);
            }
        }
        set_bnd(M, N, b, x);
    }
}

void advect(int M, int N, int b, double *d, double *d0, double *u, double *v, double dt) {
    int dtx0 = dt*N;
    int dty0 = dt*M;
    for(int i = 1; i <= M; i++) {
        for(int j = 1; j <= N; j++) {
            double x = i - (dtx0 * u[IX(i, j, M, N)]);
            double y = j - (dty0 * v[IX(i, j, M, N)]);
            if (x < 0.5) x = 0.5; if (x > (N + 0.5)) x = N + 0.5;
            int i0 = (int)x;
            int i1 = i0 + 1;
            if (y < 0.5) y = 0.5; if (y > (M + 0.5)) y = M + 0.5;
            int j0 = (int)y;
            int j1 = j0 + 1;

            double s1 = x-i0;
            double s0 = 1-s1;
            double t1 = y-j0;
            double t0 = 1-t1;

            d[IX(i, j, M, N)] = s0 * (t0*d0[IX(i0, j0, M, N)] + t1*d0[IX(i0, j1, M, N)]) + 
                                s1 * (t0*d0[IX(i1, j0, M, N)] + t1*d0[IX(i1, j1, M, N)]);
        }
    }
    set_bnd(M, N, b, d);
}

void dens_step(int M, int N, double *x, double *x0, double *u, double *v, double diff, double dt) {
    //add_source(M, N, x, x0, dt);
    SWAP(x0, x);
    diffuse(M, N, 0, x, x0, diff, dt);
    SWAP(x0, x);
    advect(M, N, 0, x, x0, u, v, dt);
}

void project(int M, int N, double *u, double *v, double *p, double *div) {
    double h = 1.0/N; //FIXME what about M?
    for(int i = 1; i <= M; i++) {
        for(int j = 1; j <= N; j++) {
            div[IX(i, j, M, N)] = -0.5 * h * (u[IX(i+1, j, M, N)] - u[IX(i-1, j, M, N)] +
                                              v[IX(i, j+1, M, N)] - v[IX(i, j-1, M, N)]);
            p[IX(i, j, M, N)] = 0;
        }
    }
    set_bnd(M, N, 0, div);
    set_bnd(M, N, 0, p);

    for(int k = 0; k < 50; k++) {
        for(int i = 1; i <= M; i++) {
            for(int j = 1; j <= N; j++) {
                p[IX(i, j, M, N)] = (div[IX(i, j, M, N)] + p[IX(i-1, j, M, N)] + p[IX(i+1, j, M, N)] +
                                                           p[IX(i, j-1, M, N)] + p[IX(i, j+1, M, N)]) / 4;
            }
        }
        set_bnd(M, N, 0, p);
    }

    for(int i = 1; i <= M; i++) {
        for(int j = 1; j <= N; j++) {
            u[IX(i, j, M, N)] -= 0.5*(p[IX(i+1, j, M, N)] - p[IX(i-1, j, M, N)])/h;
            v[IX(i, j, M, N)] -= 0.5*(p[IX(i, j+1, M, N)] - p[IX(i, j-1, M, N)])/h;
        }
    }
    set_bnd(M, N, 1, u);
    set_bnd(M, N, 2, v);
}

void vel_step(int M, int N, double *u, double *v, double *u0, double *v0, double visc, double dt) {
    //add_source(M, N, u, u0, dt);
    //add_source(M, N, v, v0, dt);
    SWAP(u0, u);
    diffuse(M, N, 1, u, u0, visc, dt);
    SWAP(v0, v);
    diffuse(M, N, 2, v, v0, visc, dt);
    project(M, N, u, v, u0, v0);
    SWAP(u0, u);
    SWAP(v0, v);
    advect(M, N, 1, u, u0, u0, v0, dt);
    advect(M, N, 2, v, v0, u0, v0, dt);
    project(M, N, u, v, u0, v0);
}

RenderThread::RenderThread(QObject *parent)
    : QThread(parent)
{
    restart = false;
    resize = false;
    abort = false;

    //fprintf(stderr, "Nulling in constructor\n");
    image_data = NULL;
    u = v = u_prev = v_prev = dens = dens_prev = NULL;
}

RenderThread::~RenderThread()
{
    mutex.lock();
    abort = true;
    condition.wakeOne();
    mutex.unlock();

    wait();
}

void RenderThread::render(QSize _resultSize, FluidUpdate update)
{
    QMutexLocker locker(&mutex);

    if(_resultSize != resultSize) {
        //fprintf(stderr, "size mismatch: %dx%d != %dx%d\n", _resultSize.height(), _resultSize.width(), resultSize.height(), resultSize.width());
        resize = true;
        resultSize = _resultSize;
    }
    updateQueue.enqueue(update);
    //fprintf(stderr, "Size is %d\n", updateQueue.size());

    if (!isRunning()) {
        start(LowPriority);
    } else {
        if(resize) {
            //fprintf(stderr, "Restarting\n");
            restart = true;
        }
        condition.wakeOne();
    }
}

void RenderThread::run()
{
    forever {
        //fprintf(stderr, "Starting forever loop\n");
        mutex.lock();
        restart = false;
        QSize windowSize = this->resultSize;
        bool resized = resize;
        resize = false;
        mutex.unlock();

        if (abort)
            return;

        int M = windowSize.height(), N = windowSize.width();
        if(resized) {
            //fprintf(stderr, "Nulling\n");
            delete[] u;
            delete[] v;
            delete[] u_prev;
            delete[] v_prev;
            delete[] dens;
            delete[] dens_prev;
            u = v = u_prev = v_prev = dens = dens_prev = NULL;

        }

        if(u == NULL) {
            //fprintf(stderr, "Newing\n");
            u = new double[(M+2)*(N+2)];
            v = new double[(M+2)*(N+2)];
            u_prev = new double[(M+2)*(N+2)];
            v_prev = new double[(M+2)*(N+2)];
            dens = new double[(M+2)*(N+2)];
            dens_prev = new double[(M+2)*(N+2)];
            image_data.resize(M*N*4);
            for(int i = 0; i < ((M+2)*(N+2)); i++) {
                u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
            }
        }

        while(1) {
            if(updateQueue.empty())
              break;

            mutex.lock();
            if(updateQueue.empty()) {
                mutex.unlock();
                break;
            }
            const FluidUpdate fu = updateQueue.dequeue();
            mutex.unlock();

            //fprintf(stderr, "(M,N)=(%d,%d), pos(%d, %d) vel(%f, %f) %01d %01d\n", M, N, fu.p.x(), fu.p.y(), fu.v.x(), fu.v.y(), fu.injectD, fu.injectV);
            if(fu.p.x() >= N || fu.p.y() >= M || fu.p.x() < 0 || fu.p.y() < 0)
              continue;
            if(fu.injectD)
                dens_prev[IX(fu.p.y()+1, fu.p.x()+1, M, N)] += density_increment;
                //dens_prev[IX(fu.p.y()+1, fu.p.x()+1, M, N)] = 50;
            if(fu.injectV) {
                u_prev[IX(fu.p.y()+1, fu.p.x()+1, M, N)] += velocity_increment / N * fu.v.x();
                v_prev[IX(fu.p.y()+1, fu.p.x()+1, M, N)] += velocity_increment / M * fu.v.y();
                //u_prev[IX(fu.p.y()+1, fu.p.x()+1, M, N)] = velocity_increment / N * fu.v.x();
                //v_prev[IX(fu.p.y()+1, fu.p.x()+1, M, N)] = velocity_increment / M * fu.v.y();
            }
        }

        if (restart)
            continue;
        //fprintf(stderr, "Before vel_step, u is %p\n", u);
        double dt = (double)timer.elapsed() / 1000.0;
        fprintf(stderr, "dt=%f\n", dt);
        timer.restart();
        vel_step(M, N, u, v, u_prev, v_prev, VISC, dt);

        if (restart)
            continue;

        //fprintf(stderr, "Before dens_step, u is %p\n", u);
        dens_step(M, N, dens, dens_prev, u, v, DIFF, dt);

        if (restart)
            continue;


        double minval = *(std::max_element(dens, dens+((M+2)*(N+2))));
        double maxval = *(std::max_element(dens, dens+((M+2)*(N+2))));
        fprintf(stderr, "min/max of dens is (%f, %f)\n", minval, maxval);
        minval = *(std::max_element(dens_prev, dens_prev+((M+2)*(N+2))));
        maxval = *(std::max_element(dens_prev, dens_prev+((M+2)*(N+2))));
        fprintf(stderr, "min/max of dens_prev is (%f, %f)\n", minval, maxval);
        minval = *(std::max_element(u, u+((M+2)*(N+2))));
        maxval = *(std::max_element(u, u+((M+2)*(N+2))));
        fprintf(stderr, "min/max of u is (%f, %f)\n", minval, maxval);
        minval = *(std::max_element(v, v+((M+2)*(N+2))));
        maxval = *(std::max_element(v, v+((M+2)*(N+2))));
        fprintf(stderr, "min/max of v is (%f, %f)\n\n", minval, maxval);

        fprintf(stderr, ".");
        QRgb *walker = reinterpret_cast<QRgb*>(image_data.data());
        double *denswalk = &dens[IX(0,1,M,N)];
        for(int i = 1; i <= M; i++) {
            ++denswalk;
            for(int j = 1; j <= N; j++) {
                int c = std::min(255, (int)((*denswalk++)*255.0));
                //if(c > 0) {
                //    fprintf(stderr, "(%d, %d) is %f (%d)\n", i, j, dens[IX(i, j, M, N)], c);
                //}
                *walker++ = qRgb(0, c, 0);
            }
            ++denswalk;
        }

        SWAP(u, u_prev);
        SWAP(v, v_prev);
        SWAP(dens, dens_prev);

        if(!restart) {
            QImage image(reinterpret_cast<const uchar *>(image_data.constData()), N, M, QImage::Format_RGB32);
            //fprintf(stderr, "Emitting\n");
            //FIXME don't copy the image every time
            emit renderedImage(image);
        }

        //mutex.lock();
        //if (!restart)
        //    condition.wait(&mutex);
        //restart = false;
        //mutex.unlock();
    }
}

uint RenderThread::rgbFromWaveLength(double wave)
{
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;

    if (wave >= 380.0 && wave <= 440.0) {
        r = -1.0 * (wave - 440.0) / (440.0 - 380.0);
        b = 1.0;
    } else if (wave >= 440.0 && wave <= 490.0) {
        g = (wave - 440.0) / (490.0 - 440.0);
        b = 1.0;
    } else if (wave >= 490.0 && wave <= 510.0) {
        g = 1.0;
        b = -1.0 * (wave - 510.0) / (510.0 - 490.0);
    } else if (wave >= 510.0 && wave <= 580.0) {
        r = (wave - 510.0) / (580.0 - 510.0);
        g = 1.0;
    } else if (wave >= 580.0 && wave <= 645.0) {
        r = 1.0;
        g = -1.0 * (wave - 645.0) / (645.0 - 580.0);
    } else if (wave >= 645.0 && wave <= 780.0) {
        r = 1.0;
    }

    double s = 1.0;
    if (wave > 700.0)
        s = 0.3 + 0.7 * (780.0 - wave) / (780.0 - 700.0);
    else if (wave <  420.0)
        s = 0.3 + 0.7 * (wave - 380.0) / (420.0 - 380.0);

    r = pow(r * s, 0.8);
    g = pow(g * s, 0.8);
    b = pow(b * s, 0.8);
    return qRgb(int(r * 255), int(g * 255), int(b * 255));
}
