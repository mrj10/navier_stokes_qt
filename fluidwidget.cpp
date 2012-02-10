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

#include <QtGui>
#include <cmath>
#include "fluidwidget.h"
#include "fluid_update.h"

FluidWidget::FluidWidget(QWidget *parent)
    : QWidget(parent)
{
    qRegisterMetaType<QImage>("QImage");
    connect(&thread, SIGNAL(renderedImage(QImage)),
            this, SLOT(updatePixmap(QImage)));

    setWindowTitle(tr("Fluid"));
#ifndef QT_NO_CURSOR
    setCursor(Qt::CrossCursor);
#endif
    resize(1024, 1024);
}

void FluidWidget::paintEvent(QPaintEvent * /* event */)
{
    QPainter painter(this);
    //painter.fillRect(rect(), Qt::black);

    if (pixmap.isNull()) {
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter,
                         tr("Rendering initial image, please wait..."));
        return;
    }

    painter.drawPixmap(0, 0, pixmap);

    QString text = tr("Use mouse wheel or the '+' and '-' keys to zoom. "
                      "Press and hold left mouse button to scroll.");
    QFontMetrics metrics = painter.fontMetrics();
    int textWidth = metrics.width(text);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 0, 0, 127));
    painter.drawRect((width() - textWidth) / 2 - 5, 0, textWidth + 10,
                     metrics.lineSpacing() + 5);
    painter.setPen(Qt::white);
    painter.drawText((width() - textWidth) / 2,
                     metrics.leading() + metrics.ascent(), text);
}

void FluidWidget::resizeEvent(QResizeEvent * /* event */)
{
    thread.render(size()/4, FluidUpdate());
}

void FluidWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Plus:
        //zoom(ZoomInFactor);
        break;
    case Qt::Key_Minus:
        //zoom(ZoomOutFactor);
        break;
    case Qt::Key_Left:
        //scroll(-ScrollStep, 0);
        break;
    case Qt::Key_Right:
        //scroll(+ScrollStep, 0);
        break;
    case Qt::Key_Down:
        //scroll(0, -ScrollStep);
        break;
    case Qt::Key_Up:
        //scroll(0, +ScrollStep);
        break;
    default:
        QWidget::keyPressEvent(event);
    }
}

void FluidWidget::wheelEvent(QWheelEvent *event)
{
    //int numDegrees = event->delta() / 8;
    //double numSteps = numDegrees / 15.0f;
    //zoom(pow(ZoomInFactor, numSteps));
}

void FluidWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        update();
        thread.render(size()/4, FluidUpdate(event->pos()/4, QVector2D(), true, false));
    }
    lastDragPos = event->pos()/4;
}

void FluidWidget::mouseMoveEvent(QMouseEvent *event)
{
    const bool density = event->buttons() & Qt::LeftButton;
    const bool velocity = event->buttons() & Qt::RightButton;
    if (density || velocity) {
        qint64 dragTime = dragTimer.elapsed();
        dragTimer.restart();
        if(dragTime < 1)
            dragTime = 1; //Take at least 1ms
        if(dragTime > 100) //Clip to 100ms
            dragTime = 100;
        QVector2D dragMovement(event->pos() - lastDragPos);
        //Divide by dragTime if you want the vector to actually be a mouse velocity rather than just the number of pixels the mouse moved
        //dragMovement /= ((double)dragTime/1000);
        lastDragPos = event->pos();
        update();
        thread.render(size()/4, FluidUpdate(event->pos()/4, dragMovement/4, density, velocity));
    }
}

void FluidWidget::mouseReleaseEvent(QMouseEvent *event)
{
    //if (event->button() == Qt::LeftButton) {
    //    pixmapOffset += event->pos() - lastDragPos;
    //    lastDragPos = QPoint();

     //   int deltaX = (width() - pixmap.width()) / 2 - pixmapOffset.x();
     //   int deltaY = (height() - pixmap.height()) / 2 - pixmapOffset.y();
     //    scroll(deltaX, deltaY);
    //}
}

void FluidWidget::updatePixmap(const QImage &image)
{
    //if (!lastDragPos.isNull())
    //    return;

    pixmap = QPixmap::fromImage(image.scaled(size()));
    //pixmapOffset = QPoint();
    //lastDragPos = QPoint();
    update();
}

void FluidWidget::zoom(double zoomFactor)
{
    /*
    curScale *= zoomFactor;
    update();
    thread.render(size());
    */
}

void FluidWidget::scroll(int deltaX, int deltaY)
{
    /*
    centerX += deltaX * curScale;
    centerY += deltaY * curScale;
    update();
    thread.render(size());
    */
}
